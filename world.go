package physics

import (
	"fmt"
	"sort"
	"sync"
)

// Frame is an immutable snapshot of the world at a single fixed step.
//
// After publication the body and contact slices are never mutated, so
// queries may read them without holding the world lock. The query logs are
// the only mutable part and are protected by logMu.
type Frame struct {
	Number uint64
	Bodies []Body
	// Contacts is sorted by (BodyA, BodyB).
	Contacts []Contact

	logMu    sync.Mutex
	rayLogs  []RayQueryRecord
	areaLogs []AreaQueryRecord
}

// World is a deterministic fixed-timestep physics world.
//
// All state changes happen under mu and publish a brand-new Frame, so a
// query that has grabbed a frame pointer always observes one complete,
// self-consistent frame even while the world keeps stepping.
type World struct {
	mu      sync.RWMutex
	fixedDt float64
	gravity Vec2
	paused  bool

	frame *Frame
	accum float64
}

// NewWorld creates an empty world advanced with a fixed timestep of fixedDt.
func NewWorld(fixedDt float64) (*World, error) {
	if fixedDt <= 0 {
		return nil, &Error{
			Code:    ErrInvalidParam,
			Param:   "fixed_dt",
			Value:   fmt.Sprintf("%g", fixedDt),
			Message: "fixed timestep must be strictly positive",
		}
	}
	w := &World{
		fixedDt: fixedDt,
		frame:   &Frame{Number: 0},
	}
	return w, nil
}

// FixedDt returns the fixed timestep the world is advanced with.
func (w *World) FixedDt() float64 { return w.fixedDt }

// SetGravity sets the global gravity applied to dynamic bodies.
func (w *World) SetGravity(g Vec2) {
	w.mu.Lock()
	w.gravity = g
	w.mu.Unlock()
}

// Gravity returns the current gravity.
func (w *World) Gravity() Vec2 {
	w.mu.RLock()
	g := w.gravity
	w.mu.RUnlock()
	return g
}

// SetPaused pauses or resumes time advancement. Single-stepping via Step
// still works while paused.
func (w *World) SetPaused(p bool) {
	w.mu.Lock()
	w.paused = p
	w.mu.Unlock()
}

// Paused reports whether the world is currently paused.
func (w *World) Paused() bool {
	w.mu.RLock()
	p := w.paused
	w.mu.RUnlock()
	return p
}

// FrameNumber returns the number of fixed steps already executed.
func (w *World) FrameNumber() uint64 {
	w.mu.RLock()
	n := w.frame.Number
	w.mu.RUnlock()
	return n
}

// Step advances the world by exactly one fixed timestep. It always performs
// a step, even while paused, so it doubles as the single-step primitive.
func (w *World) Step() error {
	w.mu.Lock()
	defer w.mu.Unlock()

	prev := w.frame
	next := &Frame{
		Number: prev.Number + 1,
		Bodies: append([]Body(nil), prev.Bodies...),
	}
	for i := range next.Bodies {
		b := &next.Bodies[i]
		if b.Type != BodyDynamic {
			continue
		}
		// Semi-implicit Euler: integrate velocity, then position.
		b.Vel = b.Vel.Add(w.gravity.Mul(w.fixedDt))
		b.Pos = b.Pos.Add(b.Vel.Mul(w.fixedDt))
		b.Rot += b.AngVel * w.fixedDt
	}
	next.Contacts = detectContacts(next.Bodies)
	w.frame = next
	return nil
}

// Advance accumulates elapsed real time and performs fixed steps until the
// accumulator is drained. When paused, time is not advanced. A negative dt
// is rejected.
func (w *World) Advance(dt float64) error {
	if dt < 0 {
		return &Error{
			Code:    ErrInvalidParam,
			Param:   "dt",
			Value:   fmt.Sprintf("%g", dt),
			Message: "elapsed time must not be negative",
		}
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.paused {
		return nil
	}
	w.accum += dt
	for w.accum >= w.fixedDt {
		w.accum -= w.fixedDt
		prev := w.frame
		next := &Frame{
			Number: prev.Number + 1,
			Bodies: append([]Body(nil), prev.Bodies...),
		}
		for i := range next.Bodies {
			b := &next.Bodies[i]
			if b.Type != BodyDynamic {
				continue
			}
			b.Vel = b.Vel.Add(w.gravity.Mul(w.fixedDt))
			b.Pos = b.Pos.Add(b.Vel.Mul(w.fixedDt))
			b.Rot += b.AngVel * w.fixedDt
		}
		next.Contacts = detectContacts(next.Bodies)
		w.frame = next
	}
	return nil
}

// AddBody inserts a body. Duplicate or empty identifiers and degenerate
// shapes are rejected with an error.
func (w *World) AddBody(b Body) error {
	if b.ID == "" {
		return &Error{
			Code:    ErrEmptyID,
			Message: "body identifier must not be empty",
		}
	}
	if b.Type != BodyStatic && b.Type != BodyDynamic {
		return &Error{
			Code:    ErrInvalidParam,
			Object:  b.ID,
			Param:   "type",
			Value:   fmt.Sprintf("%d", b.Type),
			Message: "unknown body type",
		}
	}
	if err := validateShape(b.Shape); err != nil {
		if e, ok := err.(*Error); ok {
			e.Object = b.ID
		}
		return err
	}

	w.mu.Lock()
	defer w.mu.Unlock()
	for i := range w.frame.Bodies {
		if w.frame.Bodies[i].ID == b.ID {
			return &Error{
				Code:    ErrDuplicateID,
				Object:  b.ID,
				Message: "body identifier already exists",
			}
		}
	}
	next := w.cloneLocked()
	next.Bodies = append(next.Bodies, b)
	sort.Slice(next.Bodies, func(i, j int) bool { return next.Bodies[i].ID < next.Bodies[j].ID })
	next.Contacts = detectContacts(next.Bodies)
	w.frame = next
	return nil
}

// RemoveBody removes the body with the given identifier.
func (w *World) RemoveBody(id string) error {
	if id == "" {
		return &Error{
			Code:    ErrEmptyID,
			Message: "body identifier must not be empty",
		}
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	idx := w.findLocked(id)
	if idx < 0 {
		return &Error{
			Code:    ErrNotFound,
			Object:  id,
			Message: "body not found",
		}
	}
	next := w.cloneLocked()
	next.Bodies = append(next.Bodies[:idx], next.Bodies[idx+1:]...)
	next.Contacts = detectContacts(next.Bodies)
	w.frame = next
	return nil
}

// SetTransform changes a body's position and rotation.
func (w *World) SetTransform(id string, pos Vec2, rot float64) error {
	return w.updateLocked(id, func(b *Body) {
		b.Pos = pos
		b.Rot = rot
	})
}

// SetVelocity changes a body's linear and angular velocity.
func (w *World) SetVelocity(id string, vel Vec2, angVel float64) error {
	return w.updateLocked(id, func(b *Body) {
		b.Vel = vel
		b.AngVel = angVel
	})
}

// SetLayer changes a body's collision layer.
func (w *World) SetLayer(id string, layer uint32) error {
	return w.updateLocked(id, func(b *Body) {
		b.Layer = layer
	})
}

// SetShape replaces a body's collision shape with a validated new one.
func (w *World) SetShape(id string, s Shape) error {
	if err := validateShape(s); err != nil {
		if e, ok := err.(*Error); ok {
			e.Object = id
		}
		return err
	}
	return w.updateLocked(id, func(b *Body) {
		b.Shape = s
	})
}

// Bodies returns a copy of the current frame's bodies, sorted by identifier.
func (w *World) Bodies() []Body {
	w.mu.RLock()
	out := append([]Body(nil), w.frame.Bodies...)
	w.mu.RUnlock()
	return out
}

// currentFrame returns the latest published frame. The returned frame is
// immutable except for its query logs.
func (w *World) currentFrame() *Frame {
	w.mu.RLock()
	f := w.frame
	w.mu.RUnlock()
	return f
}

// cloneLocked creates a new frame holding copies of the current bodies. The
// caller must hold w.mu.
func (w *World) cloneLocked() *Frame {
	return &Frame{
		Number: w.frame.Number,
		Bodies: append([]Body(nil), w.frame.Bodies...),
	}
}

// findLocked returns the index of the body with id, or -1. The caller must
// hold w.mu.
func (w *World) findLocked(id string) int {
	for i := range w.frame.Bodies {
		if w.frame.Bodies[i].ID == id {
			return i
		}
	}
	return -1
}

// updateLocked clones the current frame, applies fn to the named body,
// recomputes contacts and publishes the result.
func (w *World) updateLocked(id string, fn func(*Body)) error {
	if id == "" {
		return &Error{
			Code:    ErrEmptyID,
			Message: "body identifier must not be empty",
		}
	}
	w.mu.Lock()
	defer w.mu.Unlock()
	idx := w.findLocked(id)
	if idx < 0 {
		return &Error{
			Code:    ErrNotFound,
			Object:  id,
			Message: "body not found",
		}
	}
	next := w.cloneLocked()
	fn(&next.Bodies[idx])
	next.Contacts = detectContacts(next.Bodies)
	w.frame = next
	return nil
}
