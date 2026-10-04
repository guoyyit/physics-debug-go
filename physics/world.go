package physics

import (
	"math"
	"sort"
	"sync"
	"sync/atomic"
)

// Config controls world simulation. Dt is the fixed step size; every
// call to Step advances the world by exactly Dt.
type Config struct {
	Gravity            Vec2
	Dt                 float64
	VelocityIterations int // solver iterations per step, >= 1
}

// DefaultConfig returns a sane starting configuration.
func DefaultConfig() Config {
	return Config{
		Gravity:            Vec2{0, -9.8},
		Dt:                 1.0 / 60.0,
		VelocityIterations: 10,
	}
}

// World is a deterministic 2D physics world. Mutation methods (AddBody,
// RemoveBody, SetTransform, Step, ...) are serialized internally and
// must be considered writer operations. Query methods (Raycast,
// OverlapRegion, DebugFrame) are safe to call from any goroutine at any
// time, always observe one complete frame, and never block simulation.
type World struct {
	mu       sync.Mutex // serializes writer-side state changes
	cfg      Config
	bodies   map[BodyID]*Body
	order    []BodyID // sorted ascending; all iteration uses this order
	frame    uint64
	time     float64
	paused   atomic.Bool
	contacts []*Contact // manifolds produced by the last step

	view atomicFrameView // latest published immutable frame
}

// NewWorld validates cfg and creates an empty world.
func NewWorld(cfg Config) (*World, error) {
	const op = "NewWorld"
	if math.IsNaN(cfg.Dt) || math.IsInf(cfg.Dt, 0) || cfg.Dt <= 0 {
		return nil, newError(ErrInvalidParam, op,
			"Config.Dt must be a finite number > 0, got %v", cfg.Dt)
	}
	if !cfg.Gravity.IsFinite() {
		return nil, newError(ErrInvalidParam, op,
			"Config.Gravity must be finite, got %+v", cfg.Gravity)
	}
	if cfg.VelocityIterations < 1 {
		return nil, newError(ErrInvalidParam, op,
			"Config.VelocityIterations must be >= 1, got %d", cfg.VelocityIterations)
	}
	w := &World{
		cfg:    cfg,
		bodies: make(map[BodyID]*Body),
	}
	w.publishLocked()
	return w, nil
}

// insertOrder inserts id into w.order keeping it sorted.
func (w *World) insertOrder(id BodyID) {
	i := sort.Search(len(w.order), func(i int) bool { return w.order[i] >= id })
	w.order = append(w.order, 0)
	copy(w.order[i+1:], w.order[i:])
	w.order[i] = id
}

// AddBody validates def and inserts a body. IDs must be unique.
func (w *World) AddBody(def BodyDef) error {
	w.mu.Lock()
	defer w.mu.Unlock()
	b, err := newBody(def)
	if err != nil {
		return err
	}
	if _, ok := w.bodies[b.id]; ok {
		return newError(ErrDuplicateID, "World.AddBody",
			"body ID %d already exists", b.id)
	}
	w.bodies[b.id] = b
	w.insertOrder(b.id)
	w.publishLocked()
	return nil
}

// RemoveBody deletes a body. Unknown IDs are an error, not a no-op.
func (w *World) RemoveBody(id BodyID) error {
	w.mu.Lock()
	defer w.mu.Unlock()
	if _, ok := w.bodies[id]; !ok {
		return newError(ErrBodyNotFound, "World.RemoveBody",
			"body ID %d does not exist", id)
	}
	delete(w.bodies, id)
	i := sort.Search(len(w.order), func(i int) bool { return w.order[i] >= id })
	w.order = append(w.order[:i], w.order[i+1:]...)
	w.publishLocked()
	return nil
}

// SetTransform teleports a body to a new position and angle.
func (w *World) SetTransform(id BodyID, pos Vec2, angle float64) error {
	w.mu.Lock()
	defer w.mu.Unlock()
	b, ok := w.bodies[id]
	if !ok {
		return newError(ErrBodyNotFound, "World.SetTransform",
			"body ID %d does not exist", id)
	}
	if !pos.IsFinite() || math.IsNaN(angle) || math.IsInf(angle, 0) {
		return newError(ErrInvalidParam, "World.SetTransform",
			"body %d: position and angle must be finite, got pos=%+v angle=%v",
			id, pos, angle)
	}
	b.pos = pos
	b.angle = angle
	w.publishLocked()
	return nil
}

// SetVelocity sets linear and angular velocity of a body.
func (w *World) SetVelocity(id BodyID, vel Vec2, angVel float64) error {
	w.mu.Lock()
	defer w.mu.Unlock()
	b, ok := w.bodies[id]
	if !ok {
		return newError(ErrBodyNotFound, "World.SetVelocity",
			"body ID %d does not exist", id)
	}
	if !vel.IsFinite() || math.IsNaN(angVel) || math.IsInf(angVel, 0) {
		return newError(ErrInvalidParam, "World.SetVelocity",
			"body %d: velocity and angular velocity must be finite, got vel=%+v angVel=%v",
			id, vel, angVel)
	}
	b.vel = vel
	b.angVel = angVel
	return nil
}

// Step advances the world by exactly one fixed step. It is a no-op
// while paused; use StepOnce to advance a paused world.
func (w *World) Step() {
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.paused.Load() {
		return
	}
	w.stepLocked()
}

// StepOnce advances exactly one step even while paused.
func (w *World) StepOnce() {
	w.mu.Lock()
	defer w.mu.Unlock()
	w.stepLocked()
}

// Pause halts Step (queries keep working).
func (w *World) Pause() { w.paused.Store(true) }

// Resume lets Step advance again.
func (w *World) Resume() { w.paused.Store(false) }

// Paused reports whether the world is paused.
func (w *World) Paused() bool { return w.paused.Load() }

// Frame returns the index of the most recently completed frame.
func (w *World) Frame() uint64 { return w.view.Load().frame }

// Time returns the accumulated simulation time.
func (w *World) Time() float64 { return w.view.Load().time }

// stepLocked advances the simulation by one fixed step. All iteration
// follows w.order (ascending body IDs) so results are reproducible.
func (w *World) stepLocked() {
	dt := w.cfg.Dt

	// Integrate forces (gravity) into velocities.
	for _, id := range w.order {
		b := w.bodies[id]
		if b.kind == Dynamic {
			b.vel = b.vel.Add(w.cfg.Gravity.Mul(dt))
		}
	}

	// Broadphase: sorted pairs, layer mask and AABB tests.
	var contacts []*Contact
	for i := 0; i < len(w.order); i++ {
		a := w.bodies[w.order[i]]
		for j := i + 1; j < len(w.order); j++ {
			b := w.bodies[w.order[j]]
			if !shouldCollide(a, b) {
				continue
			}
			if !a.aabb().Overlaps(b.aabb()) {
				continue
			}
			contacts = append(contacts, collide(a, b)...)
		}
	}

	// Solve.
	s := solver{iterations: w.cfg.VelocityIterations}
	s.prepare(contacts, dt)
	s.solveVelocity(contacts, dt)

	// Integrate positions.
	for _, id := range w.order {
		b := w.bodies[id]
		if b.kind == Static {
			continue
		}
		b.pos = b.pos.Add(b.vel.Mul(dt))
		b.angle += b.angVel * dt
	}

	w.frame++
	w.time += dt
	w.contacts = contacts
	w.publishLocked()
}

// publishLocked builds an immutable view of the current state and
// atomically swaps it in for readers. The previous view becomes
// garbage once in-flight queries against it finish, so memory stays
// bounded regardless of how many steps or mutations occur.
func (w *World) publishLocked() {
	fv := &frameView{
		frame:  w.frame,
		time:   w.time,
		bodies: make([]bodySnapshot, 0, len(w.order)),
	}
	for _, id := range w.order {
		b := w.bodies[id]
		fv.bodies = append(fv.bodies, bodySnapshot{
			id:          b.id,
			kind:        b.kind,
			pos:         b.pos,
			angle:       b.angle,
			layer:       b.layer,
			mask:        b.mask,
			friction:    b.friction,
			restitution: b.restitution,
			shapes:      b.shapes,
			aabb:        b.aabb(),
		})
	}
	for _, c := range w.contacts {
		cd := ContactDebug{A: c.A.id, B: c.B.id, Normal: c.Normal}
		for i := 0; i < c.Count; i++ {
			p := c.Points[i]
			cd.Points = append(cd.Points, ContactPoint{
				Pos:            p.Pos,
				Penetration:    p.Penetration,
				NormalImpulse:  p.NormalImpulse,
				TangentImpulse: p.TangentImpulse,
			})
		}
		fv.contacts = append(fv.contacts, cd)
	}
	w.view.Store(fv)
}
