package physics

import (
	"encoding/json"
	"math"
	"sort"
)

// ShapeState is the serializable form of a shape. Only the fields
// relevant to Kind are meaningful.
type ShapeState struct {
	Kind   ShapeKind
	Radius float64 // circle
	HalfW  float64 // box
	HalfH  float64 // box
	Offset Vec2
	Angle  float64 // box local rotation
}

func shapeToState(s Shape) ShapeState {
	switch v := s.(type) {
	case Circle:
		return ShapeState{Kind: ShapeCircle, Radius: v.Radius, Offset: v.Offset}
	case Box:
		return ShapeState{Kind: ShapeBox, HalfW: v.HalfW, HalfH: v.HalfH, Offset: v.Offset, Angle: v.Angle}
	}
	return ShapeState{}
}

func (ss ShapeState) toShape() Shape {
	switch ss.Kind {
	case ShapeCircle:
		return Circle{Radius: ss.Radius, Offset: ss.Offset}
	case ShapeBox:
		return Box{HalfW: ss.HalfW, HalfH: ss.HalfH, Offset: ss.Offset, Angle: ss.Angle}
	}
	return nil
}

// BodyState is the full serializable state of one body.
type BodyState struct {
	ID          BodyID
	Kind        BodyKind
	Pos         Vec2
	Angle       float64
	Vel         Vec2
	AngVel      float64
	Layer       uint32
	Mask        uint32
	Density     float64
	Friction    float64
	Restitution float64
	Shapes      []ShapeState
}

// Snapshot is a complete, self-contained world state. It contains only
// plain data (no pointers into the live world), so it can be serialized
// and restored in any process.
type Snapshot struct {
	Frame  uint64
	Time   float64
	Paused bool
	Bodies []BodyState // sorted by body ID
}

// SaveSnapshot captures the current world state.
func (w *World) SaveSnapshot() *Snapshot {
	w.mu.Lock()
	defer w.mu.Unlock()
	s := &Snapshot{
		Frame:  w.frame,
		Time:   w.time,
		Paused: w.paused.Load(),
	}
	for _, id := range w.order {
		b := w.bodies[id]
		bs := BodyState{
			ID:          b.id,
			Kind:        b.kind,
			Pos:         b.pos,
			Angle:       b.angle,
			Vel:         b.vel,
			AngVel:      b.angVel,
			Layer:       b.layer,
			Mask:        b.mask,
			Friction:    b.friction,
			Restitution: b.restitution,
		}
		for _, sh := range b.shapes {
			bs.Shapes = append(bs.Shapes, shapeToState(sh))
		}
		s.Bodies = append(s.Bodies, bs)
	}
	return s
}

// validate checks the snapshot for structural and physical problems.
// It returns the first error found, naming the offending body/field.
func (s *Snapshot) validate() error {
	const op = "Snapshot.validate"
	if s == nil {
		return newError(ErrBadSnapshot, op, "snapshot is nil")
	}
	if math.IsNaN(s.Time) || math.IsInf(s.Time, 0) || s.Time < 0 {
		return newError(ErrBadSnapshot, op, "time must be a finite number >= 0, got %v", s.Time)
	}
	seen := make(map[BodyID]bool, len(s.Bodies))
	for i := range s.Bodies {
		bs := &s.Bodies[i]
		if seen[bs.ID] {
			return newError(ErrBadSnapshot, op, "duplicate body ID %d", bs.ID)
		}
		seen[bs.ID] = true
		def := BodyDef{
			ID:          bs.ID,
			Kind:        bs.Kind,
			Pos:         bs.Pos,
			Angle:       bs.Angle,
			Vel:         bs.Vel,
			AngVel:      bs.AngVel,
			Layer:       bs.Layer,
			Mask:        bs.Mask,
			Density:     bs.Density,
			Friction:    bs.Friction,
			Restitution: bs.Restitution,
		}
		for j := range bs.Shapes {
			sh := bs.Shapes[j].toShape()
			if sh == nil {
				return newError(ErrBadSnapshot, op,
					"body %d shape %d: unknown kind %d", bs.ID, j, bs.Shapes[j].Kind)
			}
			def.Shapes = append(def.Shapes, sh)
		}
		// Reuse body construction for full validation.
		if _, err := newBody(def); err != nil {
			return newError(ErrBadSnapshot, op, "body %d: %s", bs.ID, err)
		}
	}
	return nil
}

// RestoreSnapshot replaces the world state with the snapshot content.
// The snapshot is fully validated first; on error the world is left
// unchanged. Restoring and continuing to step produces exactly the
// same frames as uninterrupted simulation.
func (w *World) RestoreSnapshot(s *Snapshot) error {
	const op = "World.RestoreSnapshot"
	if err := s.validate(); err != nil {
		return newError(ErrBadSnapshot, op, "%s", err)
	}

	// Build everything before touching the world.
	bodies := make(map[BodyID]*Body, len(s.Bodies))
	order := make([]BodyID, 0, len(s.Bodies))
	for i := range s.Bodies {
		bs := &s.Bodies[i]
		def := BodyDef{
			ID:          bs.ID,
			Kind:        bs.Kind,
			Pos:         bs.Pos,
			Angle:       bs.Angle,
			Vel:         bs.Vel,
			AngVel:      bs.AngVel,
			Layer:       bs.Layer,
			Mask:        bs.Mask,
			Density:     bs.Density,
			Friction:    bs.Friction,
			Restitution: bs.Restitution,
		}
		for j := range bs.Shapes {
			def.Shapes = append(def.Shapes, bs.Shapes[j].toShape())
		}
		b, err := newBody(def)
		if err != nil {
			return newError(ErrBadSnapshot, op, "body %d: %s", bs.ID, err)
		}
		bodies[b.id] = b
		order = append(order, b.id)
	}
	sort.Slice(order, func(i, j int) bool { return order[i] < order[j] })

	w.mu.Lock()
	defer w.mu.Unlock()
	w.bodies = bodies
	w.order = order
	w.frame = s.Frame
	w.time = s.Time
	w.paused.Store(s.Paused)
	w.contacts = nil
	w.publishLocked()
	return nil
}

// Encode serializes the snapshot to stable JSON: struct field order is
// fixed and Bodies is sorted by ID.
func (s *Snapshot) Encode() ([]byte, error) {
	return json.Marshal(s)
}

// DecodeSnapshot parses and validates bytes produced by Encode.
func DecodeSnapshot(data []byte) (*Snapshot, error) {
	var s Snapshot
	if err := json.Unmarshal(data, &s); err != nil {
		return nil, newError(ErrBadSnapshot, "DecodeSnapshot", "invalid JSON: %v", err)
	}
	if err := s.validate(); err != nil {
		return nil, err
	}
	return &s, nil
}
