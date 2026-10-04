package physics

import (
	"fmt"
	"sort"
)

// SnapshotVersion is the current snapshot format version.
const SnapshotVersion = 1

// BodySnapshot is the serialized form of a single body.
type BodySnapshot struct {
	ID     string    `json:"id"`
	Type   BodyType  `json:"type"`
	Layer  uint32    `json:"layer"`
	PosX   float64   `json:"pos_x"`
	PosY   float64   `json:"pos_y"`
	Rot    float64   `json:"rot"`
	VelX   float64   `json:"vel_x"`
	VelY   float64   `json:"vel_y"`
	AngVel float64   `json:"ang_vel"`
	Shape  ShapeDesc `json:"shape"`
}

// WorldSnapshot is a fully self-contained, process-independent snapshot of
// a world. It contains no pointers or handles and can be freely serialized.
type WorldSnapshot struct {
	Version  int            `json:"version"`
	FixedDt  float64        `json:"fixed_dt"`
	GravityX float64        `json:"gravity_x"`
	GravityY float64        `json:"gravity_y"`
	Paused   bool           `json:"paused"`
	Frame    uint64         `json:"frame"`
	Accum    float64        `json:"accum"`
	Bodies   []BodySnapshot `json:"bodies"`
}

// Snapshot returns a process-independent snapshot of the current world
// state. Bodies are sorted by identifier.
func (w *World) Snapshot() *WorldSnapshot {
	w.mu.RLock()
	defer w.mu.RUnlock()

	snap := &WorldSnapshot{
		Version:  SnapshotVersion,
		FixedDt:  w.fixedDt,
		GravityX: w.gravity.X,
		GravityY: w.gravity.Y,
		Paused:   w.paused,
		Frame:    w.frame.Number,
		Accum:    w.accum,
		Bodies:   make([]BodySnapshot, 0, len(w.frame.Bodies)),
	}
	for i := range w.frame.Bodies {
		b := &w.frame.Bodies[i]
		snap.Bodies = append(snap.Bodies, BodySnapshot{
			ID:     b.ID,
			Type:   b.Type,
			Layer:  b.Layer,
			PosX:   b.Pos.X,
			PosY:   b.Pos.Y,
			Rot:    b.Rot,
			VelX:   b.Vel.X,
			VelY:   b.Vel.Y,
			AngVel: b.AngVel,
			Shape:  shapeDesc(b.Shape),
		})
	}
	return snap
}

// NewWorldFromSnapshot restores a world from a snapshot produced by
// Snapshot (possibly serialized to disk). The snapshot must not depend on
// any in-process state. Invalid snapshots (unknown version, bad timestep,
// empty or duplicate identifiers, degenerate shapes) are rejected.
func NewWorldFromSnapshot(s *WorldSnapshot) (*World, error) {
	if s == nil {
		return nil, &Error{
			Code:    ErrInvalidParam,
			Message: "snapshot must not be nil",
		}
	}
	if s.Version != SnapshotVersion {
		return nil, &Error{
			Code:    ErrInvalidParam,
			Param:   "version",
			Value:   fmt.Sprintf("%d", s.Version),
			Message: "unsupported snapshot version",
		}
	}
	if s.FixedDt <= 0 {
		return nil, &Error{
			Code:    ErrInvalidParam,
			Param:   "fixed_dt",
			Value:   fmt.Sprintf("%g", s.FixedDt),
			Message: "fixed timestep must be strictly positive",
		}
	}

	bodies := make([]Body, 0, len(s.Bodies))
	seen := make(map[string]bool, len(s.Bodies))
	for _, bs := range s.Bodies {
		if bs.ID == "" {
			return nil, &Error{
				Code:    ErrEmptyID,
				Message: "snapshot contains a body with an empty identifier",
			}
		}
		if seen[bs.ID] {
			return nil, &Error{
				Code:    ErrDuplicateID,
				Object:  bs.ID,
				Message: "snapshot contains a duplicate body identifier",
			}
		}
		seen[bs.ID] = true

		if bs.Type != BodyStatic && bs.Type != BodyDynamic {
			return nil, &Error{
				Code:    ErrInvalidParam,
				Object:  bs.ID,
				Param:   "type",
				Value:   fmt.Sprintf("%d", bs.Type),
				Message: "unknown body type",
			}
		}
		sh, err := shapeFromDesc(bs.Shape)
		if err != nil {
			if e, ok := err.(*Error); ok {
				e.Object = bs.ID
			}
			return nil, err
		}
		bodies = append(bodies, Body{
			ID:     bs.ID,
			Type:   bs.Type,
			Layer:  bs.Layer,
			Pos:    Vec2{bs.PosX, bs.PosY},
			Rot:    bs.Rot,
			Vel:    Vec2{bs.VelX, bs.VelY},
			AngVel: bs.AngVel,
			Shape:  sh,
		})
	}
	sort.Slice(bodies, func(i, j int) bool { return bodies[i].ID < bodies[j].ID })

	w := &World{
		fixedDt: s.FixedDt,
		gravity: Vec2{s.GravityX, s.GravityY},
		paused:  s.Paused,
		accum:   s.Accum,
		frame:   &Frame{Number: s.Frame},
	}
	w.frame.Bodies = bodies
	w.frame.Contacts = detectContacts(bodies)
	return w, nil
}

// shapeFromDesc reconstructs a validated shape from its serialized form.
func shapeFromDesc(d ShapeDesc) (Shape, error) {
	switch d.Kind {
	case ShapeCircle:
		c := Circle{Radius: d.Radius}
		if err := validateShape(c); err != nil {
			return nil, err
		}
		return c, nil
	case ShapeBox:
		b := Box{HalfW: d.HalfW, HalfH: d.HalfH}
		if err := validateShape(b); err != nil {
			return nil, err
		}
		return b, nil
	}
	return nil, &Error{
		Code:    ErrInvalidShape,
		Param:   "kind",
		Value:   fmt.Sprintf("%d", d.Kind),
		Message: "unknown shape kind",
	}
}
