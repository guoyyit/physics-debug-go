package physics

import (
	"encoding/json"
	"sort"
)

// ShapeDebug is the per-shape visualization record.
type ShapeDebug struct {
	Kind   ShapeKind
	Center Vec2    // world-space shape center
	Radius float64 // circles only
	// Corners lists the 4 world-space corners for boxes (nil for circles).
	Corners []Vec2
	AABB    AABB
}

// BodyDebug is the per-body visualization record.
type BodyDebug struct {
	ID     BodyID
	Kind   BodyKind
	Pos    Vec2
	Angle  float64
	AABB   AABB
	Shapes []ShapeDebug
}

// ContactDebug is a contact manifold for visualization.
type ContactDebug struct {
	A, B   BodyID
	Normal Vec2
	Points []ContactPoint
}

// RayDebug records a ray query together with its outcome so the exact
// query that produced a hit can be replayed and drawn.
type RayDebug struct {
	Query  RayQuery
	Status QueryStatus
	Hits   []Hit
}

// DebugFrame is the complete visualization snapshot of one frame. All
// slices are deterministically ordered (bodies by ID, contacts by
// (A, B) then shape pair, rays by query parameters), so serializing
// the same frame always yields the same bytes.
type DebugFrame struct {
	Frame    uint64
	Time     float64
	Bodies   []BodyDebug
	Contacts []ContactDebug
	Rays     []RayDebug
}

// DebugFrame returns the debug data of the most recently completed
// frame, including ray queries issued against that frame.
func (w *World) DebugFrame() DebugFrame {
	fv := w.view.Load()
	d := DebugFrame{
		Frame: fv.frame,
		Time:  fv.time,
	}
	for _, bs := range fv.bodies {
		bd := BodyDebug{
			ID:    bs.id,
			Kind:  bs.kind,
			Pos:   bs.pos,
			Angle: bs.angle,
			AABB:  bs.aabb,
		}
		for _, sh := range bs.shapes {
			sd := ShapeDebug{
				Kind: sh.Kind(),
				AABB: sh.AABB(bs.pos, bs.angle),
			}
			switch v := sh.(type) {
			case Circle:
				sd.Center = v.worldCenter(bs.pos, bs.angle)
				sd.Radius = v.Radius
			case Box:
				sd.Center = v.worldCenter(bs.pos, bs.angle)
				cs := v.corners(bs.pos, bs.angle)
				sd.Corners = []Vec2{cs[0], cs[1], cs[2], cs[3]}
			}
			bd.Shapes = append(bd.Shapes, sd)
		}
		d.Bodies = append(d.Bodies, bd)
	}
	d.Contacts = append(d.Contacts, fv.contacts...)
	d.Rays = fv.rayLog()
	sortRays(d.Rays)
	return d
}

// sortRays orders recorded ray queries by their parameters so the
// serialized form does not depend on the (possibly concurrent) order
// in which queries arrived.
func sortRays(r []RayDebug) {
	sort.Slice(r, func(i, j int) bool {
		a, b := r[i].Query, r[j].Query
		if a.Origin.X != b.Origin.X {
			return a.Origin.X < b.Origin.X
		}
		if a.Origin.Y != b.Origin.Y {
			return a.Origin.Y < b.Origin.Y
		}
		if a.Dir.X != b.Dir.X {
			return a.Dir.X < b.Dir.X
		}
		if a.Dir.Y != b.Dir.Y {
			return a.Dir.Y < b.Dir.Y
		}
		return a.MaxDist < b.MaxDist
	})
}

// Encode serializes the frame to stable JSON: struct field order is
// fixed by encoding/json and all slices are deterministically sorted.
func (d DebugFrame) Encode() ([]byte, error) {
	return json.Marshal(d)
}
