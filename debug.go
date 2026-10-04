package physics

import (
	"encoding/json"
	"sort"
)

// DebugBody is the per-body data of a debug frame.
type DebugBody struct {
	ID     string    `json:"id"`
	Type   BodyType  `json:"type"`
	Layer  uint32    `json:"layer"`
	Pos    Vec2      `json:"pos"`
	Rot    float64   `json:"rot"`
	Vel    Vec2      `json:"vel"`
	AngVel float64   `json:"ang_vel"`
	Shape  ShapeDesc `json:"shape"`
	AABB   AABB      `json:"aabb"`
}

// ShapeDesc describes a body's shape in the debug data.
type ShapeDesc struct {
	Kind   ShapeKind `json:"kind"`
	Radius float64   `json:"radius,omitempty"`
	HalfW  float64   `json:"half_w,omitempty"`
	HalfH  float64   `json:"half_h,omitempty"`
}

// DebugContact is a contact manifold in a debug frame.
type DebugContact struct {
	BodyA  string         `json:"body_a"`
	BodyB  string         `json:"body_b"`
	Normal Vec2           `json:"normal"`
	Points []ContactPoint `json:"points"`
}

// DebugRayQuery is a recorded ray query plus its hits in a debug frame.
type DebugRayQuery struct {
	Origin   Vec2     `json:"origin"`
	Dir      Vec2     `json:"dir"`
	MaxDist  float64  `json:"max_dist"`
	Hits     []RayHit `json:"hits"`
	Filtered int      `json:"filtered"`
}

// DebugAreaQuery is a recorded area query plus its hits in a debug frame.
type DebugAreaQuery struct {
	Kind     ShapeKind `json:"kind"`
	Center   Vec2      `json:"center"`
	Radius   float64   `json:"radius,omitempty"`
	HalfW    float64   `json:"half_w,omitempty"`
	HalfH    float64   `json:"half_h,omitempty"`
	Hits     []AreaHit `json:"hits"`
	Filtered int       `json:"filtered"`
}

// DebugFrame is a fully self-contained, deterministically ordered snapshot
// of a frame for visualization. Bodies and contacts are sorted by
// identifier; query records are sorted by their content so serialization is
// stable and reproducible regardless of goroutine scheduling.
type DebugFrame struct {
	Version     int              `json:"version"`
	Frame       uint64           `json:"frame"`
	FixedDt     float64          `json:"fixed_dt"`
	Gravity     Vec2             `json:"gravity"`
	Bodies      []DebugBody      `json:"bodies"`
	Contacts    []DebugContact   `json:"contacts"`
	RayQueries  []DebugRayQuery  `json:"ray_queries"`
	AreaQueries []DebugAreaQuery `json:"area_queries"`
}

// DebugFrameVersion is the current debug data format version.
const DebugFrameVersion = 1

// DebugFrame builds the debug data for the current frame.
func (w *World) DebugFrame() *DebugFrame {
	w.mu.RLock()
	f := w.frame
	fixedDt := w.fixedDt
	gravity := w.gravity
	w.mu.RUnlock()

	f.logMu.Lock()
	rayLogs := append([]RayQueryRecord(nil), f.rayLogs...)
	areaLogs := append([]AreaQueryRecord(nil), f.areaLogs...)
	f.logMu.Unlock()

	dbg := &DebugFrame{
		Version: DebugFrameVersion,
		Frame:   f.Number,
		FixedDt: fixedDt,
		Gravity: gravity,
		Bodies:  make([]DebugBody, 0, len(f.Bodies)),
	}
	for i := range f.Bodies {
		b := &f.Bodies[i]
		dbg.Bodies = append(dbg.Bodies, DebugBody{
			ID:     b.ID,
			Type:   b.Type,
			Layer:  b.Layer,
			Pos:    b.Pos,
			Rot:    b.Rot,
			Vel:    b.Vel,
			AngVel: b.AngVel,
			Shape:  shapeDesc(b.Shape),
			AABB:   b.AABB(),
		})
	}
	for i := range f.Contacts {
		c := &f.Contacts[i]
		points := make([]ContactPoint, len(c.Points))
		copy(points, c.Points)
		dbg.Contacts = append(dbg.Contacts, DebugContact{
			BodyA:  c.BodyA,
			BodyB:  c.BodyB,
			Normal: c.Normal,
			Points: points,
		})
	}

	for i := range rayLogs {
		r := &rayLogs[i]
		hits := make([]RayHit, len(r.Hits))
		copy(hits, r.Hits)
		dbg.RayQueries = append(dbg.RayQueries, DebugRayQuery{
			Origin:   r.Origin,
			Dir:      r.Dir,
			MaxDist:  r.MaxDist,
			Hits:     hits,
			Filtered: r.Filtered,
		})
	}
	for i := range areaLogs {
		r := &areaLogs[i]
		hits := make([]AreaHit, len(r.Hits))
		copy(hits, r.Hits)
		dbg.AreaQueries = append(dbg.AreaQueries, DebugAreaQuery{
			Kind:     r.Kind,
			Center:   r.Center,
			Radius:   r.Radius,
			HalfW:    r.HalfW,
			HalfH:    r.HalfH,
			Hits:     hits,
			Filtered: r.Filtered,
		})
	}

	sort.Slice(dbg.RayQueries, func(i, j int) bool {
		return rayQueryLess(dbg.RayQueries[i], dbg.RayQueries[j])
	})
	sort.Slice(dbg.AreaQueries, func(i, j int) bool {
		return areaQueryLess(dbg.AreaQueries[i], dbg.AreaQueries[j])
	})
	return dbg
}

// DebugJSON serializes the current frame's debug data as stable JSON.
func (w *World) DebugJSON() ([]byte, error) {
	return json.Marshal(w.DebugFrame())
}

func shapeDesc(s Shape) ShapeDesc {
	switch sh := s.(type) {
	case Circle:
		return ShapeDesc{Kind: ShapeCircle, Radius: sh.Radius}
	case Box:
		return ShapeDesc{Kind: ShapeBox, HalfW: sh.HalfW, HalfH: sh.HalfH}
	}
	return ShapeDesc{}
}

func rayQueryLess(a, b DebugRayQuery) bool {
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
	if a.MaxDist != b.MaxDist {
		return a.MaxDist < b.MaxDist
	}
	if len(a.Hits) != len(b.Hits) {
		return len(a.Hits) < len(b.Hits)
	}
	for i := range a.Hits {
		hx, hy := a.Hits[i], b.Hits[i]
		if hx.Dist != hy.Dist {
			return hx.Dist < hy.Dist
		}
		if hx.BodyID != hy.BodyID {
			return hx.BodyID < hy.BodyID
		}
	}
	return a.Filtered < b.Filtered
}

func areaQueryLess(a, b DebugAreaQuery) bool {
	if a.Kind != b.Kind {
		return a.Kind < b.Kind
	}
	if a.Center.X != b.Center.X {
		return a.Center.X < b.Center.X
	}
	if a.Center.Y != b.Center.Y {
		return a.Center.Y < b.Center.Y
	}
	if a.Radius != b.Radius {
		return a.Radius < b.Radius
	}
	if a.HalfW != b.HalfW {
		return a.HalfW < b.HalfW
	}
	if a.HalfH != b.HalfH {
		return a.HalfH < b.HalfH
	}
	if len(a.Hits) != len(b.Hits) {
		return len(a.Hits) < len(b.Hits)
	}
	for i := range a.Hits {
		if a.Hits[i].BodyID != b.Hits[i].BodyID {
			return a.Hits[i].BodyID < b.Hits[i].BodyID
		}
	}
	return a.Filtered < b.Filtered
}
