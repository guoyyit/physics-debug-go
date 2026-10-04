package physics

import (
	"fmt"
	"math"
	"sort"
)

// RayFilter selects which bodies a query may hit.
type RayFilter struct {
	// Mask is a bitmask ANDed with the body's layer. A value of zero means
	// "no layer filtering" (every layer passes).
	Mask uint32
	// StaticOnly restricts the query to static bodies when true.
	StaticOnly bool
	// DynamicOnly restricts the query to dynamic bodies when true.
	// StaticOnly and DynamicOnly must not both be set.
	DynamicOnly bool
	// Accept, when non-nil, is an additional predicate; bodies for which it
	// returns false are counted as filtered.
	Accept func(b *Body) bool
}

// accept reports whether the body passes the filter.
func (f RayFilter) accept(b *Body) bool {
	if f.StaticOnly && b.Type != BodyStatic {
		return false
	}
	if f.DynamicOnly && b.Type != BodyDynamic {
		return false
	}
	if f.Mask != 0 && b.Layer&f.Mask == 0 {
		return false
	}
	if f.Accept != nil && !f.Accept(b) {
		return false
	}
	return true
}

func (f RayFilter) validate() error {
	if f.StaticOnly && f.DynamicOnly {
		return &Error{
			Code:    ErrInvalidParam,
			Param:   "filter",
			Message: "StaticOnly and DynamicOnly are mutually exclusive",
		}
	}
	return nil
}

// RayHit describes a single ray intersection.
type RayHit struct {
	BodyID string  `json:"body_id"`
	Layer  uint32  `json:"layer"`
	Dist   float64 `json:"dist"`
	Point  Vec2    `json:"point"`
	Normal Vec2    `json:"normal"`
}

// RaycastResult is the outcome of a raycast against one frame.
type RaycastResult struct {
	// Frame is the frame number the ray was evaluated against.
	Frame uint64 `json:"frame"`
	// Hits sorted by (distance, body identifier).
	Hits []RayHit `json:"hits"`
	// Filtered counts shapes that intersected the ray but were rejected by
	// the filter.
	Filtered int `json:"filtered"`
}

// RayQueryRecord stores a ray query plus its outcome for debug data.
type RayQueryRecord struct {
	Origin   Vec2
	Dir      Vec2
	MaxDist  float64
	Hits     []RayHit
	Filtered int
}

// Raycast casts a ray against the current frame. A zero direction or a
// non-positive maxDist is an error; a ray that hits nothing returns an empty
// result with Filtered == 0, while hits rejected by the filter are counted
// in Filtered.
func (w *World) Raycast(origin, dir Vec2, maxDist float64, f RayFilter) (*RaycastResult, error) {
	if dir.Len2() == 0 {
		return nil, &Error{
			Code:    ErrInvalidParam,
			Param:   "dir",
			Value:   fmt.Sprintf("(%g,%g)", dir.X, dir.Y),
			Message: "ray direction must not be zero",
		}
	}
	if maxDist <= 0 {
		return nil, &Error{
			Code:    ErrInvalidParam,
			Param:   "max_dist",
			Value:   fmt.Sprintf("%g", maxDist),
			Message: "ray max distance must be positive",
		}
	}
	if err := f.validate(); err != nil {
		return nil, err
	}

	d := dir.Normalized()
	frame := w.currentFrame()
	res := &RaycastResult{Frame: frame.Number}

	for i := range frame.Bodies {
		b := &frame.Bodies[i]
		dist, n, ok := rayShape(origin, d, maxDist, b)
		if !ok {
			continue
		}
		if !f.accept(b) {
			res.Filtered++
			continue
		}
		res.Hits = append(res.Hits, RayHit{
			BodyID: b.ID,
			Layer:  b.Layer,
			Dist:   dist,
			Point:  origin.Add(d.Mul(dist)),
			Normal: n,
		})
	}
	sort.Slice(res.Hits, func(i, j int) bool {
		if res.Hits[i].Dist != res.Hits[j].Dist {
			return res.Hits[i].Dist < res.Hits[j].Dist
		}
		return res.Hits[i].BodyID < res.Hits[j].BodyID
	})

	frame.appendRayLog(RayQueryRecord{
		Origin:   origin,
		Dir:      d,
		MaxDist:  maxDist,
		Hits:     res.Hits,
		Filtered: res.Filtered,
	})
	return res, nil
}

// rayShape intersects the ray (origin + t*dir, t in [0, maxT]) with the
// body's shape, returning the nearest positive t and the surface normal.
func rayShape(origin, dir Vec2, maxT float64, b *Body) (float64, Vec2, bool) {
	switch s := b.Shape.(type) {
	case Circle:
		return rayCircle(origin, dir, maxT, s, b.Pos)
	case Box:
		return rayBox(origin, dir, maxT, s, b.Pos)
	}
	return 0, Vec2{}, false
}

func rayCircle(origin, dir Vec2, maxT float64, c Circle, center Vec2) (float64, Vec2, bool) {
	f := origin.Sub(center)
	proj := f.Dot(dir)
	disc := proj*proj - (f.Len2() - c.Radius*c.Radius)
	if disc < 0 {
		return 0, Vec2{}, false
	}
	sqrtDisc := math.Sqrt(disc)
	t := -proj - sqrtDisc
	if t < 0 {
		t = -proj + sqrtDisc
	}
	if t < 0 || t > maxT {
		return 0, Vec2{}, false
	}
	hit := origin.Add(dir.Mul(t))
	n := hit.Sub(center).Mul(1 / c.Radius)
	return t, n, true
}

func rayBox(origin, dir Vec2, maxT float64, bx Box, center Vec2) (float64, Vec2, bool) {
	bmin := center.Sub(Vec2{bx.HalfW, bx.HalfH})
	bmax := center.Add(Vec2{bx.HalfW, bx.HalfH})

	tmin := 0.0
	tmax := maxT
	var nmin, nmax Vec2

	// updateAxis clips the [tmin, tmax] interval against one slab and
	// tracks the entry/exit normals. It returns false when the ray is
	// parallel to and outside the slab.
	updateAxis := func(o, d, lo, hi float64, nLo, nHi Vec2) bool {
		if math.Abs(d) < 1e-15 {
			return o >= lo && o <= hi
		}
		t1 := (lo - o) / d
		t2 := (hi - o) / d
		n1, n2 := nLo, nHi
		if t1 > t2 {
			t1, t2 = t2, t1
			n1, n2 = nHi, nLo
		}
		if t1 > tmin {
			tmin = t1
			nmin = n1
		}
		if t2 < tmax {
			tmax = t2
			nmax = n2
		}
		return tmin <= tmax
	}

	if !updateAxis(origin.X, dir.X, bmin.X, bmax.X, Vec2{-1, 0}, Vec2{1, 0}) {
		return 0, Vec2{}, false
	}
	if !updateAxis(origin.Y, dir.Y, bmin.Y, bmax.Y, Vec2{0, -1}, Vec2{0, 1}) {
		return 0, Vec2{}, false
	}
	if tmin > maxT || tmax < 0 {
		return 0, Vec2{}, false
	}
	if tmin >= 0 {
		return tmin, nmin, true
	}
	// Origin is inside the box: use the exit point.
	return tmax, nmax, true
}

// AreaHit describes a single body overlapping an area query.
type AreaHit struct {
	BodyID string `json:"body_id"`
	Layer  uint32 `json:"layer"`
}

// AreaResult is the outcome of an area query against one frame.
type AreaResult struct {
	Frame    uint64    `json:"frame"`
	Hits     []AreaHit `json:"hits"`
	Filtered int       `json:"filtered"`
}

// AreaQueryRecord stores an area query plus its outcome for debug data.
type AreaQueryRecord struct {
	Kind     ShapeKind
	Center   Vec2
	Radius   float64 // for circle queries
	HalfW    float64 // for box queries
	HalfH    float64
	Hits     []AreaHit
	Filtered int
}

// OverlapCircle queries bodies whose shape overlaps the given circle. A
// non-positive radius is an error.
func (w *World) OverlapCircle(center Vec2, radius float64, f RayFilter) (*AreaResult, error) {
	if radius <= 0 {
		return nil, &Error{
			Code:    ErrInvalidParam,
			Param:   "radius",
			Value:   fmt.Sprintf("%g", radius),
			Message: "overlap circle radius must be positive",
		}
	}
	if err := f.validate(); err != nil {
		return nil, err
	}

	frame := w.currentFrame()
	res := &AreaResult{Frame: frame.Number}
	for i := range frame.Bodies {
		b := &frame.Bodies[i]
		if !shapeOverlapsCircle(b, center, radius) {
			continue
		}
		if !f.accept(b) {
			res.Filtered++
			continue
		}
		res.Hits = append(res.Hits, AreaHit{BodyID: b.ID, Layer: b.Layer})
	}
	sort.Slice(res.Hits, func(i, j int) bool { return res.Hits[i].BodyID < res.Hits[j].BodyID })

	frame.appendAreaLog(AreaQueryRecord{
		Kind:     ShapeCircle,
		Center:   center,
		Radius:   radius,
		Hits:     res.Hits,
		Filtered: res.Filtered,
	})
	return res, nil
}

// OverlapBox queries bodies whose shape overlaps the given axis-aligned box.
// Non-positive half extents are an error.
func (w *World) OverlapBox(center Vec2, halfW, halfH float64, f RayFilter) (*AreaResult, error) {
	if halfW <= 0 {
		return nil, &Error{
			Code:    ErrInvalidParam,
			Param:   "half_w",
			Value:   fmt.Sprintf("%g", halfW),
			Message: "overlap box half width must be positive",
		}
	}
	if halfH <= 0 {
		return nil, &Error{
			Code:    ErrInvalidParam,
			Param:   "half_h",
			Value:   fmt.Sprintf("%g", halfH),
			Message: "overlap box half height must be positive",
		}
	}
	if err := f.validate(); err != nil {
		return nil, err
	}

	frame := w.currentFrame()
	res := &AreaResult{Frame: frame.Number}
	for i := range frame.Bodies {
		b := &frame.Bodies[i]
		if !shapeOverlapsBox(b, center, halfW, halfH) {
			continue
		}
		if !f.accept(b) {
			res.Filtered++
			continue
		}
		res.Hits = append(res.Hits, AreaHit{BodyID: b.ID, Layer: b.Layer})
	}
	sort.Slice(res.Hits, func(i, j int) bool { return res.Hits[i].BodyID < res.Hits[j].BodyID })

	frame.appendAreaLog(AreaQueryRecord{
		Kind:     ShapeBox,
		Center:   center,
		HalfW:    halfW,
		HalfH:    halfH,
		Hits:     res.Hits,
		Filtered: res.Filtered,
	})
	return res, nil
}

// shapeOverlapsCircle reports whether body b's shape overlaps a circle.
func shapeOverlapsCircle(b *Body, center Vec2, radius float64) bool {
	switch s := b.Shape.(type) {
	case Circle:
		rsum := s.Radius + radius
		return b.Pos.Sub(center).Len2() < rsum*rsum
	case Box:
		closest := Vec2{
			X: clamp(center.X, b.Pos.X-s.HalfW, b.Pos.X+s.HalfW),
			Y: clamp(center.Y, b.Pos.Y-s.HalfH, b.Pos.Y+s.HalfH),
		}
		return center.Sub(closest).Len2() < radius*radius
	}
	return false
}

// shapeOverlapsBox reports whether body b's shape overlaps an axis-aligned
// box.
func shapeOverlapsBox(b *Body, center Vec2, halfW, halfH float64) bool {
	switch s := b.Shape.(type) {
	case Circle:
		closest := Vec2{
			X: clamp(b.Pos.X, center.X-halfW, center.X+halfW),
			Y: clamp(b.Pos.Y, center.Y-halfH, center.Y+halfH),
		}
		return b.Pos.Sub(closest).Len2() < s.Radius*s.Radius
	case Box:
		aMin := b.Pos.Sub(Vec2{s.HalfW, s.HalfH})
		aMax := b.Pos.Add(Vec2{s.HalfW, s.HalfH})
		qMin := center.Sub(Vec2{halfW, halfH})
		qMax := center.Add(Vec2{halfW, halfH})
		return aMin.X < qMax.X && aMax.X > qMin.X &&
			aMin.Y < qMax.Y && aMax.Y > qMin.Y
	}
	return false
}

// appendRayLog appends a ray query record to the frame's log.
func (f *Frame) appendRayLog(rec RayQueryRecord) {
	f.logMu.Lock()
	f.rayLogs = append(f.rayLogs, rec)
	f.logMu.Unlock()
}

// appendAreaLog appends an area query record to the frame's log.
func (f *Frame) appendAreaLog(rec AreaQueryRecord) {
	f.logMu.Lock()
	f.areaLogs = append(f.areaLogs, rec)
	f.logMu.Unlock()
}
