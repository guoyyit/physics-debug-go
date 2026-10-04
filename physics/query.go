package physics

import (
	"math"
	"sort"
)

// QueryStatus distinguishes "nothing there" from "there but filtered".
type QueryStatus int

const (
	// QueryHit: at least one result passed the filter.
	QueryHit QueryStatus = iota
	// QueryNoHit: geometry was not intersected at all.
	QueryNoHit
	// QueryFiltered: geometry was intersected but every candidate was
	// excluded by the filter.
	QueryFiltered
)

func (s QueryStatus) String() string {
	switch s {
	case QueryHit:
		return "hit"
	case QueryNoHit:
		return "no-hit"
	case QueryFiltered:
		return "filtered"
	}
	return "unknown"
}

// QueryFilter narrows query results. LayerMask of zero matches every
// layer. Exclude lists body IDs to skip.
type QueryFilter struct {
	LayerMask uint32
	Exclude   []BodyID
}

// match reports whether a body passes the filter.
func (f QueryFilter) match(b bodySnapshot) bool {
	if f.LayerMask != 0 && f.LayerMask&b.layer == 0 {
		return false
	}
	for _, id := range f.Exclude {
		if id == b.id {
			return false
		}
	}
	return true
}

// Hit describes a single ray/shape intersection.
type Hit struct {
	BodyID     BodyID
	ShapeIndex int // index into the body's shape list
	Point      Vec2
	Normal     Vec2
	Dist       float64 // distance along the ray from its origin
}

// RayQuery is a ray-cast request. Dir must be non-zero and is
// normalized internally; MaxDist caps the cast length.
type RayQuery struct {
	Origin  Vec2
	Dir     Vec2
	MaxDist float64
	Filter  QueryFilter
}

// RayResult is the outcome of a Raycast. Hits are sorted by Dist, then
// BodyID, then ShapeIndex, so ordering is stable across runs.
type RayResult struct {
	Status QueryStatus
	Hits   []Hit
}

// Raycast casts a ray against the most recently completed frame.
// Invalid parameters (zero direction, negative/NaN MaxDist, NaN origin)
// produce an *Error with code ErrInvalidParam.
func (w *World) Raycast(q RayQuery) (RayResult, error) {
	const op = "World.Raycast"
	if !q.Origin.IsFinite() {
		return RayResult{}, newError(ErrInvalidParam, op,
			"origin must be finite, got %+v", q.Origin)
	}
	if !q.Dir.IsFinite() || q.Dir.Len2() == 0 {
		return RayResult{}, newError(ErrInvalidParam, op,
			"direction must be finite and non-zero, got %+v", q.Dir)
	}
	if math.IsNaN(q.MaxDist) || math.IsInf(q.MaxDist, 0) || q.MaxDist <= 0 {
		return RayResult{}, newError(ErrInvalidParam, op,
			"MaxDist must be a finite number > 0, got %v", q.MaxDist)
	}

	fv := w.view.Load()
	dir := q.Dir.Norm()
	var all, kept []Hit
	for _, bs := range fv.bodies {
		for si, sh := range bs.shapes {
			h, ok := rayShape(q.Origin, dir, q.MaxDist, sh, bs.pos, bs.angle)
			if !ok {
				continue
			}
			h.BodyID = bs.id
			h.ShapeIndex = si
			all = append(all, h)
			if q.Filter.match(bs) {
				kept = append(kept, h)
			}
		}
	}
	sortHits(kept)

	res := RayResult{}
	switch {
	case len(all) == 0:
		res.Status = QueryNoHit
	case len(kept) == 0:
		res.Status = QueryFiltered
	default:
		res.Status = QueryHit
		res.Hits = kept
	}
	fv.recordRay(RayDebug{Query: q, Status: res.Status, Hits: res.Hits})
	return res, nil
}

// sortHits orders hits by distance, then body ID, then shape index.
func sortHits(h []Hit) {
	sort.Slice(h, func(i, j int) bool {
		if h[i].Dist != h[j].Dist {
			return h[i].Dist < h[j].Dist
		}
		if h[i].BodyID != h[j].BodyID {
			return h[i].BodyID < h[j].BodyID
		}
		return h[i].ShapeIndex < h[j].ShapeIndex
	})
}

// rayShape tests a normalized ray against one shape.
func rayShape(o, d Vec2, maxDist float64, s Shape, pos Vec2, angle float64) (Hit, bool) {
	switch sh := s.(type) {
	case Circle:
		return rayCircle(o, d, maxDist, sh, pos, angle)
	case Box:
		return rayBox(o, d, maxDist, sh, pos, angle)
	}
	return Hit{}, false
}

func rayCircle(o, d Vec2, maxDist float64, c Circle, pos Vec2, angle float64) (Hit, bool) {
	cc := c.worldCenter(pos, angle)
	m := o.Sub(cc)
	b := m.Dot(d)
	disc := b*b - (m.Dot(m) - c.Radius*c.Radius)
	if disc < 0 {
		return Hit{}, false
	}
	t := -b - math.Sqrt(disc)
	if t < 0 {
		t = 0 // origin inside the circle
	}
	if t > maxDist {
		return Hit{}, false
	}
	pt := o.Add(d.Mul(t))
	return Hit{Point: pt, Normal: pt.Sub(cc).Norm(), Dist: t}, true
}

func rayBox(o, d Vec2, maxDist float64, bx Box, pos Vec2, angle float64) (Hit, bool) {
	bc := bx.worldCenter(pos, angle)
	ba := bx.worldAngle(angle)
	lo := o.Sub(bc).Rotate(-ba)
	ld := d.Rotate(-ba)
	half := [2]float64{bx.HalfW, bx.HalfH}
	loA := [2]float64{lo.X, lo.Y}
	ldA := [2]float64{ld.X, ld.Y}

	tmin, tmax := 0.0, maxDist
	axis := -1
	for i := 0; i < 2; i++ {
		if math.Abs(ldA[i]) < 1e-12 {
			if loA[i] < -half[i] || loA[i] > half[i] {
				return Hit{}, false
			}
			continue
		}
		inv := 1 / ldA[i]
		t1 := (-half[i] - loA[i]) * inv
		t2 := (half[i] - loA[i]) * inv
		if t1 > t2 {
			t1, t2 = t2, t1
		}
		if t1 > tmin {
			tmin = t1
			axis = i
		}
		if t2 < tmax {
			tmax = t2
		}
		if tmin > tmax {
			return Hit{}, false
		}
	}
	var nLocal Vec2
	switch axis {
	case 0:
		nLocal = Vec2{math.Copysign(1, -ld.X), 0}
	case 1:
		nLocal = Vec2{0, math.Copysign(1, -ld.Y)}
	default:
		// Origin inside the box with a near-parallel ray: no meaningful
		// entry face; report an exit-less hit at the origin distance.
		nLocal = d.Neg().Norm()
	}
	return Hit{
		Point:  o.Add(d.Mul(tmin)),
		Normal: nLocal.Rotate(ba),
		Dist:   tmin,
	}, true
}

// RegionQuery is an AABB overlap request.
type RegionQuery struct {
	Box    AABB
	Filter QueryFilter
}

// RegionResult is the outcome of OverlapRegion. Bodies is sorted by ID.
type RegionResult struct {
	Status QueryStatus
	Bodies []BodyID
}

// OverlapRegion returns the bodies whose shape AABBs intersect Box.
// The test is at shape-AABB granularity (broadphase-style), not exact
// shape geometry.
func (w *World) OverlapRegion(q RegionQuery) (RegionResult, error) {
	const op = "World.OverlapRegion"
	if !q.Box.IsValid() {
		return RegionResult{}, newError(ErrInvalidParam, op,
			"box must be finite with Min <= Max per axis, got %+v", q.Box)
	}
	fv := w.view.Load()
	var all, kept []BodyID
	for _, bs := range fv.bodies {
		hit := false
		for _, sh := range bs.shapes {
			if sh.AABB(bs.pos, bs.angle).Overlaps(q.Box) {
				hit = true
				break
			}
		}
		if !hit {
			continue
		}
		all = append(all, bs.id)
		if q.Filter.match(bs) {
			kept = append(kept, bs.id)
		}
	}
	// fv.bodies is sorted by ID, so both lists are already ordered.
	res := RegionResult{}
	switch {
	case len(all) == 0:
		res.Status = QueryNoHit
	case len(kept) == 0:
		res.Status = QueryFiltered
	default:
		res.Status = QueryHit
		res.Bodies = kept
	}
	return res, nil
}
