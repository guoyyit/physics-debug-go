package physics

import (
	"math"
	"sort"
)

// ContactPoint is a single point of a contact manifold in world space.
type ContactPoint struct {
	Pos   Vec2    `json:"pos"`
	Depth float64 `json:"depth"`
}

// Contact is a contact manifold between two bodies. Normal points from the
// body with the smaller identifier to the body with the larger identifier.
type Contact struct {
	BodyA  string         `json:"body_a"`
	BodyB  string         `json:"body_b"`
	Normal Vec2           `json:"normal"`
	Points []ContactPoint `json:"points"`
}

// detectContacts computes all contacts for a body slice that is already
// sorted by identifier. The returned slice is sorted by (BodyA, BodyB).
func detectContacts(bodies []Body) []Contact {
	var out []Contact
	for i := 0; i < len(bodies); i++ {
		for j := i + 1; j < len(bodies); j++ {
			if c, ok := collide(bodies[i], bodies[j]); ok {
				out = append(out, c)
			}
		}
	}
	sort.Slice(out, func(i, j int) bool {
		if out[i].BodyA != out[j].BodyA {
			return out[i].BodyA < out[j].BodyA
		}
		return out[i].BodyB < out[j].BodyB
	})
	return out
}

// collide returns the contact manifold between a and b. Caller must ensure
// a.ID < b.ID; the normal points from a to b.
func collide(a, b Body) (Contact, bool) {
	switch sa := a.Shape.(type) {
	case Circle:
		switch sb := b.Shape.(type) {
		case Circle:
			return circleCircle(a, sa, b, sb)
		case Box:
			return circleBox(a, sa, b, sb)
		}
	case Box:
		switch sb := b.Shape.(type) {
		case Circle:
			// Compute circle-vs-box with the circle as body A, then flip.
			c, ok := circleBox(b, sb, a, sa)
			if !ok {
				return Contact{}, false
			}
			c.BodyA, c.BodyB = a.ID, b.ID
			c.Normal = c.Normal.Neg()
			return c, true
		case Box:
			return boxBox(a, sa, b, sb)
		}
	}
	return Contact{}, false
}

func circleCircle(a Body, ca Circle, b Body, cb Circle) (Contact, bool) {
	delta := b.Pos.Sub(a.Pos)
	d := delta.Len()
	rsum := ca.Radius + cb.Radius
	if d >= rsum {
		return Contact{}, false
	}
	n := Vec2{1, 0}
	if d > 0 {
		n = delta.Mul(1 / d)
	}
	depth := rsum - d
	point := a.Pos.Add(n.Mul(ca.Radius))
	return Contact{
		BodyA:  a.ID,
		BodyB:  b.ID,
		Normal: n,
		Points: []ContactPoint{{Pos: point, Depth: depth}},
	}, true
}

// circleBox computes a contact between circle a and axis-aligned box b.
func circleBox(a Body, ca Circle, b Body, bb Box) (Contact, bool) {
	boxMin := b.Pos.Sub(Vec2{bb.HalfW, bb.HalfH})
	boxMax := b.Pos.Add(Vec2{bb.HalfW, bb.HalfH})

	// Closest point on the box to the circle center.
	closest := Vec2{
		X: clamp(a.Pos.X, boxMin.X, boxMax.X),
		Y: clamp(a.Pos.Y, boxMin.Y, boxMax.Y),
	}
	delta := a.Pos.Sub(closest)
	d2 := delta.Len2()
	r := ca.Radius

	if d2 > 0 {
		if d2 >= r*r {
			return Contact{}, false
		}
		d := math.Sqrt(d2)
		// delta points from the box surface toward the circle center, so
		// the normal from circle (A) to box (B) is its negation.
		n := delta.Mul(-1 / d)
		depth := r - d
		return Contact{
			BodyA:  a.ID,
			BodyB:  b.ID,
			Normal: n,
			Points: []ContactPoint{{Pos: closest, Depth: depth}},
		}, true
	}

	// Circle center is inside the box: push out toward the nearest face.
	type face struct {
		dist float64
		n    Vec2
	}
	faces := []face{
		{a.Pos.X - boxMin.X, Vec2{-1, 0}}, // left
		{boxMax.X - a.Pos.X, Vec2{1, 0}},  // right
		{a.Pos.Y - boxMin.Y, Vec2{0, -1}}, // bottom
		{boxMax.Y - a.Pos.Y, Vec2{0, 1}},  // top
	}
	best := faces[0]
	for _, f := range faces[1:] {
		if f.dist < best.dist {
			best = f
		}
	}
	n := best.n
	depth := r + best.dist
	point := a.Pos.Add(n.Mul(best.dist))
	return Contact{
		BodyA:  a.ID,
		BodyB:  b.ID,
		Normal: n,
		Points: []ContactPoint{{Pos: point, Depth: depth}},
	}, true
}

// boxBox computes a contact between two axis-aligned boxes.
func boxBox(a Body, ba Box, b Body, bb Box) (Contact, bool) {
	aMin := a.Pos.Sub(Vec2{ba.HalfW, ba.HalfH})
	aMax := a.Pos.Add(Vec2{ba.HalfW, ba.HalfH})
	bMin := b.Pos.Sub(Vec2{bb.HalfW, bb.HalfH})
	bMax := b.Pos.Add(Vec2{bb.HalfW, bb.HalfH})

	ox := minf(aMax.X, bMax.X) - maxf(aMin.X, bMin.X)
	oy := minf(aMax.Y, bMax.Y) - maxf(aMin.Y, bMin.Y)
	if ox <= 0 || oy <= 0 {
		return Contact{}, false
	}

	var n Vec2
	var depth float64
	if ox < oy {
		depth = ox
		if b.Pos.X >= a.Pos.X {
			n = Vec2{1, 0}
		} else {
			n = Vec2{-1, 0}
		}
	} else {
		depth = oy
		if b.Pos.Y >= a.Pos.Y {
			n = Vec2{0, 1}
		} else {
			n = Vec2{0, -1}
		}
	}

	// Contact point: center of the overlap region.
	point := Vec2{
		(maxf(aMin.X, bMin.X) + minf(aMax.X, bMax.X)) * 0.5,
		(maxf(aMin.Y, bMin.Y) + minf(aMax.Y, bMax.Y)) * 0.5,
	}
	return Contact{
		BodyA:  a.ID,
		BodyB:  b.ID,
		Normal: n,
		Points: []ContactPoint{{Pos: point, Depth: depth}},
	}, true
}

func clamp(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

func minf(a, b float64) float64 {
	if a < b {
		return a
	}
	return b
}

func maxf(a, b float64) float64 {
	if a > b {
		return a
	}
	return b
}


