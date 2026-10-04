package physics

import "math"

// maxContactPoints is the manifold point budget per contact pair.
const maxContactPoints = 2

// ContactPoint is one point of a contact manifold.
type ContactPoint struct {
	Pos            Vec2    // world-space contact position
	Penetration    float64 // overlap depth along the normal (>= 0)
	NormalImpulse  float64 // accumulated normal impulse from the solver
	TangentImpulse float64 // accumulated friction impulse from the solver

	// Solver scratch data, recomputed every step.
	rA, rB       Vec2
	massN, massT float64
	bias         float64
}

// Contact is a collision manifold between two bodies. Normal points
// from body A to body B; A always has the smaller BodyID.
type Contact struct {
	A, B        *Body
	Normal      Vec2
	Points      [maxContactPoints]ContactPoint
	Count       int
	Friction    float64
	Restitution float64
}

// collide builds contact manifolds for two bodies, one per colliding
// shape pair, in deterministic (shape index) order.
func collide(a, b *Body) []*Contact {
	var out []*Contact
	for _, sa := range a.shapes {
		for _, sb := range b.shapes {
			c := collideShapes(sa, a.pos, a.angle, sb, b.pos, b.angle)
			if c == nil {
				continue
			}
			c.A, c.B = a, b
			c.Friction = math.Sqrt(a.friction * b.friction)
			c.Restitution = math.Max(a.restitution, b.restitution)
			out = append(out, c)
		}
	}
	return out
}

// collideShapes dispatches on shape kinds. The returned normal always
// points from shape a to shape b.
func collideShapes(sa Shape, pa Vec2, aa float64, sb Shape, pb Vec2, ab float64) *Contact {
	switch ta := sa.(type) {
	case Circle:
		switch tb := sb.(type) {
		case Circle:
			return circleCircle(ta, pa, aa, tb, pb, ab)
		case Box:
			return circleBox(ta, pa, aa, tb, pb, ab)
		}
	case Box:
		switch tb := sb.(type) {
		case Circle:
			c := circleBox(tb, pb, ab, ta, pa, aa)
			if c != nil {
				c.Normal = c.Normal.Neg()
			}
			return c
		case Box:
			return boxBox(ta, pa, aa, tb, pb, ab)
		}
	}
	return nil
}

func circleCircle(a Circle, pa Vec2, aa float64, b Circle, pb Vec2, ab float64) *Contact {
	ca, cb := a.worldCenter(pa, aa), b.worldCenter(pb, ab)
	d := cb.Sub(ca)
	r := a.Radius + b.Radius
	dist2 := d.Len2()
	if dist2 >= r*r {
		return nil
	}
	dist := math.Sqrt(dist2)
	n := Vec2{1, 0}
	if dist > 1e-9 {
		n = d.Mul(1 / dist)
	}
	pen := r - dist
	c := &Contact{Normal: n, Count: 1}
	c.Points[0].Pos = ca.Add(n.Mul(a.Radius - pen*0.5))
	c.Points[0].Penetration = pen
	return c
}

// circleBox returns a contact with the normal pointing from the circle
// to the box.
func circleBox(c Circle, pc Vec2, ac float64, b Box, pb Vec2, ab float64) *Contact {
	cc := c.worldCenter(pc, ac)
	bc := b.worldCenter(pb, ab)
	ba := b.worldAngle(ab)

	// Circle center in box-local space.
	local := cc.Sub(bc).Rotate(-ba)
	clamped := Vec2{
		math.Max(-b.HalfW, math.Min(b.HalfW, local.X)),
		math.Max(-b.HalfH, math.Min(b.HalfH, local.Y)),
	}

	var nLocal Vec2
	var pen float64
	var surface Vec2 // closest point on the box surface, box-local
	if clamped == local {
		// Center inside the box: push out along the least-penetration axis.
		dx := b.HalfW - math.Abs(local.X)
		dy := b.HalfH - math.Abs(local.Y)
		if dx < dy {
			sx := 1.0
			if local.X < 0 {
				sx = -1
			}
			nLocal = Vec2{sx, 0}
			surface = Vec2{sx * b.HalfW, local.Y}
			pen = c.Radius + dx
		} else {
			sy := 1.0
			if local.Y < 0 {
				sy = -1
			}
			nLocal = Vec2{0, sy}
			surface = Vec2{local.X, sy * b.HalfH}
			pen = c.Radius + dy
		}
	} else {
		d := local.Sub(clamped)
		dist := d.Len()
		if dist >= c.Radius {
			return nil
		}
		nLocal = d.Mul(1 / dist)
		surface = clamped
		pen = c.Radius - dist
	}

	// nLocal points from the box to the circle; flip for circle -> box.
	n := nLocal.Rotate(ba).Neg()
	ct := &Contact{Normal: n, Count: 1}
	ct.Points[0].Pos = bc.Add(surface.Rotate(ba))
	ct.Points[0].Penetration = pen
	return ct
}

// boxFace is one face of an OBB in world space.
type boxFace struct {
	normal Vec2 // outward unit normal
	center Vec2 // face center
	half   float64
}

// faces returns the 4 world-space faces in fixed local order:
// +x, -x, +y, -y.
func boxFaces(b Box, pb Vec2, ab float64) [4]boxFace {
	bc := b.worldCenter(pb, ab)
	ba := b.worldAngle(ab)
	type lf struct {
		n    Vec2
		off  float64
		half float64
	}
	local := [4]lf{
		{Vec2{1, 0}, b.HalfW, b.HalfH},
		{Vec2{-1, 0}, b.HalfW, b.HalfH},
		{Vec2{0, 1}, b.HalfH, b.HalfW},
		{Vec2{0, -1}, b.HalfH, b.HalfW},
	}
	var out [4]boxFace
	for i, f := range local {
		out[i] = boxFace{
			normal: f.n.Rotate(ba),
			center: bc.Add(f.n.Mul(f.off).Rotate(ba)),
			half:   f.half,
		}
	}
	return out
}

// bestFace returns the face most aligned with dir.
func bestFace(b Box, pb Vec2, ab float64, dir Vec2) boxFace {
	faces := boxFaces(b, pb, ab)
	best := faces[0]
	bestDot := faces[0].normal.Dot(dir)
	for _, f := range faces[1:] {
		if d := f.normal.Dot(dir); d > bestDot {
			best, bestDot = f, d
		}
	}
	return best
}

// faceVertices returns the two endpoints of a face in a fixed order.
func faceVertices(f boxFace) [2]Vec2 {
	t := f.normal.Perp()
	return [2]Vec2{f.center.Sub(t.Mul(f.half)), f.center.Add(t.Mul(f.half))}
}

// clipSegmentToLine keeps the portion of the segment with
// dot(p, n) <= offset, interpolating at the plane.
func clipSegmentToLine(v [2]Vec2, n Vec2, offset float64) []Vec2 {
	var out []Vec2
	d0 := n.Dot(v[0]) - offset
	d1 := n.Dot(v[1]) - offset
	if d0 <= 0 {
		out = append(out, v[0])
	}
	if d1 <= 0 {
		out = append(out, v[1])
	}
	if d0*d1 < 0 {
		t := d0 / (d0 - d1)
		out = append(out, v[0].Add(v[1].Sub(v[0]).Mul(t)))
	}
	return out
}

// boxBox computes a SAT contact with a clipped 2-point manifold.
// The normal points from box a to box b.
func boxBox(a Box, pa Vec2, aa float64, b Box, pb Vec2, ab float64) *Contact {
	ca := a.worldCenter(pa, aa)
	cb := b.worldCenter(pb, ab)
	angA, angB := a.worldAngle(aa), b.worldAngle(ab)

	// Candidate separating axes: both boxes' local axes, fixed order.
	axes := [4]Vec2{
		Vec2{1, 0}.Rotate(angA),
		Vec2{0, 1}.Rotate(angA),
		Vec2{1, 0}.Rotate(angB),
		Vec2{0, 1}.Rotate(angB),
	}
	radii := func(axis Vec2) (float64, float64) {
		axA := Vec2{1, 0}.Rotate(angA)
		ayA := Vec2{0, 1}.Rotate(angA)
		axB := Vec2{1, 0}.Rotate(angB)
		ayB := Vec2{0, 1}.Rotate(angB)
		ra := a.HalfW*math.Abs(axis.Dot(axA)) + a.HalfH*math.Abs(axis.Dot(ayA))
		rb := b.HalfW*math.Abs(axis.Dot(axB)) + b.HalfH*math.Abs(axis.Dot(ayB))
		return ra, rb
	}

	d := cb.Sub(ca)
	bestSep := math.Inf(-1)
	bestAxis := Vec2{1, 0}
	for _, axis := range axes {
		ra, rb := radii(axis)
		sep := math.Abs(d.Dot(axis)) - (ra + rb)
		if sep > 0 {
			return nil
		}
		if sep > bestSep {
			bestSep, bestAxis = sep, axis
		}
	}

	// Normal points from a to b.
	n := bestAxis
	if d.Dot(n) < 0 {
		n = n.Neg()
	}

	// Reference face: the face most opposed to the contact normal on
	// either box; ties deterministically prefer box a.
	refA := bestFace(a, pa, aa, n)       // face of a facing b
	refB := bestFace(b, pb, ab, n.Neg()) // face of b facing a
	var ref boxFace
	var incBox Box
	var incPos Vec2
	var incAng float64
	flip := false
	if refA.normal.Dot(n) >= refB.normal.Dot(n.Neg()) {
		ref = refA
		incBox, incPos, incAng = b, pb, ab
	} else {
		ref = refB
		incBox, incPos, incAng = a, pa, aa
		flip = true
	}

	// Incident face: on the other box, most opposed to the reference normal.
	inc := bestFace(incBox, incPos, incAng, ref.normal.Neg())
	pts := faceVertices(inc)

	// Clip the incident segment against the reference side planes.
	t := ref.normal.Perp()
	clipped := clipSegmentToLine(pts, t, ref.center.Dot(t)+ref.half)
	if len(clipped) < 2 {
		return nil
	}
	clipped = clipSegmentToLine([2]Vec2{clipped[0], clipped[1]}, t.Neg(), ref.center.Dot(t.Neg())+ref.half)
	if len(clipped) < 2 {
		return nil
	}

	ct := &Contact{Count: 0}
	if flip {
		ct.Normal = ref.normal.Neg()
	} else {
		ct.Normal = ref.normal
	}
	for _, p := range clipped {
		sep := ref.normal.Dot(p.Sub(ref.center))
		if sep > 0 {
			continue
		}
		ct.Points[ct.Count].Pos = p
		ct.Points[ct.Count].Penetration = -sep
		ct.Count++
		if ct.Count == maxContactPoints {
			break
		}
	}
	if ct.Count == 0 {
		return nil
	}
	return ct
}
