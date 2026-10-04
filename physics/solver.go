package physics

const (
	// baumgarte is the positional correction factor.
	baumgarte = 0.2
	// slop is the tolerated penetration before correction kicks in.
	slop = 0.005
	// restitutionThreshold: impacts slower than this are inelastic.
	restitutionThreshold = 1.0
)

// solver runs a fixed number of sequential-impulse iterations over a
// deterministically ordered contact list.
type solver struct {
	iterations int
}

// crossRV returns w x r for scalar angular velocity w.
func crossRV(w float64, r Vec2) Vec2 { return Vec2{-w * r.Y, w * r.X} }

// prepare precomputes per-point solver data before iterations.
func (s *solver) prepare(contacts []*Contact, dt float64) {
	invDt := 1 / dt
	for _, c := range contacts {
		a, b := c.A, c.B
		n := c.Normal
		t := n.Perp()
		for i := 0; i < c.Count; i++ {
			p := &c.Points[i]
			p.rA = p.Pos.Sub(a.pos)
			p.rB = p.Pos.Sub(b.pos)

			rnA := p.rA.Cross(n)
			rnB := p.rB.Cross(n)
			kn := a.invMass + b.invMass + a.invI*rnA*rnA + b.invI*rnB*rnB
			p.massN = 1 / kn

			rtA := p.rA.Cross(t)
			rtB := p.rB.Cross(t)
			kt := a.invMass + b.invMass + a.invI*rtA*rtA + b.invI*rtB*rtB
			p.massT = 1 / kt

			// Relative normal velocity at the contact point.
			dv := b.vel.Add(crossRV(b.angVel, p.rB)).Sub(a.vel.Add(crossRV(a.angVel, p.rA)))
			vn := dv.Dot(n)

			// Baumgarte positional correction.
			pen := p.Penetration - slop
			if pen < 0 {
				pen = 0
			}
			p.bias = baumgarte * invDt * pen

			// Restitution for fast enough impacts.
			if vn < -restitutionThreshold {
				if rb := -c.Restitution * vn; rb > p.bias {
					p.bias = rb
				}
			}

			p.NormalImpulse = 0
			p.TangentImpulse = 0
		}
	}
}

// applyImpulse applies impulse j at contact offsets to both bodies.
func applyImpulse(c *Contact, p *ContactPoint, j Vec2) {
	a, b := c.A, c.B
	a.vel = a.vel.Sub(j.Mul(a.invMass))
	a.angVel -= a.invI * p.rA.Cross(j)
	b.vel = b.vel.Add(j.Mul(b.invMass))
	b.angVel += b.invI * p.rB.Cross(j)
}

// solveVelocity runs the impulse iterations in a fixed order.
func (s *solver) solveVelocity(contacts []*Contact, dt float64) {
	for it := 0; it < s.iterations; it++ {
		for _, c := range contacts {
			a, b := c.A, c.B
			n := c.Normal
			t := n.Perp()
			for i := 0; i < c.Count; i++ {
				p := &c.Points[i]

				// Normal impulse with accumulated clamping.
				dv := b.vel.Add(crossRV(b.angVel, p.rB)).Sub(a.vel.Add(crossRV(a.angVel, p.rA)))
				vn := dv.Dot(n)
				lambda := -p.massN * (vn - p.bias)
				newImp := p.NormalImpulse + lambda
				if newImp < 0 {
					newImp = 0
				}
				lambda = newImp - p.NormalImpulse
				p.NormalImpulse = newImp
				applyImpulse(c, p, n.Mul(lambda))

				// Friction impulse, clamped to the friction cone.
				dv = b.vel.Add(crossRV(b.angVel, p.rB)).Sub(a.vel.Add(crossRV(a.angVel, p.rA)))
				vt := dv.Dot(t)
				lambda = -p.massT * vt
				maxF := c.Friction * p.NormalImpulse
				newT := p.TangentImpulse + lambda
				if newT > maxF {
					newT = maxF
				} else if newT < -maxF {
					newT = -maxF
				}
				lambda = newT - p.TangentImpulse
				p.TangentImpulse = newT
				applyImpulse(c, p, t.Mul(lambda))
			}
		}
	}
}
