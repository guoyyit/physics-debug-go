package physics

import "math"

// BodyID uniquely identifies a body within a world. Zero is invalid.
type BodyID uint64

// BodyKind controls how a body participates in simulation.
type BodyKind int

const (
	// Static bodies never move and have infinite mass.
	Static BodyKind = iota
	// Kinematic bodies move by velocity only, infinite mass.
	Kinematic
	// Dynamic bodies are fully simulated.
	Dynamic
)

func (k BodyKind) String() string {
	switch k {
	case Static:
		return "static"
	case Kinematic:
		return "kinematic"
	case Dynamic:
		return "dynamic"
	}
	return "unknown"
}

// BodyDef describes a body to be added to the world.
type BodyDef struct {
	ID          BodyID
	Kind        BodyKind
	Pos         Vec2
	Angle       float64
	Vel         Vec2
	AngVel      float64
	Layer       uint32 // collision layer bits this body belongs to
	Mask        uint32 // layer bits this body collides with (0 => all)
	Density     float64
	Friction    float64
	Restitution float64
	Shapes      []Shape
}

// Body is a rigid body. Users never hold *Body across steps; all
// interaction goes through World methods.
type Body struct {
	id          BodyID
	kind        BodyKind
	pos         Vec2
	angle       float64
	vel         Vec2
	angVel      float64
	layer       uint32
	mask        uint32
	friction    float64
	restitution float64
	shapes      []Shape
	invMass     float64
	invI        float64
}

// newBody validates def and builds a Body.
func newBody(def BodyDef) (*Body, error) {
	const op = "World.AddBody"
	if def.ID == 0 {
		return nil, newError(ErrInvalidParam, op, "body ID 0 is reserved and invalid")
	}
	if def.Kind < Static || def.Kind > Dynamic {
		return nil, newError(ErrInvalidParam, op,
			"body %d: unknown kind %d", def.ID, def.Kind)
	}
	if !def.Pos.IsFinite() || !def.Vel.IsFinite() {
		return nil, newError(ErrInvalidParam, op,
			"body %d: position/velocity must be finite, got pos=%+v vel=%+v",
			def.ID, def.Pos, def.Vel)
	}
	if math.IsNaN(def.Angle) || math.IsInf(def.Angle, 0) ||
		math.IsNaN(def.AngVel) || math.IsInf(def.AngVel, 0) {
		return nil, newError(ErrInvalidParam, op,
			"body %d: angle/angular velocity must be finite", def.ID)
	}
	if len(def.Shapes) == 0 {
		return nil, newError(ErrDegenerateShape, op,
			"body %d: at least one shape is required", def.ID)
	}
	for i, s := range def.Shapes {
		s = normalizeShape(s)
		if s == nil {
			return nil, newError(ErrDegenerateShape, op,
				"body %d: shape %d is nil", def.ID, i)
		}
		if err := s.Validate(); err != nil {
			return nil, newError(ErrDegenerateShape, op,
				"body %d shape %d (%s): %s", def.ID, i, s.Kind(), err)
		}
		def.Shapes[i] = s
	}
	if def.Density < 0 || math.IsNaN(def.Density) || math.IsInf(def.Density, 0) {
		return nil, newError(ErrInvalidParam, op,
			"body %d: density must be a finite number >= 0, got %v", def.ID, def.Density)
	}
	if def.Friction < 0 || math.IsNaN(def.Friction) || math.IsInf(def.Friction, 0) {
		return nil, newError(ErrInvalidParam, op,
			"body %d: friction must be a finite number >= 0, got %v", def.ID, def.Friction)
	}
	if def.Restitution < 0 || math.IsNaN(def.Restitution) || math.IsInf(def.Restitution, 0) {
		return nil, newError(ErrInvalidParam, op,
			"body %d: restitution must be a finite number >= 0, got %v", def.ID, def.Restitution)
	}

	b := &Body{
		id:          def.ID,
		kind:        def.Kind,
		pos:         def.Pos,
		angle:       def.Angle,
		vel:         def.Vel,
		angVel:      def.AngVel,
		layer:       def.Layer,
		mask:        def.Mask,
		friction:    def.Friction,
		restitution: def.Restitution,
		shapes:      make([]Shape, len(def.Shapes)),
	}
	copy(b.shapes, def.Shapes)

	if def.Kind == Dynamic {
		d := def.Density
		if d == 0 {
			d = 1 // dynamic bodies default to density 1
		}
		var mass, inertia float64
		for _, s := range b.shapes {
			m, i := s.MassData(d)
			mass += m
			inertia += i
		}
		if mass <= 0 {
			return nil, newError(ErrDegenerateShape, op,
				"body %d: dynamic body has zero total mass", def.ID)
		}
		b.invMass = 1 / mass
		b.invI = 1 / inertia
	}
	return b, nil
}

// aabb computes the union of the body's shape AABBs.
func (b *Body) aabb() AABB {
	box := b.shapes[0].AABB(b.pos, b.angle)
	for _, s := range b.shapes[1:] {
		box = box.Union(s.AABB(b.pos, b.angle))
	}
	return box
}

// shouldCollide reports whether two bodies can generate contacts.
func shouldCollide(a, b *Body) bool {
	// Two non-dynamic bodies never interact.
	if a.kind != Dynamic && b.kind != Dynamic {
		return false
	}
	// Layer/mask test: 0 mask means "collides with everything".
	am := a.mask
	if am == 0 {
		am = ^uint32(0)
	}
	bm := b.mask
	if bm == 0 {
		bm = ^uint32(0)
	}
	return am&b.layer != 0 && bm&a.layer != 0
}

// normalizeShape converts pointer shape values to their value forms so
// collision dispatch only ever sees Circle and Box values.
func normalizeShape(s Shape) Shape {
	switch v := s.(type) {
	case *Circle:
		if v == nil {
			return nil
		}
		return *v
	case *Box:
		if v == nil {
			return nil
		}
		return *v
	}
	return s
}
