package physics

// BodyType distinguishes static scenery from dynamic rigid bodies.
type BodyType int

const (
	// BodyStatic does not move under gravity and has no velocity.
	BodyStatic BodyType = iota
	// BodyDynamic is integrated by the fixed timestep.
	BodyDynamic
)

// Body is a rigid body with a collision shape. Bodies are value types; the
// world keeps sorted slices of them and publishes immutable frame snapshots.
type Body struct {
	ID     string   `json:"id"`
	Type   BodyType `json:"type"`
	Layer  uint32   `json:"layer"`
	Pos    Vec2     `json:"pos"`
	Rot    float64  `json:"rot"`
	Vel    Vec2     `json:"vel"`
	AngVel float64  `json:"ang_vel"`
	Shape  Shape    `json:"-"`
}

// AABB returns the world-space axis-aligned bounding box of the body.
func (b Body) AABB() AABB {
	return b.Shape.AABB(b.Pos)
}

// clone returns a copy that shares the (immutable) shape value.
func (b Body) clone() Body {
	return b
}
