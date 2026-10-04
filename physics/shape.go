package physics

import "math"

// ShapeKind identifies the concrete type of a Shape.
type ShapeKind int

const (
	ShapeCircle ShapeKind = iota + 1
	ShapeBox
)

func (k ShapeKind) String() string {
	switch k {
	case ShapeCircle:
		return "circle"
	case ShapeBox:
		return "box"
	}
	return "unknown"
}

// Shape is a collision shape attached to a body. Offset is the shape
// center in body-local space; Angle (boxes only) is the local rotation.
type Shape interface {
	Kind() ShapeKind
	// Validate returns an ErrDegenerateShape error when unusable.
	Validate() error
	// AABB computes the world-space bounds given the body transform.
	AABB(bodyPos Vec2, bodyAngle float64) AABB
	// MassData returns mass and rotational inertia about the body origin
	// for the given density.
	MassData(density float64) (mass, inertia float64)
	// worldCenter returns the shape center in world space.
	worldCenter(bodyPos Vec2, bodyAngle float64) Vec2
}

// Circle is a disc collision shape.
type Circle struct {
	Radius float64
	Offset Vec2
}

func (c Circle) Kind() ShapeKind { return ShapeCircle }

func (c Circle) Validate() error {
	if math.IsNaN(c.Radius) || math.IsInf(c.Radius, 0) || c.Radius <= 0 {
		return newError(ErrDegenerateShape, "Circle.Validate",
			"radius must be a finite number > 0, got %v", c.Radius)
	}
	if !c.Offset.IsFinite() {
		return newError(ErrDegenerateShape, "Circle.Validate",
			"offset contains non-finite component: %+v", c.Offset)
	}
	return nil
}

func (c Circle) worldCenter(bodyPos Vec2, bodyAngle float64) Vec2 {
	return bodyPos.Add(c.Offset.Rotate(bodyAngle))
}

func (c Circle) AABB(bodyPos Vec2, bodyAngle float64) AABB {
	ctr := c.worldCenter(bodyPos, bodyAngle)
	r := Vec2{c.Radius, c.Radius}
	return AABB{Min: ctr.Sub(r), Max: ctr.Add(r)}
}

func (c Circle) MassData(density float64) (mass, inertia float64) {
	mass = density * math.Pi * c.Radius * c.Radius
	// Inertia about the body origin: disc inertia + parallel axis term.
	inertia = mass * (0.5*c.Radius*c.Radius + c.Offset.Len2())
	return mass, inertia
}

// Box is an oriented rectangle collision shape.
type Box struct {
	HalfW, HalfH float64
	Offset       Vec2
	Angle        float64 // local rotation relative to the body
}

func (b Box) Kind() ShapeKind { return ShapeBox }

func (b Box) Validate() error {
	if math.IsNaN(b.HalfW) || math.IsInf(b.HalfW, 0) || b.HalfW <= 0 {
		return newError(ErrDegenerateShape, "Box.Validate",
			"half width must be a finite number > 0, got %v", b.HalfW)
	}
	if math.IsNaN(b.HalfH) || math.IsInf(b.HalfH, 0) || b.HalfH <= 0 {
		return newError(ErrDegenerateShape, "Box.Validate",
			"half height must be a finite number > 0, got %v", b.HalfH)
	}
	if !b.Offset.IsFinite() {
		return newError(ErrDegenerateShape, "Box.Validate",
			"offset contains non-finite component: %+v", b.Offset)
	}
	if math.IsNaN(b.Angle) || math.IsInf(b.Angle, 0) {
		return newError(ErrDegenerateShape, "Box.Validate",
			"angle must be finite, got %v", b.Angle)
	}
	return nil
}

func (b Box) worldCenter(bodyPos Vec2, bodyAngle float64) Vec2 {
	return bodyPos.Add(b.Offset.Rotate(bodyAngle))
}

// worldAngle returns the box orientation in world space.
func (b Box) worldAngle(bodyAngle float64) float64 { return bodyAngle + b.Angle }

// corners returns the 4 world-space corners in a fixed order:
// (-w,-h), (+w,-h), (+w,+h), (-w,+h) in box-local space.
func (b Box) corners(bodyPos Vec2, bodyAngle float64) [4]Vec2 {
	ctr := b.worldCenter(bodyPos, bodyAngle)
	a := b.worldAngle(bodyAngle)
	local := [4]Vec2{
		{-b.HalfW, -b.HalfH},
		{b.HalfW, -b.HalfH},
		{b.HalfW, b.HalfH},
		{-b.HalfW, b.HalfH},
	}
	var out [4]Vec2
	for i, p := range local {
		out[i] = ctr.Add(p.Rotate(a))
	}
	return out
}

func (b Box) AABB(bodyPos Vec2, bodyAngle float64) AABB {
	cs := b.corners(bodyPos, bodyAngle)
	min, max := cs[0], cs[0]
	for _, p := range cs[1:] {
		min.X = math.Min(min.X, p.X)
		min.Y = math.Min(min.Y, p.Y)
		max.X = math.Max(max.X, p.X)
		max.Y = math.Max(max.Y, p.Y)
	}
	return AABB{Min: min, Max: max}
}

func (b Box) MassData(density float64) (mass, inertia float64) {
	w, h := 2*b.HalfW, 2*b.HalfH
	mass = density * w * h
	// Inertia about the body origin: box inertia + parallel axis term.
	inertia = mass*(w*w+h*h)/12 + mass*b.Offset.Len2()
	return mass, inertia
}
