package physics

import "fmt"

// ShapeKind enumerates the supported collision shape types.
type ShapeKind int

const (
	// ShapeCircle is a circle defined by its radius.
	ShapeCircle ShapeKind = iota
	// ShapeBox is an axis-aligned box defined by its half extents.
	ShapeBox
)

// Shape is implemented by collision shapes. Shapes are treated as immutable
// after creation, so they can be shared freely between frame snapshots.
type Shape interface {
	Kind() ShapeKind
	// AABB returns the world-space axis-aligned bounding box of the shape
	// when centered at the given position.
	AABB(center Vec2) AABB
}

// Circle is a collision shape defined by its radius.
type Circle struct {
	Radius float64 `json:"radius"`
}

func (c Circle) Kind() ShapeKind { return ShapeCircle }

func (c Circle) AABB(center Vec2) AABB {
	return AABB{
		Min: Vec2{center.X - c.Radius, center.Y - c.Radius},
		Max: Vec2{center.X + c.Radius, center.Y + c.Radius},
	}
}

// Box is an axis-aligned collision shape defined by its half extents.
// Rotation is stored on the body for debug purposes; collision uses the
// axis-aligned enclosure of the box.
type Box struct {
	HalfW float64 `json:"half_w"`
	HalfH float64 `json:"half_h"`
}

func (b Box) Kind() ShapeKind { return ShapeBox }

func (b Box) AABB(center Vec2) AABB {
	return AABB{
		Min: Vec2{center.X - b.HalfW, center.Y - b.HalfH},
		Max: Vec2{center.X + b.HalfW, center.Y + b.HalfH},
	}
}

// validateShape rejects degenerate shapes.
func validateShape(s Shape) error {
	switch sh := s.(type) {
	case Circle:
		if sh.Radius <= 0 {
			return &Error{
				Code:    ErrInvalidShape,
				Param:   "radius",
				Value:   fmt.Sprintf("%g", sh.Radius),
				Message: "circle radius must be strictly positive",
			}
		}
	case Box:
		if sh.HalfW <= 0 {
			return &Error{
				Code:    ErrInvalidShape,
				Param:   "half_w",
				Value:   fmt.Sprintf("%g", sh.HalfW),
				Message: "box half width must be strictly positive",
			}
		}
		if sh.HalfH <= 0 {
			return &Error{
				Code:    ErrInvalidShape,
				Param:   "half_h",
				Value:   fmt.Sprintf("%g", sh.HalfH),
				Message: "box half height must be strictly positive",
			}
		}
	default:
		return &Error{
			Code:    ErrInvalidShape,
			Message: fmt.Sprintf("unsupported shape type %T", s),
		}
	}
	return nil
}
