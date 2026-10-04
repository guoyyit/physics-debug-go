package physics

import "math"

// Vec2 is a 2D vector. All operations are pure float64 math with a fixed
// evaluation order, so results are reproducible across runs.
type Vec2 struct {
	X, Y float64
}

func (v Vec2) Add(o Vec2) Vec2    { return Vec2{v.X + o.X, v.Y + o.Y} }
func (v Vec2) Sub(o Vec2) Vec2    { return Vec2{v.X - o.X, v.Y - o.Y} }
func (v Vec2) Mul(s float64) Vec2 { return Vec2{v.X * s, v.Y * s} }
func (v Vec2) Neg() Vec2          { return Vec2{-v.X, -v.Y} }

// Dot returns the dot product.
func (v Vec2) Dot(o Vec2) float64 { return v.X*o.X + v.Y*o.Y }

// Cross returns the z component of the 3D cross product.
func (v Vec2) Cross(o Vec2) float64 { return v.X*o.Y - v.Y*o.X }

func (v Vec2) Len2() float64 { return v.X*v.X + v.Y*v.Y }
func (v Vec2) Len() float64  { return math.Sqrt(v.Len2()) }

// Norm returns the unit vector. The zero vector normalizes to itself.
func (v Vec2) Norm() Vec2 {
	l := v.Len()
	if l == 0 {
		return Vec2{}
	}
	return v.Mul(1 / l)
}

// Perp returns the vector rotated 90 degrees counter-clockwise.
func (v Vec2) Perp() Vec2 { return Vec2{-v.Y, v.X} }

// Rotate rotates the vector by angle (radians, counter-clockwise).
func (v Vec2) Rotate(angle float64) Vec2 {
	c, s := math.Cos(angle), math.Sin(angle)
	return Vec2{c*v.X - s*v.Y, s*v.X + c*v.Y}
}

// IsFinite reports whether both components are finite (no NaN/Inf).
func (v Vec2) IsFinite() bool {
	return !math.IsNaN(v.X) && !math.IsNaN(v.Y) &&
		!math.IsInf(v.X, 0) && !math.IsInf(v.Y, 0)
}

// AABB is an axis-aligned bounding box.
type AABB struct {
	Min, Max Vec2
}

// NewAABB builds an AABB containing both points.
func NewAABB(a, b Vec2) AABB {
	return AABB{
		Min: Vec2{math.Min(a.X, b.X), math.Min(a.Y, b.Y)},
		Max: Vec2{math.Max(a.X, b.X), math.Max(a.Y, b.Y)},
	}
}

// Union returns the smallest AABB containing both.
func (a AABB) Union(b AABB) AABB {
	return AABB{
		Min: Vec2{math.Min(a.Min.X, b.Min.X), math.Min(a.Min.Y, b.Min.Y)},
		Max: Vec2{math.Max(a.Max.X, b.Max.X), math.Max(a.Max.Y, b.Max.Y)},
	}
}

// Overlaps reports whether the two boxes intersect (touching counts).
func (a AABB) Overlaps(b AABB) bool {
	return a.Min.X <= b.Max.X && a.Max.X >= b.Min.X &&
		a.Min.Y <= b.Max.Y && a.Max.Y >= b.Min.Y
}

// Contains reports whether point p is inside (boundary counts).
func (a AABB) Contains(p Vec2) bool {
	return p.X >= a.Min.X && p.X <= a.Max.X && p.Y >= a.Min.Y && p.Y <= a.Max.Y
}

// Expand grows the box by m on every side.
func (a AABB) Expand(m float64) AABB {
	return AABB{Min: Vec2{a.Min.X - m, a.Min.Y - m}, Max: Vec2{a.Max.X + m, a.Max.Y + m}}
}

// IsValid reports whether the box is finite and Min <= Max per axis.
func (a AABB) IsValid() bool {
	return a.Min.IsFinite() && a.Max.IsFinite() &&
		a.Min.X <= a.Max.X && a.Min.Y <= a.Max.Y
}

// Center returns the box center.
func (a AABB) Center() Vec2 { return a.Min.Add(a.Max).Mul(0.5) }
