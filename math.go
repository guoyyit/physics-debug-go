package physics

import "math"

// Vec2 is an immutable 2D vector. All operations are pure functions, which
// keeps frame math deterministic and safe to share across goroutines.
type Vec2 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

func (v Vec2) Add(w Vec2) Vec2 { return Vec2{v.X + w.X, v.Y + w.Y} }
func (v Vec2) Sub(w Vec2) Vec2 { return Vec2{v.X - w.X, v.Y - w.Y} }
func (v Vec2) Mul(s float64) Vec2 {
	return Vec2{v.X * s, v.Y * s}
}
func (v Vec2) Neg() Vec2          { return Vec2{-v.X, -v.Y} }
func (v Vec2) Dot(w Vec2) float64 { return v.X*w.X + v.Y*w.Y }
func (v Vec2) Cross(w Vec2) float64 {
	return v.X*w.Y - v.Y*w.X
}
func (v Vec2) Len2() float64 { return v.Dot(v) }
func (v Vec2) Len() float64  { return math.Sqrt(v.Len2()) }

// Normalized returns a unit vector. The zero vector is returned unchanged;
// callers must validate inputs before relying on the result.
func (v Vec2) Normalized() Vec2 {
	l := v.Len()
	if l == 0 {
		return Vec2{}
	}
	return v.Mul(1 / l)
}

// AABB is an axis-aligned bounding box.
type AABB struct {
	Min Vec2 `json:"min"`
	Max Vec2 `json:"max"`
}

func (a AABB) Overlaps(b AABB) bool {
	return a.Min.X <= b.Max.X && a.Max.X >= b.Min.X &&
		a.Min.Y <= b.Max.Y && a.Max.Y >= b.Min.Y
}

func (a AABB) ContainsPoint(p Vec2) bool {
	return p.X >= a.Min.X && p.X <= a.Max.X &&
		p.Y >= a.Min.Y && p.Y <= a.Max.Y
}

func (a AABB) Center() Vec2 {
	return Vec2{(a.Min.X + a.Max.X) * 0.5, (a.Min.Y + a.Max.Y) * 0.5}
}

func (a AABB) Area() float64 {
	return (a.Max.X - a.Min.X) * (a.Max.Y - a.Min.Y)
}
