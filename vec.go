package vec

import (
	"fmt"
	"math"
)

type Vec2 struct {
	X, Y float64
}

// String returns string representation of this vector.
func (v Vec2) String() string {
	return fmt.Sprintf("Vec2{X: %f, Y: %f}", v.X, v.Y)
}

// Equal checks if two vectors are equal. (Be careful when comparing floating point numbers!)
func (v Vec2) Equal(other Vec2) bool {
	return v.X == other.X && v.Y == other.Y
}

// Add two vector
func (v Vec2) Add(other Vec2) Vec2 {
	return Vec2{v.X + other.X, v.Y + other.Y}
}

// Div divides
func (v Vec2) Div(number float64) Vec2 {
	return Vec2{v.X / number, v.Y / number}
}

// Sub returns this - other
func (v Vec2) Sub(other Vec2) Vec2 {
	return Vec2{v.X - other.X, v.Y - other.Y}
}

// Neg negates a vector.
func (v Vec2) Neg() Vec2 {
	return Vec2{-v.X, -v.Y}
}

// NegY negates Y of vector.
func (v Vec2) NegY() Vec2 {
	return Vec2{v.X, -v.Y}
}

// Scale scales vector
func (v Vec2) Scale(s float64) Vec2 {
	return Vec2{v.X * s, v.Y * s}
}

// Mult scales vector
func (v Vec2) Mult(other Vec2) Vec2 {
	return Vec2{v.X * other.X, v.Y * other.Y}
}

// Dot returns dot product
func (v Vec2) Dot(other Vec2) float64 {
	return v.X*other.X + v.Y*other.Y
}

// Cross calculates the 2D vector cross product analog.
// The cross product of 2D vectors results in a 3D vector with only a z component.
// This function returns the magnitude of the z value.
func (v Vec2) Cross(other Vec2) float64 {
	return v.X*other.Y - v.Y*other.X
}

// Perp returns a perpendicular vector. (90 degree rotation)
func (v Vec2) Perp() Vec2 {
	return Vec2{-v.Y, v.X}
}

// ReversePerp returns a perpendicular vector. (-90 degree rotation)
func (v Vec2) ReversePerp() Vec2 {
	return Vec2{v.Y, -v.X}
}

// Returns the vector projection onto other.
func (v Vec2) Project(other Vec2) Vec2 {
	return other.Scale(v.Dot(other) / other.Dot(other))
}

// ToAngle returns the angular direction v is pointing in (in radians).
func (v Vec2) ToAngle() float64 {
	return math.Atan2(v.Y, v.X)
}

// RotateComplex uses complex number multiplication to rotate this by other.
//
// Scaling will occur if this is not a unit vector.
func (v Vec2) RotateComplex(other Vec2) Vec2 {
	return Vec2{v.X*other.X - v.Y*other.Y, v.X*other.Y + v.Y*other.X}
}

// Rotate a vector by an angle in radians
func (v Vec2) Rotate(angle float64) Vec2 {
	return Vec2{
		X: v.X*math.Cos(angle) - v.Y*math.Sin(angle),
		Y: v.X*math.Sin(angle) + v.Y*math.Cos(angle),
	}
}

// UnrotateComplex is inverse of Vector.Rotate().
func (v Vec2) UnrotateComplex(other Vec2) Vec2 {
	return Vec2{v.X*other.X + v.Y*other.Y, v.Y*other.X - v.X*other.Y}
}

// LengthSq returns the squared length of this vector.
//
// Faster than  Vector.Length() when you only need to compare lengths.
func (v Vec2) LengthSq() float64 {
	return v.Dot(v)
}

// Mag returns the magnitude of this vector
func (v Vec2) Mag() float64 {
	return math.Sqrt(v.Dot(v))
}

// Unit returns a normalized copy of this vector (unit vector).
func (v Vec2) Unit() Vec2 {
	// return v.Mult(1.0 / (v.Length() + math.SmallestNonzeroFloat64))
	return v.Scale(1.0 / (v.Mag() + 1e-50))
}

// Spherical linearly interpolate between this and other.
func (v Vec2) LerpSpherical(other Vec2, t float64) Vec2 {
	dot := v.Unit().Dot(other.Unit())
	omega := math.Acos(clamp(dot, -1, 1))

	if omega < 1e-3 {
		return v.Lerp(other, t)
	}

	denom := 1.0 / math.Sin(omega)
	return v.Scale(math.Sin((1.0-t)*omega) * denom).Add(other.Scale(math.Sin(t*omega) * denom))
}

// Spherical linearly interpolate between this towards other by no more than angle a radians.
func (v Vec2) LerpSphericalClamp(other Vec2, angle float64) Vec2 {
	dot := v.Unit().Dot(other.Unit())
	omega := math.Acos(clamp(dot, -1, 1))
	return v.LerpSpherical(other, math.Min(angle, omega)/omega)
}

// ClampMag clamps this vector magnitude to m.
func (v Vec2) ClampMag(m float64) Vec2 {
	if v.Dot(v) > m*m {
		return v.Unit().Scale(m)
	}
	return Vec2{v.X, v.Y}
}

// Lerp linearly interpolates between this and other vector.
func (v Vec2) Lerp(other Vec2, t float64) Vec2 {
	return v.Scale(1.0 - t).Add(other.Scale(t))
}

// LerpDistance linearly interpolates between this towards other by distance dist.
func (v Vec2) LerpDistance(other Vec2, dist float64) Vec2 {
	return v.Add(other.Sub(v).ClampMag(dist))
}

// Returns the distance between this and other.
func (v Vec2) Distance(other Vec2) float64 {
	return math.Hypot(v.X-other.X, v.Y-other.Y)
	// return v.Sub(other).Length()
}

// DistanceSq returns the squared distance between this and other.
//
// Faster than Vector.Distance() when you only need to compare distances.
func (v Vec2) DistanceSq(other Vec2) float64 {
	return v.Sub(other).LengthSq()
}

// IsNear returns true if the distance between this and other is less than dist.
func (v Vec2) IsNear(other Vec2, dist float64) bool {
	return v.DistanceSq(other) < dist*dist
}

// Round returns the nearest integer Vector, rounding half away from zero.
func (v Vec2) Round() Vec2 {
	return Vec2{math.Round(v.X), math.Round(v.Y)}
}

// Floor vector
func (v Vec2) Floor() Vec2 {
	return Vec2{math.Floor(v.X), math.Floor(v.Y)}
}

// ForAngle returns the unit length vector for the given angle (in radians).
func ForAngle(a float64) Vec2 {
	return Vec2{math.Cos(a), math.Sin(a)}
}

func clamp(f, min, max float64) float64 {
	if f > min {
		return math.Min(f, max)
	} else {
		return math.Min(min, max)
	}
}

func clamp01(f float64) float64 {
	return math.Max(0, math.Min(f, 1))
}
