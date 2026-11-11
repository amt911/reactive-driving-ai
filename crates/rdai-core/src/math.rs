//! Minimal 2D math. Units are SI (metres, seconds, radians); angles are counter-clockwise from +X.

use core::ops::{Add, AddAssign, Mul, Neg, Sub, SubAssign};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub const ZERO: Vec2 = Vec2 { x: 0.0, y: 0.0 };

    #[inline]
    pub const fn new(x: f32, y: f32) -> Self {
        Vec2 { x, y }
    }
    #[inline]
    pub fn from_angle(a: f32) -> Self {
        Vec2::new(a.cos(), a.sin())
    }
    #[inline]
    pub fn dot(self, o: Vec2) -> f32 {
        self.x * o.x + self.y * o.y
    }
    /// Z component of the 3D cross product.
    #[inline]
    pub fn cross(self, o: Vec2) -> f32 {
        self.x * o.y - self.y * o.x
    }
    #[inline]
    pub fn length_sq(self) -> f32 {
        self.dot(self)
    }
    #[inline]
    pub fn length(self) -> f32 {
        self.length_sq().sqrt()
    }
    #[inline]
    pub fn normalized(self) -> Vec2 {
        let l = self.length();
        if l > 1e-9 {
            self * (1.0 / l)
        } else {
            Vec2::ZERO
        }
    }
    /// Rotated +90 degrees (points to the left of the direction).
    #[inline]
    pub fn perp(self) -> Vec2 {
        Vec2::new(-self.y, self.x)
    }
    #[inline]
    pub fn angle(self) -> f32 {
        self.y.atan2(self.x)
    }
    #[inline]
    pub fn rotated(self, a: f32) -> Vec2 {
        let (s, c) = a.sin_cos();
        Vec2::new(self.x * c - self.y * s, self.x * s + self.y * c)
    }
    #[inline]
    pub fn lerp(self, o: Vec2, t: f32) -> Vec2 {
        self + (o - self) * t
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    #[inline]
    fn add(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x + o.x, self.y + o.y)
    }
}
impl Sub for Vec2 {
    type Output = Vec2;
    #[inline]
    fn sub(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x - o.x, self.y - o.y)
    }
}
impl Mul<f32> for Vec2 {
    type Output = Vec2;
    #[inline]
    fn mul(self, k: f32) -> Vec2 {
        Vec2::new(self.x * k, self.y * k)
    }
}
impl Neg for Vec2 {
    type Output = Vec2;
    #[inline]
    fn neg(self) -> Vec2 {
        Vec2::new(-self.x, -self.y)
    }
}
impl AddAssign for Vec2 {
    #[inline]
    fn add_assign(&mut self, o: Vec2) {
        self.x += o.x;
        self.y += o.y;
    }
}
impl SubAssign for Vec2 {
    #[inline]
    fn sub_assign(&mut self, o: Vec2) {
        self.x -= o.x;
        self.y -= o.y;
    }
}

/// Wraps an angle to (-pi, pi].
#[inline]
pub fn wrap_angle(a: f32) -> f32 {
    let two_pi = core::f32::consts::TAU;
    let mut r = a % two_pi;
    if r <= -core::f32::consts::PI {
        r += two_pi;
    } else if r > core::f32::consts::PI {
        r -= two_pi;
    }
    r
}

/// Signed curvature of the circle through three points (Menger curvature). Positive = left turn.
#[inline]
pub fn menger_curvature(a: Vec2, b: Vec2, c: Vec2) -> f32 {
    let ab = b - a;
    let bc = c - b;
    let ca = a - c;
    let denom = ab.length() * bc.length() * ca.length();
    if denom < 1e-9 {
        0.0
    } else {
        2.0 * ab.cross(bc) / denom
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_angle_stays_in_range() {
        for k in -20..20 {
            let a = k as f32 * 0.7;
            let w = wrap_angle(a);
            assert!(w > -core::f32::consts::PI - 1e-6 && w <= core::f32::consts::PI + 1e-6);
            assert!(
                ((a - w) / core::f32::consts::TAU).fract().abs() < 1e-3
                    || ((a - w) / core::f32::consts::TAU).fract().abs() > 0.999
            );
        }
    }

    #[test]
    fn curvature_of_circle_matches_radius() {
        let r = 50.0;
        let p = |t: f32| Vec2::new(r * t.cos(), r * t.sin());
        let k = menger_curvature(p(0.0), p(0.1), p(0.2));
        assert!((k - 1.0 / r).abs() < 1e-4, "k = {k}");
        let k_right = menger_curvature(p(0.2), p(0.1), p(0.0));
        assert!((k_right + 1.0 / r).abs() < 1e-4);
    }
}
