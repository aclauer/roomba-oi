//! Differential-drive odometry from the two wheel encoders (packets 43 and 44).
//!
//! The counters are 16-bit and wrap; deltas are taken modulo 2^16 as signed, so any update
//! rate above one wrap per sample (about 29 m of wheel travel) is safe.

use crate::{COUNTS_PER_REV, WHEELBASE_MM, WHEEL_DIAMETER_MM};

/// Robot geometry used to turn counts into meters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Geometry {
    /// Encoder counts per wheel revolution.
    pub counts_per_rev: f64,
    /// Wheel diameter in meters.
    pub wheel_diameter_m: f64,
    /// Distance between wheel contact points in meters.
    pub wheelbase_m: f64,
}

impl Default for Geometry {
    fn default() -> Self {
        Self {
            counts_per_rev: COUNTS_PER_REV,
            wheel_diameter_m: WHEEL_DIAMETER_MM / 1000.0,
            wheelbase_m: WHEELBASE_MM / 1000.0,
        }
    }
}

impl Geometry {
    /// Meters per encoder count.
    pub fn meters_per_count(&self) -> f64 {
        core::f64::consts::PI * self.wheel_diameter_m / self.counts_per_rev
    }
}

/// Planar pose in the odometry frame: meters and radians, x forward, y left, yaw CCW.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose2 {
    /// Meters.
    pub x: f64,
    /// Meters.
    pub y: f64,
    /// Radians, wrapped to (-pi, pi].
    pub yaw: f64,
}

/// Motion in one update.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Delta {
    /// Left wheel travel in meters.
    pub left_m: f64,
    /// Right wheel travel in meters.
    pub right_m: f64,
    /// Body forward travel in meters (mean of the wheels).
    pub forward_m: f64,
    /// Yaw change in radians.
    pub yaw_rad: f64,
}

/// Integrates encoder counts into a pose. Feed every encoder reading in order.
#[derive(Debug, Clone, PartialEq)]
pub struct WheelOdometry {
    geometry: Geometry,
    pose: Pose2,
    last: Option<(u16, u16)>,
}

impl Default for WheelOdometry {
    fn default() -> Self {
        Self::new(Geometry::default())
    }
}

impl WheelOdometry {
    /// Start at the origin with this geometry.
    pub fn new(geometry: Geometry) -> Self {
        Self {
            geometry,
            pose: Pose2::default(),
            last: None,
        }
    }

    /// The geometry in use.
    pub fn geometry(&self) -> &Geometry {
        &self.geometry
    }

    /// Current pose.
    pub fn pose(&self) -> Pose2 {
        self.pose
    }

    /// Forget the previous counts (after a robot reset) without moving the pose.
    pub fn resync(&mut self) {
        self.last = None;
    }

    /// Move the pose back to the origin and forget the previous counts.
    pub fn reset(&mut self) {
        self.pose = Pose2::default();
        self.last = None;
    }

    /// Integrate one pair of raw counts. The first call only latches; it returns a zero delta.
    pub fn update(&mut self, left: u16, right: u16) -> Delta {
        let Some((l0, r0)) = self.last.replace((left, right)) else {
            return Delta::default();
        };
        let dl = left.wrapping_sub(l0) as i16 as f64 * self.geometry.meters_per_count();
        let dr = right.wrapping_sub(r0) as i16 as f64 * self.geometry.meters_per_count();
        let forward = 0.5 * (dl + dr);
        let dyaw = (dr - dl) / self.geometry.wheelbase_m;
        // Midpoint integration: advance along the heading halfway through the turn.
        let mid = self.pose.yaw + 0.5 * dyaw;
        self.pose.x += forward * mid.cos();
        self.pose.y += forward * mid.sin();
        self.pose.yaw = wrap_pi(self.pose.yaw + dyaw);
        Delta {
            left_m: dl,
            right_m: dr,
            forward_m: forward,
            yaw_rad: dyaw,
        }
    }

    /// Wheel velocities in mm/s for a body command, the inverse of what `update` integrates.
    /// Inputs are m/s and rad/s; outputs are what `cmd::drive_direct` takes, as `(right, left)`.
    pub fn wheel_speeds_mm_s(&self, linear_m_s: f64, angular_rad_s: f64) -> (i16, i16) {
        let half = 0.5 * self.geometry.wheelbase_m;
        let right = (linear_m_s + angular_rad_s * half) * 1000.0;
        let left = (linear_m_s - angular_rad_s * half) * 1000.0;
        (clamp_i16(right), clamp_i16(left))
    }
}

fn clamp_i16(v: f64) -> i16 {
    v.round().clamp(i16::MIN as f64, i16::MAX as f64) as i16
}

/// Wrap an angle to (-pi, pi].
pub fn wrap_pi(a: f64) -> f64 {
    use core::f64::consts::PI;
    let mut a = (a + PI) % (2.0 * PI);
    if a <= 0.0 {
        a += 2.0 * PI;
    }
    a - PI
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::PI;

    fn approx(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn meters_per_count_matches_spec_figure() {
        // pi * 72 mm / 508.8 = 0.4446 mm per count.
        let g = Geometry::default();
        assert!((g.meters_per_count() * 1000.0 - 0.44456).abs() < 1e-4);
    }

    #[test]
    fn first_update_only_latches() {
        let mut o = WheelOdometry::default();
        assert_eq!(o.update(100, 200), Delta::default());
        assert_eq!(o.pose(), Pose2::default());
    }

    #[test]
    fn straight_line_one_revolution() {
        let mut o = WheelOdometry::default();
        o.update(0, 0);
        let counts = COUNTS_PER_REV.round() as u16; // 509
        let d = o.update(counts, counts);
        let expected = counts as f64 * o.geometry().meters_per_count();
        assert!(approx(d.forward_m, expected));
        assert!(approx(o.pose().x, expected));
        assert!(approx(o.pose().y, 0.0));
        assert!(approx(o.pose().yaw, 0.0));
    }

    #[test]
    fn spin_in_place_turns_left_when_right_wheel_leads() {
        let mut o = WheelOdometry::default();
        o.update(1000, 1000);
        let d = o.update(900, 1100);
        assert!(d.yaw_rad > 0.0);
        assert!(approx(d.forward_m, 0.0));
        assert!(approx(o.pose().x, 0.0) && approx(o.pose().y, 0.0));
    }

    #[test]
    fn counter_wrap_is_handled() {
        let mut o = WheelOdometry::default();
        o.update(65_530, 65_530);
        let d = o.update(5, 5);
        // 11 counts forward, not -65525.
        assert!(approx(d.left_m, 11.0 * o.geometry().meters_per_count()));
        let mut back = WheelOdometry::default();
        back.update(5, 5);
        let d = back.update(65_530, 65_530);
        assert!(approx(d.left_m, -11.0 * back.geometry().meters_per_count()));
    }

    #[test]
    fn square_path_returns_home() {
        let mut o = WheelOdometry::default();
        o.update(0, 0);
        let mut l: u16 = 0;
        let mut r: u16 = 0;
        let mpc = o.geometry().meters_per_count();
        let side = (1.0 / mpc).round() as u16;
        // A quarter turn in place: each wheel moves (pi/2 * wheelbase / 2).
        let quarter = ((PI / 2.0) * o.geometry().wheelbase_m / 2.0 / mpc).round() as u16;
        for _ in 0..4 {
            l = l.wrapping_add(side);
            r = r.wrapping_add(side);
            o.update(l, r);
            l = l.wrapping_sub(quarter);
            r = r.wrapping_add(quarter);
            o.update(l, r);
        }
        let p = o.pose();
        // Rounding the quarter turn to whole counts leaves a few millimeters.
        assert!(p.x.abs() < 0.01 && p.y.abs() < 0.01, "{p:?}");
        assert!(wrap_pi(p.yaw).abs() < 0.02, "{p:?}");
    }

    #[test]
    fn wheel_speeds_invert_kinematics() {
        let o = WheelOdometry::default();
        assert_eq!(o.wheel_speeds_mm_s(0.2, 0.0), (200, 200));
        let (r, l) = o.wheel_speeds_mm_s(0.0, 1.0);
        assert_eq!(r, 118); // 0.235 / 2 * 1000, rounded
        assert_eq!(l, -118);
        assert_eq!(o.wheel_speeds_mm_s(100.0, 0.0), (i16::MAX, i16::MAX));
    }

    #[test]
    fn wrap_pi_bounds() {
        assert!(approx(wrap_pi(3.0 * PI), PI));
        assert!(approx(wrap_pi(-PI), PI));
        assert!(approx(wrap_pi(0.5), 0.5));
        assert!(approx(wrap_pi(-3.5 * PI), 0.5 * PI));
    }
}
