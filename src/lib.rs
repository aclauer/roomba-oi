//! iRobot Roomba Open Interface (600 series / Create 2), sans-io.
//!
//! See the crate README for the shape of the API. Everything here is plain data in and out;
//! open the serial port however you like.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod cmd;
pub mod odometry;
pub mod sensor;
pub mod stream;

/// Default serial baud rate after power-on.
pub const DEFAULT_BAUD: u32 = 115_200;

/// Encoder counts per wheel revolution (spec: 508.8).
pub const COUNTS_PER_REV: f64 = 508.8;
/// Wheel diameter in millimeters (spec: 72.0).
pub const WHEEL_DIAMETER_MM: f64 = 72.0;
/// Distance between the drive wheels in millimeters (spec: 235.0).
pub const WHEELBASE_MM: f64 = 235.0;
/// Maximum commanded wheel or body velocity in mm/s.
pub const MAX_VELOCITY_MM_S: i16 = 500;
/// Maximum turning radius magnitude in mm for [`cmd::drive`].
pub const MAX_RADIUS_MM: i16 = 2000;

pub use sensor::{ChargingState, OiMode, Packet, Sensors};
pub use stream::{Frame, StreamParser};
