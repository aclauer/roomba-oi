//! Command encoding. Each function returns the exact bytes to write to the port.

use crate::sensor::Packet;
use crate::{MAX_RADIUS_MM, MAX_VELOCITY_MM_S};

/// Open Interface opcodes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
#[allow(missing_docs)]
pub enum Opcode {
    Reset = 7,
    Start = 128,
    Baud = 129,
    Control = 130,
    Safe = 131,
    Full = 132,
    Power = 133,
    Spot = 134,
    Clean = 135,
    Max = 136,
    Drive = 137,
    Motors = 138,
    Leds = 139,
    Song = 140,
    Play = 141,
    Sensors = 142,
    SeekDock = 143,
    PwmMotors = 144,
    DriveDirect = 145,
    DrivePwm = 146,
    Stream = 148,
    QueryList = 149,
    PauseResumeStream = 150,
    SchedulingLeds = 162,
    DigitLedsRaw = 163,
    DigitLedsAscii = 164,
    Buttons = 165,
    Schedule = 167,
    SetDayTime = 168,
    Stop = 173,
}

/// Baud rate codes for [`baud`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[allow(missing_docs)]
pub enum BaudCode {
    B300 = 0,
    B600 = 1,
    B1200 = 2,
    B2400 = 3,
    B4800 = 4,
    B9600 = 5,
    B14400 = 6,
    B19200 = 7,
    B28800 = 8,
    B38400 = 9,
    B57600 = 10,
    B115200 = 11,
}

/// Turning radius for [`drive`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Radius {
    /// Arc about a point `mm` to the side of the robot; positive turns left. Clamped to ±2000.
    Arc(i16),
    /// Drive straight (special value 0x8000).
    Straight,
    /// Spin in place clockwise (special value -1).
    SpinClockwise,
    /// Spin in place counter-clockwise (special value 1).
    SpinCounterClockwise,
}

impl Radius {
    fn code(self) -> u16 {
        match self {
            Radius::Arc(mm) => mm.clamp(-MAX_RADIUS_MM, MAX_RADIUS_MM) as u16,
            Radius::Straight => 0x8000,
            Radius::SpinClockwise => 0xFFFF,
            Radius::SpinCounterClockwise => 0x0001,
        }
    }
}

/// Which cleaning motors to run with [`motors`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Motors {
    /// Main (center) brush on.
    pub main_brush: bool,
    /// Side brush on.
    pub side_brush: bool,
    /// Vacuum on.
    pub vacuum: bool,
    /// Run the main brush in the reverse direction.
    pub main_brush_reverse: bool,
    /// Run the side brush clockwise instead of the default counter-clockwise.
    pub side_brush_clockwise: bool,
}

impl Motors {
    /// Everything off.
    pub const OFF: Motors = Motors {
        main_brush: false,
        side_brush: false,
        vacuum: false,
        main_brush_reverse: false,
        side_brush_clockwise: false,
    };

    /// Main brush, side brush and vacuum on, default directions.
    pub const ALL: Motors = Motors {
        main_brush: true,
        side_brush: true,
        vacuum: true,
        main_brush_reverse: false,
        side_brush_clockwise: false,
    };

    fn bits(self) -> u8 {
        (self.side_brush as u8)
            | (self.vacuum as u8) << 1
            | (self.main_brush as u8) << 2
            | (self.side_brush_clockwise as u8) << 3
            | (self.main_brush_reverse as u8) << 4
    }
}

fn be16(v: u16) -> [u8; 2] {
    v.to_be_bytes()
}

/// Enter Passive mode. Required before any other command after power-on.
pub fn start() -> Vec<u8> {
    vec![Opcode::Start as u8]
}

/// Reset the robot as if the battery were pulled. The robot replies with boot text.
pub fn reset() -> Vec<u8> {
    vec![Opcode::Reset as u8]
}

/// Stop the OI: all motors off, Off mode. Send [`start`] to talk again.
pub fn stop() -> Vec<u8> {
    vec![Opcode::Stop as u8]
}

/// Change the baud rate. Wait 100 ms before using the new rate.
pub fn baud(code: BaudCode) -> Vec<u8> {
    vec![Opcode::Baud as u8, code as u8]
}

/// Enter Safe mode: full control with cliff, wheel-drop and charger safeties.
pub fn safe() -> Vec<u8> {
    vec![Opcode::Safe as u8]
}

/// Enter Full mode: full control, safeties off.
pub fn full() -> Vec<u8> {
    vec![Opcode::Full as u8]
}

/// Power down. Returns to Passive mode.
pub fn power() -> Vec<u8> {
    vec![Opcode::Power as u8]
}

/// Start the stock Clean cycle (Passive mode).
pub fn clean() -> Vec<u8> {
    vec![Opcode::Clean as u8]
}

/// Start the stock Spot cycle (Passive mode).
pub fn spot() -> Vec<u8> {
    vec![Opcode::Spot as u8]
}

/// Start the stock Max cycle (Passive mode).
pub fn max() -> Vec<u8> {
    vec![Opcode::Max as u8]
}

/// Drive to the dock using the robot's own behavior (Passive mode).
pub fn seek_dock() -> Vec<u8> {
    vec![Opcode::SeekDock as u8]
}

/// Drive at `velocity_mm_s` (clamped to ±500) along `radius`.
pub fn drive(velocity_mm_s: i16, radius: Radius) -> Vec<u8> {
    let v = be16(velocity_mm_s.clamp(-MAX_VELOCITY_MM_S, MAX_VELOCITY_MM_S) as u16);
    let r = be16(radius.code());
    vec![Opcode::Drive as u8, v[0], v[1], r[0], r[1]]
}

/// Drive each wheel at its own velocity in mm/s (clamped to ±500). Note the order: right, left.
pub fn drive_direct(right_mm_s: i16, left_mm_s: i16) -> Vec<u8> {
    let r = be16(right_mm_s.clamp(-MAX_VELOCITY_MM_S, MAX_VELOCITY_MM_S) as u16);
    let l = be16(left_mm_s.clamp(-MAX_VELOCITY_MM_S, MAX_VELOCITY_MM_S) as u16);
    vec![Opcode::DriveDirect as u8, r[0], r[1], l[0], l[1]]
}

/// Drive each wheel with a raw PWM in ±255.
pub fn drive_pwm(right: i16, left: i16) -> Vec<u8> {
    let r = be16(right.clamp(-255, 255) as u16);
    let l = be16(left.clamp(-255, 255) as u16);
    vec![Opcode::DrivePwm as u8, r[0], r[1], l[0], l[1]]
}

/// Stop both wheels.
pub fn halt() -> Vec<u8> {
    drive_direct(0, 0)
}

/// Switch the cleaning motors on or off.
pub fn motors(m: Motors) -> Vec<u8> {
    vec![Opcode::Motors as u8, m.bits()]
}

/// Set cleaning motor duty cycles: brushes in ±127, vacuum in 0..=127.
pub fn pwm_motors(main_brush: i8, side_brush: i8, vacuum: u8) -> Vec<u8> {
    vec![
        Opcode::PwmMotors as u8,
        main_brush as u8,
        side_brush as u8,
        vacuum.min(127),
    ]
}

/// Request one sensor packet or group. The robot replies with exactly `packet.size()` bytes.
pub fn sensors(packet: Packet) -> Vec<u8> {
    vec![Opcode::Sensors as u8, packet as u8]
}

/// Request several packets at once. The robot replies with their data concatenated, in order.
pub fn query_list(packets: &[Packet]) -> Vec<u8> {
    let mut out = Vec::with_capacity(2 + packets.len());
    out.push(Opcode::QueryList as u8);
    out.push(packets.len() as u8);
    out.extend(packets.iter().map(|p| *p as u8));
    out
}

/// Stream these packets every 15 ms, framed as parsed by [`crate::StreamParser`].
///
/// The spec limits one frame to what 115200 baud carries in 15 ms (about 172 bytes).
pub fn stream(packets: &[Packet]) -> Vec<u8> {
    let mut out = Vec::with_capacity(2 + packets.len());
    out.push(Opcode::Stream as u8);
    out.push(packets.len() as u8);
    out.extend(packets.iter().map(|p| *p as u8));
    out
}

/// Pause the stream without forgetting the packet list.
pub fn pause_stream() -> Vec<u8> {
    vec![Opcode::PauseResumeStream as u8, 0]
}

/// Resume a paused stream.
pub fn resume_stream() -> Vec<u8> {
    vec![Opcode::PauseResumeStream as u8, 1]
}

/// Show up to four ASCII characters on the 7-segment display (600 series with digits only).
pub fn digit_leds_ascii(text: &str) -> Vec<u8> {
    let mut out = vec![Opcode::DigitLedsAscii as u8, b' ', b' ', b' ', b' '];
    for (slot, ch) in out[1..].iter_mut().zip(text.bytes()) {
        *slot = if ch.is_ascii_graphic() || ch == b' ' { ch } else { b' ' };
    }
    out
}

/// Set the day of week (0 = Sunday) and 24-hour time on the robot's clock.
pub fn set_day_time(day: u8, hour: u8, minute: u8) -> Vec<u8> {
    vec![
        Opcode::SetDayTime as u8,
        day.min(6),
        hour.min(23),
        minute.min(59),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mode_commands_are_single_bytes() {
        assert_eq!(start(), [128]);
        assert_eq!(safe(), [131]);
        assert_eq!(full(), [132]);
        assert_eq!(clean(), [135]);
        assert_eq!(seek_dock(), [143]);
        assert_eq!(stop(), [173]);
    }

    #[test]
    fn drive_encodes_big_endian_and_special_radii() {
        // Spec example: 200 mm/s on a 500 mm left arc.
        assert_eq!(drive(200, Radius::Arc(500)), [137, 0x00, 0xC8, 0x01, 0xF4]);
        assert_eq!(drive(-200, Radius::Straight), [137, 0xFF, 0x38, 0x80, 0x00]);
        assert_eq!(drive(100, Radius::SpinClockwise), [137, 0, 100, 0xFF, 0xFF]);
        assert_eq!(drive(100, Radius::SpinCounterClockwise), [137, 0, 100, 0, 1]);
    }

    #[test]
    fn drive_clamps() {
        assert_eq!(drive(900, Radius::Arc(9000)), [137, 0x01, 0xF4, 0x07, 0xD0]);
        assert_eq!(drive_direct(-900, 900), [145, 0xFE, 0x0C, 0x01, 0xF4]);
    }

    #[test]
    fn drive_direct_is_right_then_left() {
        assert_eq!(drive_direct(100, -100), [145, 0, 100, 0xFF, 0x9C]);
        assert_eq!(halt(), [145, 0, 0, 0, 0]);
    }

    #[test]
    fn motor_bits_match_spec() {
        assert_eq!(motors(Motors::OFF), [138, 0]);
        assert_eq!(motors(Motors::ALL), [138, 0b111]);
        let m = Motors {
            main_brush: true,
            main_brush_reverse: true,
            side_brush: true,
            side_brush_clockwise: true,
            vacuum: false,
        };
        assert_eq!(motors(m), [138, 0b11101]);
    }

    #[test]
    fn pwm_motors_clamps_vacuum() {
        assert_eq!(pwm_motors(-127, 64, 200), [144, 0x81, 64, 127]);
    }

    #[test]
    fn sensor_requests() {
        assert_eq!(sensors(Packet::Group100), [142, 100]);
        assert_eq!(
            query_list(&[Packet::LeftEncoderCounts, Packet::RightEncoderCounts]),
            [149, 2, 43, 44]
        );
        assert_eq!(
            stream(&[Packet::BumpsWheelDrops, Packet::ChargingState]),
            [148, 2, 7, 21]
        );
        assert_eq!(pause_stream(), [150, 0]);
        assert_eq!(resume_stream(), [150, 1]);
    }

    #[test]
    fn digits_pad_and_sanitize() {
        assert_eq!(digit_leds_ascii("HI"), [164, b'H', b'I', b' ', b' ']);
        assert_eq!(digit_leds_ascii("ABCDE"), [164, b'A', b'B', b'C', b'D']);
        assert_eq!(digit_leds_ascii("A\nB"), [164, b'A', b' ', b'B', b' ']);
    }
}
