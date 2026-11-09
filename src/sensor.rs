//! Sensor packet ids, sizes, groups and decoding.

use core::fmt;

/// A sensor packet or packet group id, as used by `Sensors`, `Query List` and `Stream`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[repr(u8)]
#[allow(missing_docs)]
pub enum Packet {
    Group0 = 0,
    Group1 = 1,
    Group2 = 2,
    Group3 = 3,
    Group4 = 4,
    Group5 = 5,
    Group6 = 6,
    BumpsWheelDrops = 7,
    Wall = 8,
    CliffLeft = 9,
    CliffFrontLeft = 10,
    CliffFrontRight = 11,
    CliffRight = 12,
    VirtualWall = 13,
    WheelOvercurrents = 14,
    DirtDetect = 15,
    Unused16 = 16,
    IrOpcode = 17,
    Buttons = 18,
    Distance = 19,
    Angle = 20,
    ChargingState = 21,
    Voltage = 22,
    Current = 23,
    Temperature = 24,
    BatteryCharge = 25,
    BatteryCapacity = 26,
    WallSignal = 27,
    CliffLeftSignal = 28,
    CliffFrontLeftSignal = 29,
    CliffFrontRightSignal = 30,
    CliffRightSignal = 31,
    Unused32 = 32,
    Unused33 = 33,
    ChargingSources = 34,
    OiMode = 35,
    SongNumber = 36,
    SongPlaying = 37,
    StreamPackets = 38,
    RequestedVelocity = 39,
    RequestedRadius = 40,
    RequestedRightVelocity = 41,
    RequestedLeftVelocity = 42,
    LeftEncoderCounts = 43,
    RightEncoderCounts = 44,
    LightBumper = 45,
    LightBumpLeftSignal = 46,
    LightBumpFrontLeftSignal = 47,
    LightBumpCenterLeftSignal = 48,
    LightBumpCenterRightSignal = 49,
    LightBumpFrontRightSignal = 50,
    LightBumpRightSignal = 51,
    IrOpcodeLeft = 52,
    IrOpcodeRight = 53,
    LeftMotorCurrent = 54,
    RightMotorCurrent = 55,
    MainBrushCurrent = 56,
    SideBrushCurrent = 57,
    Stasis = 58,
    Group100 = 100,
    Group101 = 101,
    Group106 = 106,
    Group107 = 107,
}

impl Packet {
    /// Look up a packet by its wire id.
    pub fn from_id(id: u8) -> Option<Packet> {
        use Packet::*;
        Some(match id {
            0 => Group0,
            1 => Group1,
            2 => Group2,
            3 => Group3,
            4 => Group4,
            5 => Group5,
            6 => Group6,
            7 => BumpsWheelDrops,
            8 => Wall,
            9 => CliffLeft,
            10 => CliffFrontLeft,
            11 => CliffFrontRight,
            12 => CliffRight,
            13 => VirtualWall,
            14 => WheelOvercurrents,
            15 => DirtDetect,
            16 => Unused16,
            17 => IrOpcode,
            18 => Buttons,
            19 => Distance,
            20 => Angle,
            21 => ChargingState,
            22 => Voltage,
            23 => Current,
            24 => Temperature,
            25 => BatteryCharge,
            26 => BatteryCapacity,
            27 => WallSignal,
            28 => CliffLeftSignal,
            29 => CliffFrontLeftSignal,
            30 => CliffFrontRightSignal,
            31 => CliffRightSignal,
            32 => Unused32,
            33 => Unused33,
            34 => ChargingSources,
            35 => OiMode,
            36 => SongNumber,
            37 => SongPlaying,
            38 => StreamPackets,
            39 => RequestedVelocity,
            40 => RequestedRadius,
            41 => RequestedRightVelocity,
            42 => RequestedLeftVelocity,
            43 => LeftEncoderCounts,
            44 => RightEncoderCounts,
            45 => LightBumper,
            46 => LightBumpLeftSignal,
            47 => LightBumpFrontLeftSignal,
            48 => LightBumpCenterLeftSignal,
            49 => LightBumpCenterRightSignal,
            50 => LightBumpFrontRightSignal,
            51 => LightBumpRightSignal,
            52 => IrOpcodeLeft,
            53 => IrOpcodeRight,
            54 => LeftMotorCurrent,
            55 => RightMotorCurrent,
            56 => MainBrushCurrent,
            57 => SideBrushCurrent,
            58 => Stasis,
            100 => Group100,
            101 => Group101,
            106 => Group106,
            107 => Group107,
            _ => return None,
        })
    }

    /// True for the group ids that expand to several packets.
    pub fn is_group(self) -> bool {
        !self.members().is_empty()
    }

    /// The single packets a group id expands to, in wire order. Empty for a single packet.
    pub fn members(self) -> &'static [Packet] {
        use Packet::*;
        const G1: &[Packet] = &[
            BumpsWheelDrops,
            Wall,
            CliffLeft,
            CliffFrontLeft,
            CliffFrontRight,
            CliffRight,
            VirtualWall,
            WheelOvercurrents,
            DirtDetect,
            Unused16,
        ];
        const G2: &[Packet] = &[IrOpcode, Buttons, Distance, Angle];
        const G3: &[Packet] = &[
            ChargingState,
            Voltage,
            Current,
            Temperature,
            BatteryCharge,
            BatteryCapacity,
        ];
        const G4: &[Packet] = &[
            WallSignal,
            CliffLeftSignal,
            CliffFrontLeftSignal,
            CliffFrontRightSignal,
            CliffRightSignal,
            Unused32,
            Unused33,
            ChargingSources,
        ];
        const G5: &[Packet] = &[
            OiMode,
            SongNumber,
            SongPlaying,
            StreamPackets,
            RequestedVelocity,
            RequestedRadius,
            RequestedRightVelocity,
            RequestedLeftVelocity,
        ];
        const G101: &[Packet] = &[
            LeftEncoderCounts,
            RightEncoderCounts,
            LightBumper,
            LightBumpLeftSignal,
            LightBumpFrontLeftSignal,
            LightBumpCenterLeftSignal,
            LightBumpCenterRightSignal,
            LightBumpFrontRightSignal,
            LightBumpRightSignal,
            IrOpcodeLeft,
            IrOpcodeRight,
            LeftMotorCurrent,
            RightMotorCurrent,
            MainBrushCurrent,
            SideBrushCurrent,
            Stasis,
        ];
        const G106: &[Packet] = &[
            LightBumpLeftSignal,
            LightBumpFrontLeftSignal,
            LightBumpCenterLeftSignal,
            LightBumpCenterRightSignal,
            LightBumpFrontRightSignal,
            LightBumpRightSignal,
        ];
        const G107: &[Packet] = &[
            LeftMotorCurrent,
            RightMotorCurrent,
            MainBrushCurrent,
            SideBrushCurrent,
            Stasis,
        ];
        const G0: &[Packet] = &concat::<20>(G1, G2, G3);
        const G6: &[Packet] = &concat::<36>(G0, G4, G5);
        const G100: &[Packet] = &concat::<52>(G6, G101, &[]);
        match self {
            Group0 => G0,
            Group1 => G1,
            Group2 => G2,
            Group3 => G3,
            Group4 => G4,
            Group5 => G5,
            Group6 => G6,
            Group100 => G100,
            Group101 => G101,
            Group106 => G106,
            Group107 => G107,
            _ => &[],
        }
    }

    /// Number of data bytes the robot sends for this id.
    pub fn size(self) -> usize {
        use Packet::*;
        match self {
            BumpsWheelDrops | Wall | CliffLeft | CliffFrontLeft | CliffFrontRight | CliffRight
            | VirtualWall | WheelOvercurrents | DirtDetect | Unused16 | IrOpcode | Buttons
            | ChargingState | Temperature | Unused32 | ChargingSources | OiMode | SongNumber
            | SongPlaying | StreamPackets | LightBumper | IrOpcodeLeft | IrOpcodeRight
            | Stasis => 1,
            Distance | Angle | Voltage | Current | BatteryCharge | BatteryCapacity | WallSignal
            | CliffLeftSignal | CliffFrontLeftSignal | CliffFrontRightSignal | CliffRightSignal
            | Unused33 | RequestedVelocity | RequestedRadius | RequestedRightVelocity
            | RequestedLeftVelocity | LeftEncoderCounts | RightEncoderCounts
            | LightBumpLeftSignal | LightBumpFrontLeftSignal | LightBumpCenterLeftSignal
            | LightBumpCenterRightSignal | LightBumpFrontRightSignal | LightBumpRightSignal
            | LeftMotorCurrent | RightMotorCurrent | MainBrushCurrent | SideBrushCurrent => 2,
            group => group.members().iter().map(|p| p.size()).sum(),
        }
    }
}

const fn concat<const N: usize>(a: &[Packet], b: &[Packet], c: &[Packet]) -> [Packet; N] {
    let mut out = [Packet::Unused16; N];
    let mut i = 0;
    let mut k = 0;
    while k < a.len() {
        out[i] = a[k];
        i += 1;
        k += 1;
    }
    k = 0;
    while k < b.len() {
        out[i] = b[k];
        i += 1;
        k += 1;
    }
    k = 0;
    while k < c.len() {
        out[i] = c[k];
        i += 1;
        k += 1;
    }
    assert!(i == N, "group length mismatch");
    out
}

/// Packet 21.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum ChargingState {
    NotCharging,
    Reconditioning,
    FullCharging,
    TrickleCharging,
    Waiting,
    Fault,
    Unknown(u8),
}

impl ChargingState {
    fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::NotCharging,
            1 => Self::Reconditioning,
            2 => Self::FullCharging,
            3 => Self::TrickleCharging,
            4 => Self::Waiting,
            5 => Self::Fault,
            other => Self::Unknown(other),
        }
    }

    /// True while current is flowing into the battery (full or trickle).
    pub fn is_charging(self) -> bool {
        matches!(
            self,
            Self::Reconditioning | Self::FullCharging | Self::TrickleCharging
        )
    }
}

/// Packet 35.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum OiMode {
    Off,
    Passive,
    Safe,
    Full,
    Unknown(u8),
}

impl OiMode {
    fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Off,
            1 => Self::Passive,
            2 => Self::Safe,
            3 => Self::Full,
            other => Self::Unknown(other),
        }
    }
}

/// Packet 7 bit fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BumpsWheelDrops {
    /// Right bumper pressed.
    pub bump_right: bool,
    /// Left bumper pressed.
    pub bump_left: bool,
    /// Right wheel dropped.
    pub wheel_drop_right: bool,
    /// Left wheel dropped.
    pub wheel_drop_left: bool,
}

impl BumpsWheelDrops {
    fn from_u8(v: u8) -> Self {
        Self {
            bump_right: v & 1 != 0,
            bump_left: v & 2 != 0,
            wheel_drop_right: v & 4 != 0,
            wheel_drop_left: v & 8 != 0,
        }
    }

    /// Either bumper.
    pub fn any_bump(self) -> bool {
        self.bump_left || self.bump_right
    }

    /// Either wheel.
    pub fn any_drop(self) -> bool {
        self.wheel_drop_left || self.wheel_drop_right
    }
}

/// Packet 34 bit fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ChargingSources {
    /// Charger plugged into the robot.
    pub internal_charger: bool,
    /// Sitting on the Home Base.
    pub home_base: bool,
}

/// Packet 45 bit fields (light bumper, left to right).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub struct LightBumper {
    pub left: bool,
    pub front_left: bool,
    pub center_left: bool,
    pub center_right: bool,
    pub front_right: bool,
    pub right: bool,
}

/// Packet 18 bit fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub struct Buttons {
    pub clean: bool,
    pub spot: bool,
    pub dock: bool,
    pub minute: bool,
    pub hour: bool,
    pub day: bool,
    pub schedule: bool,
    pub clock: bool,
}

/// A decode failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// The byte slice is not `packet.size()` long.
    WrongLength {
        /// The packet being decoded.
        packet: Packet,
        /// Bytes expected.
        expected: usize,
        /// Bytes given.
        got: usize,
    },
    /// The id is not a packet the spec defines.
    UnknownPacket(u8),
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DecodeError::WrongLength {
                packet,
                expected,
                got,
            } => write!(f, "{packet:?} needs {expected} bytes, got {got}"),
            DecodeError::UnknownPacket(id) => write!(f, "unknown sensor packet id {id}"),
        }
    }
}

impl std::error::Error for DecodeError {}

/// The last known value of every sensor. Fields stay `None` until a packet carrying them arrives.
#[derive(Debug, Clone, Default, PartialEq)]
#[allow(missing_docs)]
pub struct Sensors {
    pub bumps_wheel_drops: Option<BumpsWheelDrops>,
    pub wall: Option<bool>,
    pub cliff_left: Option<bool>,
    pub cliff_front_left: Option<bool>,
    pub cliff_front_right: Option<bool>,
    pub cliff_right: Option<bool>,
    pub virtual_wall: Option<bool>,
    pub wheel_overcurrents: Option<u8>,
    pub dirt_detect: Option<u8>,
    pub ir_opcode: Option<u8>,
    pub buttons: Option<Buttons>,
    /// Millimeters travelled since the last request. Unreliable at low speed; prefer encoders.
    pub distance_mm: Option<i16>,
    /// Degrees turned since the last request. Unreliable; prefer encoders.
    pub angle_deg: Option<i16>,
    pub charging_state: Option<ChargingState>,
    pub voltage_mv: Option<u16>,
    /// Positive while charging, negative while discharging.
    pub current_ma: Option<i16>,
    pub temperature_c: Option<i8>,
    pub battery_charge_mah: Option<u16>,
    pub battery_capacity_mah: Option<u16>,
    pub wall_signal: Option<u16>,
    pub cliff_left_signal: Option<u16>,
    pub cliff_front_left_signal: Option<u16>,
    pub cliff_front_right_signal: Option<u16>,
    pub cliff_right_signal: Option<u16>,
    pub charging_sources: Option<ChargingSources>,
    pub oi_mode: Option<OiMode>,
    pub song_number: Option<u8>,
    pub song_playing: Option<bool>,
    pub stream_packets: Option<u8>,
    pub requested_velocity: Option<i16>,
    pub requested_radius: Option<i16>,
    pub requested_right_velocity: Option<i16>,
    pub requested_left_velocity: Option<i16>,
    /// Raw 16-bit wrapping count. Feed to [`crate::odometry::WheelOdometry`].
    pub left_encoder_counts: Option<u16>,
    /// Raw 16-bit wrapping count.
    pub right_encoder_counts: Option<u16>,
    pub light_bumper: Option<LightBumper>,
    pub light_bump_left_signal: Option<u16>,
    pub light_bump_front_left_signal: Option<u16>,
    pub light_bump_center_left_signal: Option<u16>,
    pub light_bump_center_right_signal: Option<u16>,
    pub light_bump_front_right_signal: Option<u16>,
    pub light_bump_right_signal: Option<u16>,
    pub ir_opcode_left: Option<u8>,
    pub ir_opcode_right: Option<u8>,
    pub left_motor_current_ma: Option<i16>,
    pub right_motor_current_ma: Option<i16>,
    pub main_brush_current_ma: Option<i16>,
    pub side_brush_current_ma: Option<i16>,
    /// Bit 0: stasis toggling (moving forward). Bit 1: stasis disabled.
    pub stasis: Option<u8>,
}

fn u16be(b: &[u8]) -> u16 {
    u16::from_be_bytes([b[0], b[1]])
}

fn i16be(b: &[u8]) -> i16 {
    i16::from_be_bytes([b[0], b[1]])
}

impl Sensors {
    /// Apply one packet's bytes, or a group's concatenated bytes, to the state.
    pub fn update(&mut self, packet: Packet, data: &[u8]) -> Result<(), DecodeError> {
        let expected = packet.size();
        if data.len() != expected {
            return Err(DecodeError::WrongLength {
                packet,
                expected,
                got: data.len(),
            });
        }
        if packet.is_group() {
            let mut offset = 0;
            for member in packet.members() {
                let n = member.size();
                self.update(*member, &data[offset..offset + n])?;
                offset += n;
            }
            return Ok(());
        }
        use Packet::*;
        match packet {
            BumpsWheelDrops => self.bumps_wheel_drops = Some(self::BumpsWheelDrops::from_u8(data[0])),
            Wall => self.wall = Some(data[0] != 0),
            CliffLeft => self.cliff_left = Some(data[0] != 0),
            CliffFrontLeft => self.cliff_front_left = Some(data[0] != 0),
            CliffFrontRight => self.cliff_front_right = Some(data[0] != 0),
            CliffRight => self.cliff_right = Some(data[0] != 0),
            VirtualWall => self.virtual_wall = Some(data[0] != 0),
            WheelOvercurrents => self.wheel_overcurrents = Some(data[0]),
            DirtDetect => self.dirt_detect = Some(data[0]),
            Unused16 | Unused32 | Unused33 => {}
            IrOpcode => self.ir_opcode = Some(data[0]),
            Buttons => {
                let v = data[0];
                self.buttons = Some(self::Buttons {
                    clean: v & 1 != 0,
                    spot: v & 2 != 0,
                    dock: v & 4 != 0,
                    minute: v & 8 != 0,
                    hour: v & 16 != 0,
                    day: v & 32 != 0,
                    schedule: v & 64 != 0,
                    clock: v & 128 != 0,
                })
            }
            Distance => self.distance_mm = Some(i16be(data)),
            Angle => self.angle_deg = Some(i16be(data)),
            ChargingState => self.charging_state = Some(self::ChargingState::from_u8(data[0])),
            Voltage => self.voltage_mv = Some(u16be(data)),
            Current => self.current_ma = Some(i16be(data)),
            Temperature => self.temperature_c = Some(data[0] as i8),
            BatteryCharge => self.battery_charge_mah = Some(u16be(data)),
            BatteryCapacity => self.battery_capacity_mah = Some(u16be(data)),
            WallSignal => self.wall_signal = Some(u16be(data)),
            CliffLeftSignal => self.cliff_left_signal = Some(u16be(data)),
            CliffFrontLeftSignal => self.cliff_front_left_signal = Some(u16be(data)),
            CliffFrontRightSignal => self.cliff_front_right_signal = Some(u16be(data)),
            CliffRightSignal => self.cliff_right_signal = Some(u16be(data)),
            ChargingSources => {
                self.charging_sources = Some(self::ChargingSources {
                    internal_charger: data[0] & 1 != 0,
                    home_base: data[0] & 2 != 0,
                })
            }
            OiMode => self.oi_mode = Some(self::OiMode::from_u8(data[0])),
            SongNumber => self.song_number = Some(data[0]),
            SongPlaying => self.song_playing = Some(data[0] != 0),
            StreamPackets => self.stream_packets = Some(data[0]),
            RequestedVelocity => self.requested_velocity = Some(i16be(data)),
            RequestedRadius => self.requested_radius = Some(i16be(data)),
            RequestedRightVelocity => self.requested_right_velocity = Some(i16be(data)),
            RequestedLeftVelocity => self.requested_left_velocity = Some(i16be(data)),
            LeftEncoderCounts => self.left_encoder_counts = Some(u16be(data)),
            RightEncoderCounts => self.right_encoder_counts = Some(u16be(data)),
            LightBumper => {
                let v = data[0];
                self.light_bumper = Some(self::LightBumper {
                    left: v & 1 != 0,
                    front_left: v & 2 != 0,
                    center_left: v & 4 != 0,
                    center_right: v & 8 != 0,
                    front_right: v & 16 != 0,
                    right: v & 32 != 0,
                })
            }
            LightBumpLeftSignal => self.light_bump_left_signal = Some(u16be(data)),
            LightBumpFrontLeftSignal => self.light_bump_front_left_signal = Some(u16be(data)),
            LightBumpCenterLeftSignal => self.light_bump_center_left_signal = Some(u16be(data)),
            LightBumpCenterRightSignal => self.light_bump_center_right_signal = Some(u16be(data)),
            LightBumpFrontRightSignal => self.light_bump_front_right_signal = Some(u16be(data)),
            LightBumpRightSignal => self.light_bump_right_signal = Some(u16be(data)),
            IrOpcodeLeft => self.ir_opcode_left = Some(data[0]),
            IrOpcodeRight => self.ir_opcode_right = Some(data[0]),
            LeftMotorCurrent => self.left_motor_current_ma = Some(i16be(data)),
            RightMotorCurrent => self.right_motor_current_ma = Some(i16be(data)),
            MainBrushCurrent => self.main_brush_current_ma = Some(i16be(data)),
            SideBrushCurrent => self.side_brush_current_ma = Some(i16be(data)),
            Stasis => self.stasis = Some(data[0]),
            Group0 | Group1 | Group2 | Group3 | Group4 | Group5 | Group6 | Group100 | Group101
            | Group106 | Group107 => unreachable!("groups handled above"),
        }
        Ok(())
    }

    /// Decode the reply to `query_list(packets)`: the packets' data concatenated in order.
    pub fn update_list(&mut self, packets: &[Packet], data: &[u8]) -> Result<(), DecodeError> {
        let total: usize = packets.iter().map(|p| p.size()).sum();
        if data.len() != total {
            return Err(DecodeError::WrongLength {
                packet: packets.first().copied().unwrap_or(Packet::Unused16),
                expected: total,
                got: data.len(),
            });
        }
        let mut offset = 0;
        for p in packets {
            let n = p.size();
            self.update(*p, &data[offset..offset + n])?;
            offset += n;
        }
        Ok(())
    }

    /// Any cliff sensor active.
    pub fn any_cliff(&self) -> bool {
        [
            self.cliff_left,
            self.cliff_front_left,
            self.cliff_front_right,
            self.cliff_right,
        ]
        .contains(&Some(true))
    }

    /// Battery fraction 0..1 from charge over capacity, when both are known and capacity is nonzero.
    pub fn battery_fraction(&self) -> Option<f32> {
        match (self.battery_charge_mah, self.battery_capacity_mah) {
            (Some(c), Some(cap)) if cap > 0 => Some((c as f32 / cap as f32).min(1.0)),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn group_sizes_match_spec() {
        assert_eq!(Packet::Group0.size(), 26);
        assert_eq!(Packet::Group1.size(), 10);
        assert_eq!(Packet::Group2.size(), 6);
        assert_eq!(Packet::Group3.size(), 10);
        assert_eq!(Packet::Group4.size(), 14);
        assert_eq!(Packet::Group5.size(), 12);
        assert_eq!(Packet::Group6.size(), 52);
        assert_eq!(Packet::Group100.size(), 80);
        assert_eq!(Packet::Group101.size(), 28);
        assert_eq!(Packet::Group106.size(), 12);
        assert_eq!(Packet::Group107.size(), 9);
    }

    #[test]
    fn every_id_round_trips() {
        for id in 0..=255u8 {
            if let Some(p) = Packet::from_id(id) {
                assert_eq!(p as u8, id);
            }
        }
        assert_eq!(Packet::from_id(59), None);
        assert_eq!(Packet::from_id(99), None);
    }

    #[test]
    fn decodes_single_packets() {
        let mut s = Sensors::default();
        s.update(Packet::BumpsWheelDrops, &[0b1010]).unwrap();
        assert_eq!(
            s.bumps_wheel_drops.unwrap(),
            BumpsWheelDrops {
                bump_right: false,
                bump_left: true,
                wheel_drop_right: false,
                wheel_drop_left: true,
            }
        );
        s.update(Packet::Voltage, &[0x3E, 0x80]).unwrap();
        assert_eq!(s.voltage_mv, Some(16000));
        s.update(Packet::Current, &[0xFF, 0x38]).unwrap();
        assert_eq!(s.current_ma, Some(-200));
        s.update(Packet::Temperature, &[0xF6]).unwrap();
        assert_eq!(s.temperature_c, Some(-10));
        s.update(Packet::ChargingState, &[3]).unwrap();
        assert_eq!(s.charging_state, Some(ChargingState::TrickleCharging));
        assert!(s.charging_state.unwrap().is_charging());
        s.update(Packet::OiMode, &[2]).unwrap();
        assert_eq!(s.oi_mode, Some(OiMode::Safe));
        s.update(Packet::LeftEncoderCounts, &[0xFF, 0xFE]).unwrap();
        assert_eq!(s.left_encoder_counts, Some(65534));
        s.update(Packet::ChargingSources, &[2]).unwrap();
        assert!(s.charging_sources.unwrap().home_base);
    }

    #[test]
    fn decodes_group_3() {
        let mut s = Sensors::default();
        let data = [2u8, 0x3E, 0x80, 0x01, 0xF4, 0x14, 0x07, 0xD0, 0x0B, 0xB8];
        s.update(Packet::Group3, &data).unwrap();
        assert_eq!(s.charging_state, Some(ChargingState::FullCharging));
        assert_eq!(s.voltage_mv, Some(16000));
        assert_eq!(s.current_ma, Some(500));
        assert_eq!(s.temperature_c, Some(20));
        assert_eq!(s.battery_charge_mah, Some(2000));
        assert_eq!(s.battery_capacity_mah, Some(3000));
        assert!((s.battery_fraction().unwrap() - 2.0 / 3.0).abs() < 1e-6);
    }

    #[test]
    fn group_100_covers_everything() {
        let mut s = Sensors::default();
        let mut data = vec![0u8; 80];
        // Encoders are at offsets 52 and 54 in group 100.
        data[52..54].copy_from_slice(&1234u16.to_be_bytes());
        data[54..56].copy_from_slice(&4321u16.to_be_bytes());
        data[79] = 1; // stasis
        s.update(Packet::Group100, &data).unwrap();
        assert_eq!(s.left_encoder_counts, Some(1234));
        assert_eq!(s.right_encoder_counts, Some(4321));
        assert_eq!(s.stasis, Some(1));
        assert_eq!(s.oi_mode, Some(OiMode::Off));
    }

    #[test]
    fn wrong_length_is_an_error() {
        let mut s = Sensors::default();
        assert_eq!(
            s.update(Packet::Voltage, &[1]),
            Err(DecodeError::WrongLength {
                packet: Packet::Voltage,
                expected: 2,
                got: 1
            })
        );
    }

    #[test]
    fn query_list_reply() {
        let mut s = Sensors::default();
        s.update_list(
            &[Packet::OiMode, Packet::LeftEncoderCounts],
            &[3, 0x00, 0x10],
        )
        .unwrap();
        assert_eq!(s.oi_mode, Some(OiMode::Full));
        assert_eq!(s.left_encoder_counts, Some(16));
    }
}
