//! Framing for the `Stream` command (opcode 148).
//!
//! Every 15 ms the robot sends `[19][N][id][data...][id][data...]...[checksum]` where `N` is the
//! number of bytes between it and the checksum, and the sum of all bytes from the header through
//! the checksum is zero modulo 256.

use crate::sensor::{DecodeError, Packet, Sensors};

/// Stream frame header byte.
pub const HEADER: u8 = 19;

/// One parsed stream frame: the packets it carried, in order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    /// `(packet, data)` pairs in wire order.
    pub packets: Vec<(Packet, Vec<u8>)>,
}

impl Frame {
    /// Apply every packet in the frame to `sensors`.
    pub fn apply_to(&self, sensors: &mut Sensors) -> Result<(), DecodeError> {
        for (p, data) in &self.packets {
            sensors.update(*p, data)?;
        }
        Ok(())
    }
}

/// Why a frame was dropped. The parser resynchronizes on its own; these are for logging.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamError {
    /// Checksum mismatch; the frame was discarded.
    BadChecksum,
    /// The payload held an id that is not a sensor packet.
    UnknownPacket(u8),
    /// The payload ended in the middle of a packet.
    Truncated,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Header,
    Length,
    Payload,
    Checksum,
}

/// Incremental parser. Push bytes as they arrive; collect frames and errors.
#[derive(Debug, Clone)]
pub struct StreamParser {
    state: State,
    expected: usize,
    payload: Vec<u8>,
    /// Frames and errors in the order they were seen since the last `drain`.
    events: Vec<Result<Frame, StreamError>>,
}

impl Default for StreamParser {
    fn default() -> Self {
        Self::new()
    }
}

impl StreamParser {
    /// An empty parser waiting for a header byte.
    pub fn new() -> Self {
        Self {
            state: State::Header,
            expected: 0,
            payload: Vec::with_capacity(256),
            events: Vec::new(),
        }
    }

    /// Feed bytes. Call [`StreamParser::drain`] afterwards for the results.
    pub fn push(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.push_byte(b);
        }
    }

    /// Take the frames and errors parsed so far.
    pub fn drain(&mut self) -> Vec<Result<Frame, StreamError>> {
        std::mem::take(&mut self.events)
    }

    /// Convenience: push bytes and return the results in one call.
    pub fn parse(&mut self, bytes: &[u8]) -> Vec<Result<Frame, StreamError>> {
        self.push(bytes);
        self.drain()
    }

    fn push_byte(&mut self, b: u8) {
        match self.state {
            State::Header => {
                if b == HEADER {
                    self.state = State::Length;
                }
            }
            State::Length => {
                self.expected = b as usize;
                self.payload.clear();
                self.state = if b == 0 {
                    State::Checksum
                } else {
                    State::Payload
                };
            }
            State::Payload => {
                self.payload.push(b);
                if self.payload.len() == self.expected {
                    self.state = State::Checksum;
                }
            }
            State::Checksum => {
                self.state = State::Header;
                let sum = HEADER
                    .wrapping_add(self.expected as u8)
                    .wrapping_add(self.payload.iter().fold(0u8, |a, x| a.wrapping_add(*x)))
                    .wrapping_add(b);
                if sum != 0 {
                    self.events.push(Err(StreamError::BadChecksum));
                    return;
                }
                self.events.push(split_payload(&self.payload));
            }
        }
    }
}

fn split_payload(payload: &[u8]) -> Result<Frame, StreamError> {
    let mut packets = Vec::new();
    let mut i = 0;
    while i < payload.len() {
        let id = payload[i];
        let packet = Packet::from_id(id).ok_or(StreamError::UnknownPacket(id))?;
        let n = packet.size();
        let start = i + 1;
        let end = start + n;
        if end > payload.len() {
            return Err(StreamError::Truncated);
        }
        packets.push((packet, payload[start..end].to_vec()));
        i = end;
    }
    Ok(Frame { packets })
}

/// Build a valid frame, for tests and simulators.
pub fn encode_frame(packets: &[(Packet, &[u8])]) -> Vec<u8> {
    let mut out = vec![HEADER, 0];
    for (p, data) in packets {
        out.push(*p as u8);
        out.extend_from_slice(data);
    }
    out[1] = (out.len() - 2) as u8;
    let sum = out.iter().fold(0u8, |a, x| a.wrapping_add(*x));
    out.push(0u8.wrapping_sub(sum));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The example frame from the spec: packets 29 (2 bytes) and 13 (1 byte).
    const SPEC_FRAME: [u8; 8] = [19, 5, 29, 2, 25, 13, 0, 163];

    #[test]
    fn parses_the_spec_example() {
        let mut p = StreamParser::new();
        let frames = p.parse(&SPEC_FRAME);
        assert_eq!(
            frames,
            vec![Ok(Frame {
                packets: vec![
                    (Packet::CliffFrontLeftSignal, vec![2, 25]),
                    (Packet::VirtualWall, vec![0]),
                ],
            })]
        );
    }

    #[test]
    fn encode_matches_spec_example() {
        let f = encode_frame(&[
            (Packet::CliffFrontLeftSignal, &[2, 25]),
            (Packet::VirtualWall, &[0]),
        ]);
        assert_eq!(f, SPEC_FRAME);
    }

    #[test]
    fn byte_at_a_time_and_split_across_pushes() {
        let mut p = StreamParser::new();
        for b in &SPEC_FRAME[..5] {
            p.push(&[*b]);
        }
        assert!(p.drain().is_empty());
        let frames = p.parse(&SPEC_FRAME[5..]);
        assert_eq!(frames.len(), 1);
        assert!(frames[0].is_ok());
    }

    #[test]
    fn resyncs_after_garbage_and_bad_checksum() {
        let mut p = StreamParser::new();
        let mut bad = SPEC_FRAME.to_vec();
        bad[7] ^= 1;
        let mut input = vec![0xAA, 0x55];
        input.extend_from_slice(&bad);
        input.extend_from_slice(&SPEC_FRAME);
        let events = p.parse(&input);
        assert_eq!(events.len(), 2);
        assert_eq!(events[0], Err(StreamError::BadChecksum));
        assert!(events[1].is_ok());
    }

    #[test]
    fn unknown_id_is_reported_not_fatal() {
        let mut p = StreamParser::new();
        let mut frame = vec![19, 2, 99, 0];
        let sum = frame.iter().fold(0u8, |a, x| a.wrapping_add(*x));
        frame.push(0u8.wrapping_sub(sum));
        frame.extend_from_slice(&SPEC_FRAME);
        let events = p.parse(&frame);
        assert_eq!(events[0], Err(StreamError::UnknownPacket(99)));
        assert!(events[1].is_ok());
    }

    #[test]
    fn frame_updates_sensors() {
        let f = encode_frame(&[
            (Packet::LeftEncoderCounts, &[0x01, 0x00]),
            (Packet::RightEncoderCounts, &[0x02, 0x00]),
            (Packet::BumpsWheelDrops, &[1]),
        ]);
        let mut p = StreamParser::new();
        let mut s = Sensors::default();
        for ev in p.parse(&f) {
            ev.unwrap().apply_to(&mut s).unwrap();
        }
        assert_eq!(s.left_encoder_counts, Some(256));
        assert_eq!(s.right_encoder_counts, Some(512));
        assert!(s.bumps_wheel_drops.unwrap().bump_right);
    }

    #[test]
    fn empty_frame_is_valid() {
        let mut p = StreamParser::new();
        let f = encode_frame(&[]);
        assert_eq!(f, [19, 0, 237]);
        assert_eq!(p.parse(&f), vec![Ok(Frame { packets: vec![] })]);
    }
}
