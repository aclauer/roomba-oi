# roomba-oi

The iRobot Roomba Open Interface (OI) as a pure Rust codec: the 600-series spec, which is
the Create 2 spec. No I/O. You own the serial port (115200 8N1, 0-5 V TTL on the 7-pin
mini-DIN); this crate turns intentions into bytes and bytes into sensor values.

- `cmd`: every command as a byte vector (`cmd::drive_direct(right, left)`, `cmd::stream(&[...])`).
- `sensor`: packet ids, sizes and group membership; `Sensors` decodes single packets and groups.
- `stream`: a resynchronizing parser for the `Stream` (opcode 148) framing with checksum.
- `odometry`: wheel-encoder differential-drive integration with 16-bit wrap handling.

Written for the dimos Roomba module of the crumbo robot vacuum, kept separate so it is useful
to anyone driving a Roomba from Rust. Reference: iRobot Create 2 / Roomba 600 Open Interface
Specification.

## Waking and modes

- The robot sleeps about five minutes after entering Passive mode without charging. Pulse the
  BRC pin low for 50-500 ms to wake it; the crate cannot do that for you.
- `cmd::start()` enters Passive, `cmd::safe()` and `cmd::full()` take control. Cleaning and
  docking commands (`clean`, `seek_dock`) put the robot back into Passive.
- Send one command per `cmd::*` call and wait at least 20 ms before the next one when changing
  modes, per the spec.
