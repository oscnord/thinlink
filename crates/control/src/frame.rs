//! CRC-valid `MAVLink` 1 and 2 frames of the `ardupilotmega` dialect (which includes `common`),
//! read with the `mavlink` crate and kept as received so they can be forwarded unchanged. Bytes
//! that do not form such a frame (noise, truncation, a bad CRC, a message id the dialect does not
//! define) are skipped.

use std::io::{self, Read};

use mavlink::dialects::ardupilotmega::MavMessage;
use mavlink::error::{MessageReadError, ParserError};
use mavlink::{MAVLinkMessageRaw, MavlinkReader, Message};

/// Receive buffer for one datagram. `MAVLink` over UDP carries one or a few frames (at most 280
/// bytes each) per datagram; anything longer is truncated and its cut frame skipped.
pub const MAX_DATAGRAM: usize = 2048;

pub struct Frame(MAVLinkMessageRaw);

impl Frame {
    pub fn message_id(&self) -> u32 {
        self.0.message_id()
    }

    pub fn system_id(&self) -> u32 {
        self.0.system_id()
    }

    pub fn component_id(&self) -> u8 {
        self.0.component_id()
    }

    pub fn is_v2(&self) -> bool {
        matches!(self.0, MAVLinkMessageRaw::V2(_))
    }

    /// The frame exactly as received.
    pub fn bytes(&self) -> &[u8] {
        match &self.0 {
            MAVLinkMessageRaw::V1(raw) => raw.raw_bytes(),
            MAVLinkMessageRaw::V2(raw) => raw.raw_bytes(),
        }
    }

    /// Bytes on the wire; at most 280.
    pub fn size(&self) -> u32 {
        u32::try_from(self.bytes().len()).unwrap_or(u32::MAX)
    }

    /// The decoded message. Fails when a field holds a value the dialect does not define, such as
    /// an unknown command.
    pub fn decode(&self) -> Result<MavMessage, ParserError> {
        MavMessage::parse(self.0.version(), self.0.message_id(), self.0.payload())
    }
}

/// Frames from a byte stream such as SITL's TCP port.
pub struct FrameReader<R>(MavlinkReader<R>);

impl<R: Read> FrameReader<R> {
    pub fn new(source: R) -> Self {
        Self(MavlinkReader::new(source))
    }

    /// The next frame. Any I/O error, end of input included, ends the stream.
    pub fn next_frame(&mut self) -> io::Result<Frame> {
        match self.0.read_any_raw_message::<MavMessage>() {
            Ok(raw) => Ok(Frame(raw)),
            Err(MessageReadError::Io(e)) => Err(e),
            Err(MessageReadError::Parse(e)) => Err(io::Error::other(e.to_string())),
        }
    }

    pub fn source_mut(&mut self) -> &mut R {
        self.0.get_mut()
    }
}

/// The frames of one datagram, in order.
pub fn datagram_frames(datagram: &[u8]) -> impl Iterator<Item = Frame> + '_ {
    let mut reader = FrameReader(MavlinkReader::with_capacity(datagram.len(), datagram));
    std::iter::from_fn(move || reader.next_frame().ok())
}

/// `message` as a frame from system 1, component 1.
#[cfg(test)]
pub(crate) fn encoded(message: &MavMessage) -> Frame {
    let header = mavlink::MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 0,
    };
    let mut bytes = Vec::new();
    mavlink::write_v2_msg(&mut bytes, header, message).unwrap();
    datagram_frames(&bytes).next().unwrap()
}

#[cfg(test)]
mod tests {
    use mavlink::MavHeader;
    use mavlink::dialects::ardupilotmega::{COMMAND_LONG_DATA, HEARTBEAT_DATA, MavCmd};

    use super::*;

    fn encode(message: &MavMessage) -> Vec<u8> {
        let header = MavHeader {
            system_id: 255,
            component_id: 190,
            sequence: 7,
        };
        let mut bytes = Vec::new();
        mavlink::write_v2_msg(&mut bytes, header, message).unwrap();
        bytes
    }

    #[test]
    fn hostile_datagrams_yield_only_whole_valid_frames() {
        let heartbeat = encode(&MavMessage::HEARTBEAT(HEARTBEAT_DATA::default()));
        let command = encode(&MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            command: MavCmd::MAV_CMD_REQUEST_MESSAGE,
            ..COMMAND_LONG_DATA::default()
        }));
        let mut corrupt = command.clone();
        corrupt[12] ^= 0x01;
        let mut unknown = heartbeat.clone();
        // No dialect defines message id 0xFFFFFF, so there is no CRC extra to check it with.
        unknown[7..10].copy_from_slice(&0x00FF_FFFF_u32.to_le_bytes()[..3]);
        let cases: &[(&str, Vec<u8>, &[u32])] = &[
            (
                "two frames",
                [heartbeat.clone(), command.clone()].concat(),
                &[0, 76],
            ),
            (
                "noise around a frame",
                [&[0xFD, 9, 0][..], &command, &[0xFE, 1]].concat(),
                &[76],
            ),
            ("truncated", command[..command.len() - 1].to_vec(), &[]),
            (
                "bad crc, then a frame",
                [corrupt, heartbeat.clone()].concat(),
                &[0],
            ),
            ("unknown message id", unknown, &[]),
            ("empty", Vec::new(), &[]),
            ("all 0xFD", vec![0xFD; MAX_DATAGRAM], &[]),
        ];
        for (name, datagram, ids) in cases {
            let frames: Vec<Frame> = datagram_frames(datagram).collect();
            let found: Vec<u32> = frames.iter().map(Frame::message_id).collect();
            assert_eq!(found, *ids, "{name}");
            for frame in &frames {
                assert!(
                    datagram
                        .windows(frame.bytes().len())
                        .any(|w| w == frame.bytes())
                );
            }
        }
    }
}
