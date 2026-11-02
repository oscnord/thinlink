//! How the bridge reaches the autopilot: a TCP port on loopback (SITL) or a serial port (a flight
//! controller over USB or a UART). Addresses are spelled as `MAVLink` tools spell them:
//! `tcpout:127.0.0.1:5760` and `serial:/dev/ttyACM0:921600`.

use std::io::{self, Read, Write};
use std::net::{Shutdown, SocketAddr, TcpStream};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use serde::Deserialize;
use serialport::SerialPort;

const MIN_BAUD: u32 = 9600;
const MAX_BAUD: u32 = 3_000_000;

#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(try_from = "String")]
pub enum Autopilot {
    /// `tcpout:<ip>:<port>`.
    Tcp(SocketAddr),
    /// `serial:<device>:<baud>`.
    Serial { device: String, baud: u32 },
}

impl TryFrom<String> for Autopilot {
    type Error = String;

    fn try_from(text: String) -> Result<Self, String> {
        let invalid = |why: &str| format!("`{text}` is not an autopilot address: {why}");
        let (kind, rest) = text
            .split_once(':')
            .ok_or_else(|| invalid("expected tcpout:<ip>:<port> or serial:<device>:<baud>"))?;
        match kind {
            "tcpout" => rest
                .parse()
                .map(Self::Tcp)
                .map_err(|_| invalid("the TCP address must be <ip>:<port>")),
            "serial" => {
                let (device, baud) = rest
                    .rsplit_once(':')
                    .ok_or_else(|| invalid("expected serial:<device>:<baud>"))?;
                let baud = baud
                    .parse::<u32>()
                    .ok()
                    .filter(|baud| (MIN_BAUD..=MAX_BAUD).contains(baud))
                    .ok_or_else(|| invalid("the baud rate must be 9600 to 3000000"))?;
                if !device.starts_with("/dev/") {
                    return Err(invalid("the serial device must be a path under /dev"));
                }
                Ok(Self::Serial {
                    device: device.to_owned(),
                    baud,
                })
            }
            _ => Err(invalid("only tcpout and serial are supported")),
        }
    }
}

impl Autopilot {
    /// Opens the connection with `poll` as its read timeout and `write_timeout` as the write's (a
    /// serial port's clone writes with `SERIAL_WRITE_TIMEOUT` instead).
    pub(crate) fn connect(
        &self,
        connect_timeout: Duration,
        poll: Duration,
        write_timeout: Duration,
    ) -> io::Result<Box<dyn Wire>> {
        match self {
            Self::Tcp(addr) => {
                let stream = TcpStream::connect_timeout(addr, connect_timeout)?;
                stream.set_read_timeout(Some(poll))?;
                stream.set_write_timeout(Some(write_timeout))?;
                stream.set_nodelay(true)?;
                Ok(Box::new(stream))
            }
            Self::Serial { device, baud } => {
                let port = serialport::new(device, *baud).timeout(poll).open()?;
                Ok(Box::new(SerialWire {
                    port,
                    closed: Arc::default(),
                }))
            }
        }
    }
}

/// One open connection, cloned for the thread that writes to it.
pub(crate) trait Wire: Read + Write + Send {
    fn try_clone(&self) -> io::Result<Box<dyn Wire>>;

    /// Ends the connection for every clone: the reader fails at its next read.
    fn shutdown(&self);
}

impl Wire for TcpStream {
    fn try_clone(&self) -> io::Result<Box<dyn Wire>> {
        Self::try_clone(self).map(|clone| Box::new(clone) as Box<dyn Wire>)
    }

    fn shutdown(&self) {
        let _ = Self::shutdown(self, Shutdown::Both);
    }
}

/// How long a write to a serial port may wait for room. A port's timeout covers reads and writes
/// and each clone keeps its own, so the writer's clone gets this and the reader keeps its short
/// poll. A `MAVLink` v2 frame is at most 280 bytes, 2800 bits with start and stop bits: 3 ms at 921600
/// baud and 24 ms at 115200. 100 ms is over 30 frames at the first and four at the second, and a
/// stalled port is still given up quickly, since the writer holds the lock the control endpoint
/// writes under.
const SERIAL_WRITE_TIMEOUT: Duration = Duration::from_millis(100);

/// A serial port has no shutdown: the clones share a flag the reads check.
struct SerialWire {
    port: Box<dyn SerialPort>,
    closed: Arc<AtomicBool>,
}

impl Read for SerialWire {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if self.closed.load(Ordering::Relaxed) {
            return Err(io::ErrorKind::BrokenPipe.into());
        }
        self.port.read(buf)
    }
}

impl Write for SerialWire {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.port.write(buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.port.flush()
    }
}

impl Wire for SerialWire {
    fn try_clone(&self) -> io::Result<Box<dyn Wire>> {
        let mut port = self.port.try_clone()?;
        port.set_timeout(SERIAL_WRITE_TIMEOUT)?;
        Ok(Box::new(Self {
            port,
            closed: Arc::clone(&self.closed),
        }))
    }

    fn shutdown(&self) {
        self.closed.store(true, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn autopilot_addresses_are_checked() {
        let tcp = |addr: &str| Some(Autopilot::Tcp(addr.parse().unwrap()));
        let serial = |device: &str, baud| {
            Some(Autopilot::Serial {
                device: device.to_owned(),
                baud,
            })
        };
        let cases = [
            ("tcpout:127.0.0.1:5760", tcp("127.0.0.1:5760")),
            (
                "serial:/dev/ttyACM0:115200",
                serial("/dev/ttyACM0", 115_200),
            ),
            (
                "serial:/dev/serial0:921600",
                serial("/dev/serial0", 921_600),
            ),
            ("serial:/dev/ttyACM0", None),
            ("serial:/dev/ttyACM0:fast", None),
            ("serial:/dev/ttyACM0:300", None),
            ("serial:ttyACM0:115200", None),
            ("serial:/tmp/pipe:115200", None),
            ("tcpout:localhost:5760", None),
            ("tcpin:127.0.0.1:5760", None),
            ("udpout:127.0.0.1:14550", None),
            ("127.0.0.1:5760", None),
        ];
        for (text, expected) in cases {
            let parsed = Autopilot::try_from(text.to_owned()).ok();
            assert_eq!(parsed, expected, "{text}");
        }
    }
}
