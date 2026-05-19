//! PSK Reporter spot upload.
//!
//! Sends reception reports to `report.pskreporter.info:4739` as IPFIX
//! (RFC 5101) datagrams over UDP. The wire format mirrors the JTDX / WSJT-X
//! reference implementations byte-for-byte — the same two template
//! descriptors, the same data-record layout, the same modulo-4 Set padding —
//! because PSK Reporter silently drops any datagram it cannot parse, so there
//! is no margin for guessing.
//!
//! Every datagram is laid out exactly as JTDX's `sendReport()` builds it:
//!
//!   header  +  rxInfoDescriptor  +  txInfoDescriptor  +  rxInfoData  +  txInfoData
//!
//! The two template descriptors are sent in *every* datagram, not just the
//! first few. PSK Reporter does not durably cache templates against the
//! observation id across datagrams, so a descriptor-less datagram referencing
//! a template is unparseable and dropped. The descriptors total only ~150
//! bytes; at one datagram per five minutes the overhead is negligible.
//!
//! A single sender template (`0x50E3`, seven fields, the one JTDX and WSJT-X
//! use) carries every spot. The `sNR` field is mandatory in that template, so
//! a spot with no calibrated SNR — an accumulation-only decode — is sent with
//! an SNR of 0, the same conventional "no report" placeholder already shown
//! in the decode table. There is no separate no-SNR template: PSK Reporter
//! only accepts the templates it knows, and a home-made one is rejected.
//!
//! Etiquette enforced here: a station is reported at most once per flush
//! window, flushes are every five minutes, and the timer is anchored to
//! process start (not the wall clock) so monitors do not all report in sync.

use std::net::{ToSocketAddrs, UdpSocket};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// PSK Reporter ingest endpoint (must be the `report.` name since 2018).
const HOST: &str = "report.pskreporter.info:4739";
/// PSK Reporter's IANA enterprise number — tags every proprietary field.
const ENTERPRISE: u32 = 30351;
/// Sender-information template id — the one JTDX and WSJT-X use.
const TX_TEMPLATE_ID: u16 = 0x50E3;
/// Receiver-information template id.
const RX_TEMPLATE_ID: u16 = 0x50E2;
/// Flush interval — one datagram per five minutes (unless full).
const FLUSH: Duration = Duration::from_secs(5 * 60);
/// Software identifier reported as `decodingSoftware`.
const SOFTWARE: &str = "GW4WND FT8 Skimmer";
/// Cap on the pending-spot queue, so a long network outage cannot grow it
/// without bound.
const MAX_QUEUE: usize = 256;

/// One station heard — becomes a single sender-information record.
#[derive(Clone, Debug, PartialEq)]
pub struct Spot {
    /// Transmitting station's callsign (with any prefix/suffix).
    pub call: String,
    /// Transmitting station's Maidenhead locator (4 or 6 char).
    pub grid: String,
    /// RF frequency the station was heard on, Hz.
    pub freq_hz: u32,
    /// Signal-to-noise estimate, dB — `None` when no calibrated SNR exists
    /// (accumulation-only decodes). A `None` spot is sent with an SNR of 0.
    pub snr: Option<i32>,
    /// Mode string, e.g. "FT8".
    pub mode: String,
    /// Unix time the station was heard.
    pub time: u32,
}

// ---- IPFIX byte helpers (everything big-endian / network order) ----------

fn push_u16(b: &mut Vec<u8>, v: u16) {
    b.extend_from_slice(&v.to_be_bytes());
}
fn push_u32(b: &mut Vec<u8>, v: u32) {
    b.extend_from_slice(&v.to_be_bytes());
}
/// Variable-length string: a 1-byte length prefix then UTF-8 bytes (max 254).
fn push_str(b: &mut Vec<u8>, s: &str) {
    let utf = s.as_bytes();
    let n = utf.len().min(254);
    b.push(n as u8);
    b.extend_from_slice(&utf[..n]);
}
/// Pad to the next 4-byte boundary with NUL bytes.
fn pad4(b: &mut Vec<u8>) {
    while b.len() % 4 != 0 {
        b.push(0);
    }
}
/// Finish an IPFIX Set: pad to a 4-byte boundary, then patch its length
/// field (bytes 2..4) to the padded size.
fn finish_set(set: &mut Vec<u8>) {
    pad4(set);
    let len = set.len() as u16;
    set[2..4].copy_from_slice(&len.to_be_bytes());
}

/// Append an enterprise-specific field specifier to a template.
fn ent_field(d: &mut Vec<u8>, ie: u16, len: u16) {
    push_u16(d, 0x8000 + ie);
    push_u16(d, len);
    push_u32(d, ENTERPRISE);
}

/// Sender-information template descriptor (Set ID 2, template
/// `TX_TEMPLATE_ID`).
///
/// Seven fields: senderCallsign, frequency, sNR, mode, senderLocator,
/// informationSource, dateTimeSeconds — the set JTDX and WSJT-X use.
fn tx_descriptor() -> Vec<u8> {
    let mut d = Vec::new();
    push_u16(&mut d, 2); // Template Set ID
    push_u16(&mut d, 0); // length placeholder
    push_u16(&mut d, TX_TEMPLATE_ID);
    push_u16(&mut d, 7); // field count
    ent_field(&mut d, 1, 0xFFFF); // senderCallsign     (variable)
    ent_field(&mut d, 5, 4); // frequency          (u32)
    ent_field(&mut d, 6, 1); // sNR                (i8)
    ent_field(&mut d, 10, 0xFFFF); // mode               (variable)
    ent_field(&mut d, 3, 0xFFFF); // senderLocator      (variable)
    ent_field(&mut d, 11, 1); // informationSource  (u8)
    push_u16(&mut d, 150); // dateTimeSeconds — IETF element, no enterprise
    push_u16(&mut d, 4);
    finish_set(&mut d);
    d
}

/// Receiver-information template descriptor (Options Template Set ID 3).
fn rx_descriptor() -> Vec<u8> {
    let mut d = Vec::new();
    push_u16(&mut d, 3); // Options Template Set ID
    push_u16(&mut d, 0); // length placeholder
    push_u16(&mut d, RX_TEMPLATE_ID);
    push_u16(&mut d, 4); // field count
    push_u16(&mut d, 0); // scope field count
    ent_field(&mut d, 2, 0xFFFF); // receiverCallsign
    ent_field(&mut d, 4, 0xFFFF); // receiverLocator
    ent_field(&mut d, 8, 0xFFFF); // decodingSoftware
    ent_field(&mut d, 9, 0xFFFF); // antennaInformation
    finish_set(&mut d);
    d
}

/// Extract the *sender's* callsign from an FT8 message.
///
/// FT8 messages name two stations as `<recipient> <sender> <report/grid>`,
/// or `CQ <sender> <grid>` / `CQ DX <sender> <grid>` for a call. The sender —
/// the station actually transmitting, and so the one to spot or to track —
/// is the callsign immediately before the trailing report or grid token.
///
/// This is the single source of truth for "who sent this": both PSK Reporter
/// spotting and the skimmer view's per-station track grouping use it, so they
/// can never disagree about a message's identity. Returns `None` for free
/// text or anything without a recognisable sender callsign.
pub fn sender_callsign(message: &str) -> Option<String> {
    let tokens: Vec<&str> = message.split_whitespace().collect();
    if tokens.len() < 2 {
        return None;
    }
    // The last token is a grid, a report (`-07`, `R-12`, `+02`), `RRR`,
    // `RR73` or `73`. The sender is the token before it — skipping a bare
    // `R` in the `Call Call R Grid` form.
    let mut ci = tokens.len() - 2;
    if tokens[ci].eq_ignore_ascii_case("R") && ci > 0 {
        ci -= 1;
    }
    let call = tokens[ci];
    if is_callsign(call) {
        Some(call.to_ascii_uppercase())
    } else {
        None
    }
}

/// Decode an FT8 message into a spottable `(callsign, grid)`.
///
/// PSK Reporter only wants standard messages that carry the sender's
/// callsign *and* their locator — CQ calls and the `Call Call Grid` /
/// `Call Call R Grid` exchange forms. Report exchanges (`-12`, `R-07`,
/// `RRR`, `73`) carry no grid and yield `None`, as do non-standard / free
/// text messages.
pub fn parse_spot(message: &str) -> Option<(String, String)> {
    let tokens: Vec<&str> = message.split_whitespace().collect();
    if tokens.len() < 2 {
        return None;
    }
    // The grid is the final token; bail out if it is not a locator.
    let grid = tokens[tokens.len() - 1];
    if !is_grid(grid) {
        return None;
    }
    // The sender is found by the shared rule, so spotting and the skimmer's
    // track grouping always agree on a message's identity.
    let call = sender_callsign(message)?;
    Some((call, grid.to_ascii_uppercase()))
}

/// True for a 4- or 6-character Maidenhead locator (`IO82`, `IO82KM`).
fn is_grid(s: &str) -> bool {
    let b = s.as_bytes();
    if b.len() != 4 && b.len() != 6 {
        return false;
    }
    let up = |c: u8| c.to_ascii_uppercase();
    let field = |c: u8| (b'A'..=b'R').contains(&up(c));
    let digit = |c: u8| c.is_ascii_digit();
    let sub = |c: u8| (b'A'..=b'X').contains(&up(c));
    field(b[0])
        && field(b[1])
        && digit(b[2])
        && digit(b[3])
        && (b.len() == 4 || (sub(b[4]) && sub(b[5])))
}

/// True for a plausible amateur callsign: alphanumeric plus `/`, at least one
/// letter and one digit. The digit requirement also rejects "CQ" and "DX".
fn is_callsign(s: &str) -> bool {
    if s.len() < 3 || s.len() > 11 {
        return false;
    }
    let mut has_letter = false;
    let mut has_digit = false;
    for c in s.chars() {
        if c.is_ascii_alphabetic() {
            has_letter = true;
        } else if c.is_ascii_digit() {
            has_digit = true;
        } else if c != '/' {
            return false;
        }
    }
    has_letter && has_digit
}

/// PSK Reporter spot uploader. Created on, and driven by, the decoder thread.
pub struct Reporter {
    socket: UdpSocket,
    connected: bool,
    rx_call: String,
    rx_grid: String,
    rx_ant: String,
    observation_id: u32,
    sequence: u32,
    spots: Vec<Spot>,
    last_send: Instant,
}

impl Reporter {
    /// Create a reporter for the given receiving station. Never fails — if
    /// the host cannot be resolved yet, sending is retried on each flush.
    pub fn new(rx_call: &str, rx_grid: &str, rx_ant: &str) -> Self {
        let socket = UdpSocket::bind("0.0.0.0:0")
            .expect("bind UDP socket for PSK Reporter");
        let observation_id = {
            // Constant for this session, reasonably unique across monitors.
            let nanos = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.subsec_nanos())
                .unwrap_or(0);
            nanos ^ (std::process::id().wrapping_mul(2_654_435_761))
        };
        let mut r = Reporter {
            socket,
            connected: false,
            rx_call: rx_call.to_ascii_uppercase(),
            rx_grid: rx_grid.to_string(),
            rx_ant: rx_ant.to_string(),
            observation_id,
            sequence: 0,
            spots: Vec::new(),
            // Anchored to process start, so flushes are not wall-clock-synced.
            last_send: Instant::now(),
        };
        r.try_connect();
        r
    }

    /// Queue a spot. A repeat of a callsign already queued this window
    /// replaces the earlier entry (one report per call per flush). A real
    /// SNR is never lost to a later accumulation decode: if the incoming
    /// spot has no SNR but the queued one does, the SNR is carried forward.
    /// Spots of our own callsign are ignored.
    pub fn add_spot(&mut self, mut spot: Spot) {
        if spot.call.eq_ignore_ascii_case(&self.rx_call) {
            return;
        }
        if let Some(existing) = self.spots.iter_mut().find(|s| s.call == spot.call) {
            if spot.snr.is_none() {
                spot.snr = existing.snr;
            }
            *existing = spot;
            return;
        }
        if self.spots.len() < MAX_QUEUE {
            self.spots.push(spot);
        }
    }

    /// Called once per slot. Sends a datagram when the five-minute window has
    /// elapsed and there is something worth sending.
    pub fn tick(&mut self) {
        if self.last_send.elapsed() < FLUSH {
            return;
        }
        self.last_send = Instant::now();
        self.flush();
    }

    /// Build and send a datagram now, if there is anything to report.
    /// Pending spots are cleared only on a successful send.
    pub fn flush(&mut self) {
        if self.spots.is_empty() {
            return;
        }
        let datagram = self.build_datagram();
        match self.send(&datagram) {
            Ok(()) => {
                let n = self.spots.len();
                self.spots.clear();
                eprintln!("[pskreporter] sent {n} spot(s), {} bytes", datagram.len());
            }
            Err(e) => {
                // Keep the spots queued and retry on the next window.
                eprintln!("[pskreporter] send failed: {e}");
            }
        }
    }

    /// Number of spots currently queued (diagnostic).
    pub fn pending(&self) -> usize {
        self.spots.len()
    }

    fn try_connect(&mut self) {
        match HOST.to_socket_addrs() {
            Ok(mut addrs) => {
                if let Some(addr) = addrs.next() {
                    match self.socket.connect(addr) {
                        Ok(()) => self.connected = true,
                        Err(e) => eprintln!("[pskreporter] connect failed: {e}"),
                    }
                }
            }
            Err(e) => eprintln!("[pskreporter] DNS lookup failed: {e}"),
        }
    }

    fn send(&mut self, datagram: &[u8]) -> std::io::Result<()> {
        if !self.connected {
            self.try_connect();
        }
        if !self.connected {
            return Err(std::io::Error::new(
                std::io::ErrorKind::NotConnected,
                "PSK Reporter host not resolved",
            ));
        }
        self.socket.send(datagram).map(|_| ())
    }

    /// Receiver-information data record (Set `RX_TEMPLATE_ID`).
    fn rx_record(&self) -> Vec<u8> {
        let mut d = Vec::new();
        push_u16(&mut d, RX_TEMPLATE_ID);
        push_u16(&mut d, 0); // length placeholder
        push_str(&mut d, &self.rx_call);
        push_str(&mut d, &self.rx_grid);
        push_str(&mut d, SOFTWARE);
        push_str(&mut d, &self.rx_ant);
        finish_set(&mut d);
        d
    }

    /// Sender-information data record (Set `TX_TEMPLATE_ID`) — one entry per
    /// queued spot. Field order matches `tx_descriptor`. A spot with no
    /// calibrated SNR is sent with an SNR of 0.
    fn tx_record(&self) -> Vec<u8> {
        let mut d = Vec::new();
        push_u16(&mut d, TX_TEMPLATE_ID);
        push_u16(&mut d, 0); // length placeholder
        for s in &self.spots {
            push_str(&mut d, &s.call);
            push_u32(&mut d, s.freq_hz);
            d.push(s.snr.unwrap_or(0).clamp(-128, 127) as i8 as u8);
            push_str(&mut d, &s.mode);
            push_str(&mut d, &s.grid);
            d.push(1); // informationSource = 1 (automatically gathered)
            push_u32(&mut d, s.time);
        }
        finish_set(&mut d);
        d
    }

    /// Assemble a complete IPFIX datagram:
    /// header + rxDescriptor + txDescriptor + rxRecord + txRecord.
    fn build_datagram(&mut self) -> Vec<u8> {
        let mut msg = Vec::with_capacity(512);
        // --- message header (16 bytes) ---
        push_u16(&mut msg, 10); // IPFIX version
        push_u16(&mut msg, 0); // total length  (patched below)
        push_u32(&mut msg, 0); // export time   (patched below)
        self.sequence = self.sequence.wrapping_add(1);
        push_u32(&mut msg, self.sequence);
        push_u32(&mut msg, self.observation_id);

        // Descriptors in every datagram — PSK Reporter does not durably cache
        // templates across datagrams, so a descriptor-less one is dropped.
        msg.extend_from_slice(&rx_descriptor());
        msg.extend_from_slice(&tx_descriptor());
        msg.extend_from_slice(&self.rx_record());
        msg.extend_from_slice(&self.tx_record());

        // Each Set is already 4-byte aligned, so this is a no-op in practice;
        // kept for exactness with the reference implementation.
        pad4(&mut msg);
        let total = msg.len() as u16;
        msg[2..4].copy_from_slice(&total.to_be_bytes());
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs() as u32)
            .unwrap_or(0);
        msg[4..8].copy_from_slice(&now.to_be_bytes());
        msg
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spot(call: &str, snr: Option<i32>) -> Spot {
        Spot {
            call: call.into(),
            grid: "IO82".into(),
            freq_hz: 144_174_966,
            snr,
            mode: "FT8".into(),
            time: 1_700_000_000,
        }
    }

    #[test]
    fn parses_cq_messages() {
        assert_eq!(
            parse_spot("CQ G1HJW JO01"),
            Some(("G1HJW".into(), "JO01".into()))
        );
        assert_eq!(
            parse_spot("CQ DX G0VXM IO93"),
            Some(("G0VXM".into(), "IO93".into()))
        );
        assert_eq!(
            parse_spot("CQ GW4WND IO82KM"),
            Some(("GW4WND".into(), "IO82KM".into()))
        );
    }

    #[test]
    fn parses_exchange_messages() {
        // "to from grid" — the sender is the second callsign.
        assert_eq!(
            parse_spot("G0VXM DJ9ON JO31"),
            Some(("DJ9ON".into(), "JO31".into()))
        );
        // "to from R grid" — the bare R is skipped.
        assert_eq!(
            parse_spot("G0VXM DJ9ON R JO31"),
            Some(("DJ9ON".into(), "JO31".into()))
        );
    }

    #[test]
    fn sender_callsign_identifies_the_transmitting_station() {
        // CQ forms — sender is the calling station.
        assert_eq!(sender_callsign("CQ G1HJW JO01").as_deref(), Some("G1HJW"));
        assert_eq!(
            sender_callsign("CQ DX G0VXM IO93").as_deref(),
            Some("G0VXM")
        );
        // Exchange forms — sender is the second (from) callsign, whatever the
        // trailing report. Across a QSO the two stations alternate as sender,
        // so keying tracks on this correctly yields one track per station
        // rather than one track zig-zagging between their frequencies.
        assert_eq!(
            sender_callsign("G1HJW G0BIX 73").as_deref(),
            Some("G0BIX")
        );
        assert_eq!(
            sender_callsign("G0BIX G1HJW +02").as_deref(),
            Some("G1HJW")
        );
        assert_eq!(
            sender_callsign("G1HJW G0BIX R-01").as_deref(),
            Some("G0BIX")
        );
        // Free text / no sender — no callsign.
        assert_eq!(sender_callsign("TNX FER QSO"), None);
    }

    #[test]
    fn rejects_messages_without_a_grid() {
        assert_eq!(parse_spot("G0VXM DJ9ON -10"), None);
        assert_eq!(parse_spot("G0VXM DJ9ON R-07"), None);
        assert_eq!(parse_spot("G0VXM DJ9ON RRR"), None);
        assert_eq!(parse_spot("G0VXM DJ9ON 73"), None);
        assert_eq!(parse_spot("CQ G1HJW"), None);
        assert_eq!(parse_spot("TNX 73 GL"), None);
    }

    #[test]
    fn grid_validation() {
        assert!(is_grid("IO82"));
        assert!(is_grid("IO82KM"));
        assert!(is_grid("jo31"));
        assert!(!is_grid("IO8")); // too short
        assert!(!is_grid("IO82X")); // odd length
        assert!(!is_grid("ZZ99")); // field letters past R
        assert!(!is_grid("73")); // a report, not a grid
    }

    #[test]
    fn tx_descriptor_is_byte_exact() {
        // The sender template as JTDX/WSJT-X emit it: Set 2, template 0x50E3,
        // 7 fields, 60 bytes total (already 4-byte aligned).
        let d = tx_descriptor();
        let expect: &[u8] = &[
            0x00, 0x02, 0x00, 0x3C, 0x50, 0xE3, 0x00, 0x07, // set hdr
            0x80, 0x01, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F, // senderCallsign
            0x80, 0x05, 0x00, 0x04, 0x00, 0x00, 0x76, 0x8F, // frequency
            0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x76, 0x8F, // sNR
            0x80, 0x0A, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F, // mode
            0x80, 0x03, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F, // senderLocator
            0x80, 0x0B, 0x00, 0x01, 0x00, 0x00, 0x76, 0x8F, // informationSource
            0x00, 0x96, 0x00, 0x04, // dateTimeSeconds (IE 150)
        ];
        assert_eq!(d, expect);
    }

    #[test]
    fn rx_descriptor_is_byte_exact() {
        // The receiver template: Options Set 3, template 0x50E2, 4 fields,
        // 42 bytes padded to 44.
        let d = rx_descriptor();
        let expect: &[u8] = &[
            0x00, 0x03, 0x00, 0x2C, 0x50, 0xE2, 0x00, 0x04, 0x00, 0x00, // set hdr + scope
            0x80, 0x02, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F, // receiverCallsign
            0x80, 0x04, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F, // receiverLocator
            0x80, 0x08, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F, // decodingSoftware
            0x80, 0x09, 0xFF, 0xFF, 0x00, 0x00, 0x76, 0x8F, // antennaInformation
            0x00, 0x00, // padding to 44
        ];
        assert_eq!(d, expect);
    }

    #[test]
    fn datagram_layout_and_alignment() {
        let mut r = Reporter::new("GW4WND", "IO82KM", "");
        r.add_spot(spot("G1HJW", Some(-11)));
        let dg = r.build_datagram();
        // IPFIX version 10.
        assert_eq!(&dg[0..2], &[0x00, 0x0A]);
        // Length field equals the real length.
        let len = u16::from_be_bytes([dg[2], dg[3]]) as usize;
        assert_eq!(len, dg.len());
        // Whole datagram is 4-byte aligned.
        assert_eq!(dg.len() % 4, 0);
        // Both descriptors and both data records are present, in JTDX order:
        // header(16) then Set 3 (rx desc), Set 2 (tx desc), 0x50E2, 0x50E3.
        assert_eq!(&dg[16..18], &[0x00, 0x03]); // rx descriptor Set
        let has = |id: [u8; 2]| dg.windows(2).any(|w| w == id);
        assert!(has([0x00, 0x02])); // tx descriptor Set
        assert!(has([0x50, 0xE2])); // rx data record
        assert!(has([0x50, 0xE3])); // tx data record
        // Descriptors are sent on every datagram, not just the first.
        let dg2 = r.build_datagram();
        assert_eq!(&dg2[16..18], &[0x00, 0x03]);
        // Observation id stable; sequence increments.
        assert_eq!(&dg[12..16], &dg2[12..16]);
        assert_ne!(&dg[8..12], &dg2[8..12]);
    }

    #[test]
    fn no_snr_spot_encodes_as_zero() {
        let mut r = Reporter::new("GW4WND", "IO82KM", "");
        r.add_spot(spot("DJ9ON", None));
        let rec = r.tx_record();
        // Set header (4) + call: 1-byte len + "DJ9ON" (5) = offset 10,
        // then frequency u32 (4) -> SNR byte at offset 14.
        assert_eq!(rec[14], 0);
    }

    #[test]
    fn own_callsign_is_not_spotted() {
        let mut r = Reporter::new("GW4WND", "IO82KM", "");
        r.add_spot(spot("GW4WND", Some(0)));
        assert_eq!(r.pending(), 0);
    }

    #[test]
    fn repeat_callsign_replaces_not_duplicates() {
        let mut r = Reporter::new("GW4WND", "IO82KM", "");
        for snr in [-15, -8, -3] {
            r.add_spot(spot("G1HJW", Some(snr)));
        }
        assert_eq!(r.pending(), 1);
    }

    #[test]
    fn accumulation_decode_does_not_erase_a_real_snr() {
        let mut r = Reporter::new("GW4WND", "IO82KM", "");
        r.add_spot(spot("G1HJW", Some(-9))); // heard single-shot first
        r.add_spot(spot("G1HJW", None)); // later accumulation decode
        assert_eq!(r.pending(), 1);
        assert_eq!(r.spots[0].snr, Some(-9));
    }
}
