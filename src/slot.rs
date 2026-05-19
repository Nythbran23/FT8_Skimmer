//! UTC-aligned 15-second slot framing.
//!
//! FT8 transmits in rigid 15 s T/R periods aligned to UTC. `SlotAssembler`
//! consumes a continuous stream of 12 kHz samples and emits exactly one
//! `Slot` of `SLOT_SAMPLES` samples per period boundary, tagged with the UTC
//! time of that boundary.
//!
//! Timing model: the stream start time is sampled once (`SlotAssembler::new`)
//! and sample index N is taken to be `start + N / SAMPLE_RATE` seconds. Codec
//! vs system-clock drift over a single 15 s slot is negligible for a monitor;
//! a production tool would re-discipline against decoded DT.

use chrono::{DateTime, Duration as ChronoDuration, TimeZone, Utc};

/// Sample rate the decoder and slot framing operate at.
pub const SAMPLE_RATE: usize = 12_000;
/// Length of one FT8 T/R period.
pub const SLOT_SECS: i64 = 15;
/// Samples in one slot (180 000).
pub const SLOT_SAMPLES: usize = SAMPLE_RATE * SLOT_SECS as usize;

/// One complete 15 s slot of 12 kHz audio.
#[derive(Clone)]
pub struct Slot {
    /// UTC time of the slot boundary (always a multiple of 15 s).
    pub utc: DateTime<Utc>,
    /// Exactly `SLOT_SAMPLES` mono samples at `SAMPLE_RATE`.
    pub samples: Vec<f32>,
}

/// Assembles a continuous 12 kHz sample stream into UTC-aligned slots.
pub struct SlotAssembler {
    buf: Vec<f32>,
    /// Global sample index of `buf[0]`.
    buf_start: u64,
    /// Global sample index where the next slot begins.
    next_slot_start: u64,
    /// UTC time of the next slot boundary.
    next_slot_utc: DateTime<Utc>,
}

impl SlotAssembler {
    /// Create an assembler whose stream begins at `stream_start`.
    pub fn new(stream_start: DateTime<Utc>) -> Self {
        let ts = stream_start.timestamp();
        // First 15 s boundary strictly after the stream start second.
        let next = (ts.div_euclid(SLOT_SECS) + 1) * SLOT_SECS;
        let next_slot_utc = Utc.timestamp_opt(next, 0).single().expect("valid ts");
        let ms = (next_slot_utc - stream_start).num_milliseconds();
        let next_slot_start = (ms as f64 * SAMPLE_RATE as f64 / 1000.0).round() as u64;
        Self {
            buf: Vec::new(),
            buf_start: 0,
            next_slot_start,
            next_slot_utc,
        }
    }

    /// UTC boundary of the slot currently being filled.
    pub fn next_slot_utc(&self) -> DateTime<Utc> {
        self.next_slot_utc
    }

    /// Feed resampled 12 kHz samples; returns any slots that completed.
    pub fn push(&mut self, samples: &[f32]) -> Vec<Slot> {
        self.buf.extend_from_slice(samples);
        let mut out = Vec::new();
        loop {
            let total = self.buf_start + self.buf.len() as u64;
            if total < self.next_slot_start + SLOT_SAMPLES as u64 {
                break;
            }
            let local = (self.next_slot_start - self.buf_start) as usize;
            let slot = Slot {
                utc: self.next_slot_utc,
                samples: self.buf[local..local + SLOT_SAMPLES].to_vec(),
            };
            out.push(slot);
            self.next_slot_start += SLOT_SAMPLES as u64;
            self.next_slot_utc += ChronoDuration::seconds(SLOT_SECS);
        }
        // Drop consumed history to bound memory.
        let keep_from =
            ((self.next_slot_start - self.buf_start) as usize).min(self.buf.len());
        if keep_from > 0 {
            self.buf.drain(..keep_from);
            self.buf_start += keep_from as u64;
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slots_align_to_utc_boundaries() {
        // 1_700_000_003 mod 15 == 8, so the first boundary is +7 s away.
        let start = Utc.timestamp_opt(1_700_000_003, 0).single().unwrap();
        let mut asm = SlotAssembler::new(start);

        let first_boundary = Utc.timestamp_opt(1_700_000_010, 0).single().unwrap();
        assert_eq!(asm.next_slot_utc(), first_boundary);

        // 7 s of lead-in (84 000 samples) — not enough for a slot yet.
        assert!(asm.push(&vec![0.0; 7 * SAMPLE_RATE]).is_empty());

        // One full slot worth -> exactly one slot emitted.
        let slots = asm.push(&vec![0.1; SLOT_SAMPLES]);
        assert_eq!(slots.len(), 1);
        assert_eq!(slots[0].utc, first_boundary);
        assert_eq!(slots[0].samples.len(), SLOT_SAMPLES);

        // Another slot's worth -> next boundary, 15 s later.
        let slots = asm.push(&vec![0.2; SLOT_SAMPLES]);
        assert_eq!(slots.len(), 1);
        assert_eq!(
            slots[0].utc,
            Utc.timestamp_opt(1_700_000_025, 0).single().unwrap()
        );
    }

    #[test]
    fn multiple_slots_emit_in_one_push() {
        // 1_700_000_010 is exactly on a 15 s boundary, so the first emitted
        // slot is the *next* boundary, +15 s away.
        let start = Utc.timestamp_opt(1_700_000_010, 0).single().unwrap();
        let mut asm = SlotAssembler::new(start);
        // Push the 15 s lead-in plus three slots in one call.
        let big = vec![0.0; 15 * SAMPLE_RATE + 3 * SLOT_SAMPLES];
        let slots = asm.push(&big);
        assert_eq!(slots.len(), 3);
        for (i, s) in slots.iter().enumerate() {
            let expect =
                Utc.timestamp_opt(1_700_000_025 + i as i64 * 15, 0).single().unwrap();
            assert_eq!(s.utc, expect);
            assert_eq!(s.samples.len(), SLOT_SAMPLES);
        }
    }

    #[test]
    fn arbitrary_chunking_is_stable() {
        // Feeding the same total in odd-sized chunks must give identical slots.
        let start = Utc.timestamp_opt(1_700_000_010, 0).single().unwrap();
        let mut asm = SlotAssembler::new(start);
        let total = 15 * SAMPLE_RATE + 2 * SLOT_SAMPLES;
        let data: Vec<f32> = (0..total).map(|i| i as f32).collect();
        let mut collected = Vec::new();
        for chunk in data.chunks(777) {
            collected.extend(asm.push(chunk));
        }
        assert_eq!(collected.len(), 2);
        // First slot's first sample is sample index 15*12000.
        assert_eq!(collected[0].samples[0], (15 * SAMPLE_RATE) as f32);
    }
}
