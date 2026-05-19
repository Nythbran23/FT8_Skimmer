//! Cross-period soft-LLR accumulation.
//!
//! FT8 stations routinely repeat the same message over several 15 s periods —
//! an unanswered CQ, a held call. Every transmission of an identical message
//! produces the *identical* 174-bit LDPC codeword, so the per-period soft LLR
//! vectors can simply be summed: combining `N` periods lifts the effective
//! SNR by ~10·log10(N) dB (3 dB at N=2, 9 dB at N=8). CRC-14 — checked inside
//! the LDPC decoder — is the sole arbiter, so a corrupt sum (e.g. one that
//! spans a message change) just fails CRC and is never emitted; there is no
//! false-decode risk from over-eager accumulation.
//!
//! An *undecoded* signal offers only two handles for cross-period association:
//! audio frequency and time offset (DT). Each track therefore keeps a sliding
//! window of the last [`WINDOW`] LLR vectors — old vectors age out (so a stale
//! message self-clears) and the track centroid eases toward each new sample
//! (so slow frequency drift is followed).
//!
//! This module is deliberately FFI-free and `Decode`-free: it does tracking
//! and summation only, and is unit-tested in isolation. `mshv.rs` drives it —
//! feeding LLR samples in and running the accumulated sums back through the
//! LDPC decoder.

use std::collections::VecDeque;

/// FT8 LDPC codeword length, bits.
pub const LLR_LEN: usize = 174;

/// Sliding-window depth — how many periods of LLRs are summed at most.
pub const WINDOW: usize = 8;
/// Association tolerance in audio frequency, Hz.
pub const FREQ_TOL_HZ: f32 = 4.0;
/// Association tolerance in time offset, seconds.
pub const DT_TOL_S: f32 = 0.30;
/// Drop a track after this many periods with no update.
pub const MAX_AGE: u64 = WINDOW as u64;

/// One candidate's base (non-AP) LLR vector for a single period.
#[derive(Clone)]
pub struct LlrSample {
    pub freq_hz: f32,
    pub dt: f32,
    pub sync: i32,
    pub llr: [f64; LLR_LEN],
    /// Caller-assigned identity for this sample's period — carried opaquely
    /// through the accumulator so that, when a track produces an accumulated
    /// decode, the exact set of contributing periods can be recovered. In
    /// `mshv.rs` this is the period's `periods` table row id, stable across
    /// slots, so a track's tag list correctly identifies contributing
    /// periods from earlier slots, not only the current one.
    pub tag: i64,
}

/// A signal followed across periods at roughly constant (freq, dt).
struct Track {
    freq_hz: f32,
    dt: f32,
    /// Windowed LLR vectors paired with their caller-assigned period tags.
    recent: VecDeque<(i64, [f64; LLR_LEN])>,
    last_period: u64,
    /// Last message reported for this track — for cross-slot de-duplication.
    last_reported: Option<String>,
    /// FT8 slot-sequence parity this track belongs to: 0 for even slots
    /// (UTC :00/:30), 1 for odd (:15/:45). An FT8 station transmits in one
    /// sequence only, so its overs always land in same-parity slots —
    /// adjacent slots can never carry two overs from the same sender. A track
    /// only ever accumulates samples of its own parity, so two different
    /// signals that happen to sit close in frequency in consecutive slots are
    /// never summed into one (corrupt) track.
    parity: u8,
}

impl Track {
    /// Element-wise sum of the windowed LLR vectors.
    fn sum(&self) -> [f64; LLR_LEN] {
        let mut s = [0.0f64; LLR_LEN];
        for (_, v) in &self.recent {
            for i in 0..LLR_LEN {
                s[i] += v[i];
            }
        }
        s
    }

    /// The period tags currently in the window, oldest first.
    fn tags(&self) -> Vec<i64> {
        self.recent.iter().map(|(tag, _)| *tag).collect()
    }
}

/// Tracks signals across periods and accumulates their LLR vectors.
pub struct Accumulator {
    tracks: Vec<Track>,
    period: u64,
    /// Parity of the slot currently being added — set by `next_period`,
    /// used by `associate` to keep tracks within one FT8 slot sequence.
    cur_parity: u8,
}

impl Default for Accumulator {
    fn default() -> Self {
        Self::new()
    }
}

impl Accumulator {
    pub fn new() -> Self {
        Self {
            tracks: Vec::new(),
            period: 0,
            cur_parity: 0,
        }
    }

    /// Advance to the next period and drop stale tracks. Call once per slot,
    /// before [`add`](Self::add).
    ///
    /// `parity` is the FT8 slot-sequence parity of the slot about to be
    /// added — 0 for an even UTC slot (:00/:30), 1 for an odd one (:15/:45),
    /// i.e. `(utc_unix_seconds / 15) % 2`. Samples added this period only
    /// associate with tracks of the same parity.
    pub fn next_period(&mut self, parity: u8) {
        self.period += 1;
        self.cur_parity = parity & 1;
        let p = self.period;
        self.tracks
            .retain(|t| p.saturating_sub(t.last_period) <= MAX_AGE);
    }

    /// Fold this period's candidate LLR samples into the tracks. Returns the
    /// indices of tracks that were updated — the candidates for an
    /// accumulated re-decode.
    pub fn add(&mut self, samples: &[LlrSample]) -> Vec<usize> {
        let mut touched: Vec<usize> = Vec::new();
        for s in samples {
            let idx = self.associate(s);
            let t = &mut self.tracks[idx];
            if t.recent.len() == WINDOW {
                t.recent.pop_front();
            }
            t.recent.push_back((s.tag, s.llr));
            // Ease the centroid toward the new sample so the track follows
            // slow frequency drift without chasing noise.
            t.freq_hz = 0.7 * t.freq_hz + 0.3 * s.freq_hz;
            t.dt = 0.7 * t.dt + 0.3 * s.dt;
            t.last_period = self.period;
            if !touched.contains(&idx) {
                touched.push(idx);
            }
        }
        touched
    }

    /// Find the track matching `s` (closest within tolerance), or create one.
    ///
    /// Only tracks of the current slot's parity are considered: an FT8
    /// station's overs always fall in same-parity slots, so a sample can only
    /// belong to a track of its own parity. A new track inherits the current
    /// parity.
    fn associate(&mut self, s: &LlrSample) -> usize {
        let mut best: Option<(usize, f32)> = None;
        for (i, t) in self.tracks.iter().enumerate() {
            if t.parity != self.cur_parity {
                continue;
            }
            let df = (t.freq_hz - s.freq_hz).abs();
            let dd = (t.dt - s.dt).abs();
            if df <= FREQ_TOL_HZ && dd <= DT_TOL_S && best.map_or(true, |(_, bd)| df < bd) {
                best = Some((i, df));
            }
        }
        if let Some((i, _)) = best {
            return i;
        }
        self.tracks.push(Track {
            freq_hz: s.freq_hz,
            dt: s.dt,
            recent: VecDeque::with_capacity(WINDOW),
            last_period: self.period,
            last_reported: None,
            parity: self.cur_parity,
        });
        self.tracks.len() - 1
    }

    /// Number of periods currently summed for a track.
    pub fn depth(&self, idx: usize) -> usize {
        self.tracks[idx].recent.len()
    }

    /// The accumulated (summed) LLR vector for a track.
    pub fn summed_llr(&self, idx: usize) -> [f64; LLR_LEN] {
        self.tracks[idx].sum()
    }

    /// The period tags currently summed for a track — the exact set of
    /// periods that fed an accumulated decode for this track.
    pub fn track_tags(&self, idx: usize) -> Vec<i64> {
        self.tracks[idx].tags()
    }

    /// A track's current centroid frequency, Hz.
    pub fn track_freq(&self, idx: usize) -> f32 {
        self.tracks[idx].freq_hz
    }

    /// A track's current centroid time offset, seconds.
    pub fn track_dt(&self, idx: usize) -> f32 {
        self.tracks[idx].dt
    }

    /// Record that `msg` was decoded for a track; returns `true` if this is a
    /// *new* result (differs from the last one reported for that track), so
    /// the caller can suppress repeats while still catching message changes.
    pub fn note_decode(&mut self, idx: usize, msg: &str) -> bool {
        let t = &mut self.tracks[idx];
        let is_new = t.last_reported.as_deref() != Some(msg);
        t.last_reported = Some(msg.to_string());
        is_new
    }

    /// Number of live tracks (diagnostic).
    pub fn track_count(&self) -> usize {
        self.tracks.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(freq: f32, fill: f64) -> LlrSample {
        LlrSample {
            freq_hz: freq,
            dt: 0.1,
            sync: 1,
            llr: [fill; LLR_LEN],
            tag: 0,
        }
    }

    /// Step the accumulator one slot, alternating parity 0,1,0,1,... as real
    /// consecutive UTC slots do.
    fn step(acc: &mut Accumulator, slot: u64) {
        acc.next_period((slot & 1) as u8);
    }

    #[test]
    fn same_frequency_accumulates_one_track() {
        let mut acc = Accumulator::new();
        // A station appears every *other* slot — same parity each time.
        for slot in [0u64, 2, 4] {
            step(&mut acc, slot);
            acc.add(&[sample(1000.0, 1.0)]);
        }
        assert_eq!(acc.track_count(), 1);
        assert_eq!(acc.depth(0), 3);
        // Three periods of all-1.0 LLRs sum to 3.0 per bit.
        assert!((acc.summed_llr(0)[0] - 3.0).abs() < 1e-9);
    }

    #[test]
    fn distant_frequency_forms_a_new_track() {
        let mut acc = Accumulator::new();
        step(&mut acc, 0);
        acc.add(&[sample(1000.0, 1.0), sample(1500.0, 1.0)]);
        assert_eq!(acc.track_count(), 2);
    }

    #[test]
    fn small_drift_stays_one_track() {
        let mut acc = Accumulator::new();
        step(&mut acc, 0);
        acc.add(&[sample(1000.0, 1.0)]);
        step(&mut acc, 2); // same parity, two slots on
        acc.add(&[sample(1002.5, 1.0)]); // within FREQ_TOL_HZ
        assert_eq!(acc.track_count(), 1);
        assert_eq!(acc.depth(0), 2);
    }

    #[test]
    fn opposite_parity_does_not_accumulate() {
        // Two signals at the same frequency but in opposite slot sequences
        // are different stations and must never share a track.
        let mut acc = Accumulator::new();
        step(&mut acc, 0); // even slot
        acc.add(&[sample(1000.0, 1.0)]);
        step(&mut acc, 1); // odd slot — adjacent, opposite parity
        acc.add(&[sample(1000.0, 1.0)]);
        assert_eq!(acc.track_count(), 2, "parities must form separate tracks");
        assert_eq!(acc.depth(0), 1);
        assert_eq!(acc.depth(1), 1);
    }

    #[test]
    fn both_parities_track_independently() {
        // A monitor hears stations in both sequences; each parity keeps its
        // own track and accumulates only within itself.
        let mut acc = Accumulator::new();
        for slot in 0..6u64 {
            step(&mut acc, slot);
            // An even-slot station at 1000 Hz, an odd-slot one at 2000 Hz.
            let freq = if slot % 2 == 0 { 1000.0 } else { 2000.0 };
            acc.add(&[sample(freq, 1.0)]);
        }
        assert_eq!(acc.track_count(), 2);
        // Three even slots, three odd — each track depth 3.
        assert_eq!(acc.depth(0), 3);
        assert_eq!(acc.depth(1), 3);
    }

    #[test]
    fn window_caps_accumulation_depth() {
        let mut acc = Accumulator::new();
        // Same-parity slots: 0, 2, 4, ...
        for k in 0..(WINDOW + 5) {
            step(&mut acc, (k as u64) * 2);
            acc.add(&[sample(1000.0, 1.0)]);
        }
        assert_eq!(acc.depth(0), WINDOW);
    }

    #[test]
    fn stale_tracks_are_pruned() {
        let mut acc = Accumulator::new();
        step(&mut acc, 0);
        acc.add(&[sample(1000.0, 1.0)]);
        for k in 1..(MAX_AGE + 3) {
            step(&mut acc, k);
        }
        assert_eq!(acc.track_count(), 0);
    }

    #[test]
    fn note_decode_flags_only_changes() {
        let mut acc = Accumulator::new();
        step(&mut acc, 0);
        acc.add(&[sample(1000.0, 1.0)]);
        assert!(acc.note_decode(0, "CQ G4ABC IO81"));
        assert!(!acc.note_decode(0, "CQ G4ABC IO81")); // repeat
        assert!(acc.note_decode(0, "G4ABC GW4WND 73")); // changed
    }
}
