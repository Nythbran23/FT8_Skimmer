//! The decoder boundary.
//!
//! `Ft8Decoder` is the seam where the real engine plugs in. Today the only
//! implementor is `StubDecoder`, which does no FT8 decoding — it reports the
//! strongest spectral peaks in a slot purely so the capture -> framing ->
//! decode -> UI pipeline can be exercised against live off-air audio.
//!
//! The next phase replaces `StubDecoder` with `MshvDecoder`: an FFI wrapper
//! over the (de-Qt'd, instrumented) MSHV `ft8b()` multi-decode. The trait
//! signature is deliberately slot-in / `Vec<Decode>`-out so that swap is the
//! only change the pipeline sees. Soft-LLR accumulation is a *second*
//! implementor that wraps the MSHV one and keeps per-signal LLR tracks.

use crate::dsp::Spectrum;
use crate::slot::{Slot, SAMPLE_RATE};
use chrono::{DateTime, Utc};
use crossbeam_channel::{unbounded, Receiver};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;

/// How a decode was obtained — lets the UI show single-shot vs accumulation.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DecodeSource {
    /// Decoded from a single 15 s slot (stock FT8).
    SingleShot,
    /// Decoded only after soft-combining `n` slots.
    Accumulated(u32),
    /// Decoded both ways (combined `n` slots, also caught single-shot).
    Both(u32),
}

impl DecodeSource {
    pub fn badge(&self) -> String {
        match self {
            DecodeSource::SingleShot => "single".to_string(),
            DecodeSource::Accumulated(n) => format!("accum x{n}"),
            DecodeSource::Both(n) => format!("both x{n}"),
        }
    }
}

/// One decoded FT8 message.
#[derive(Clone, Debug)]
pub struct Decode {
    /// UTC of the slot the decode belongs to.
    pub utc: DateTime<Utc>,
    /// Signal-to-noise estimate, dB in 2500 Hz reference bandwidth.
    pub snr_db: i32,
    /// Time offset of the signal, seconds.
    pub dt: f32,
    /// Audio frequency in the passband, Hz.
    pub freq_hz: f32,
    /// Decoded message text.
    pub message: String,
    /// Provenance of this decode.
    pub source: DecodeSource,
    /// For an `Accumulated`/`Both` decode: the `periods` table row ids of the
    /// periods whose soft-LLR vectors were summed to produce it — written as
    /// `accum_periods` links by the decoder thread. Empty for a single-shot
    /// decode. Populated by `run_accumulation`, which is given the period row
    /// ids after the `periods` rows have been written.
    pub contrib_tags: Vec<i64>,
}

/// A slot decoder. One call must find *every* signal in the passband
/// (FT8 multi-decode), not just the strongest.
///
/// Decoding a slot is a two-step sequence so that cross-period accumulation
/// can be linked to the exact `periods` rows it combined:
///
///  1. [`decode`](Self::decode) runs the single-shot decode and stashes this
///     slot's soft-LLR evidence (drained by [`take_period_soft`]).
///  2. The caller writes the period rows, obtaining their database ids, then
///     calls [`run_accumulation`](Self::run_accumulation) with those ids. Any
///     accumulated decodes come back tagged with the row ids of their
///     contributing periods.
///
/// A decoder with no accumulation (the stub) implements only `decode`; the
/// default `run_accumulation` returns nothing.
pub trait Ft8Decoder: Send {
    /// Single-shot decode of one slot. Also collects the slot's soft-LLR
    /// evidence, retrievable via [`take_period_soft`](Self::take_period_soft).
    fn decode(&mut self, slot: &Slot) -> Vec<Decode>;

    fn name(&self) -> &str;

    /// Drain the soft-LLR evidence collected during the most recent
    /// `decode()` call, for persistence in the `periods` table.
    ///
    /// Each [`PeriodSoft`] is one 15 s period the decoder evaluated, with its
    /// 174-element soft-LLR vector and a flag for whether it decoded
    /// single-shot. The default returns nothing — only a soft-decision
    /// decoder (the MSHV engine) has LLRs to hand over; the stub does not.
    fn take_period_soft(&mut self) -> Vec<crate::store::PeriodSoft> {
        Vec::new()
    }

    /// Run cross-period soft-LLR accumulation for the slot just decoded.
    ///
    /// `period_ids` are the `periods` table row ids of the soft evidence from
    /// the most recent `decode()`, in the same order as `take_period_soft`
    /// returned it. The accumulator uses them as period tags, so accumulated
    /// decodes carry the row ids of *every* contributing period — including
    /// those captured in earlier slots — in their `contrib_tags`.
    ///
    /// Returns any decodes recovered only by accumulation. The default
    /// returns nothing (the stub does not accumulate).
    fn run_accumulation(&mut self, _period_ids: &[i64]) -> Vec<Decode> {
        Vec::new()
    }
}

/// Placeholder decoder: reports the loudest spectral peaks in a slot.
///
/// This does NOT decode FT8. It exists so the rest of the application can be
/// run and verified against a real radio today — you will see a row per
/// strong carrier, proving audio -> slot -> decode -> table all work.
pub struct StubDecoder {
    spec: Spectrum,
}

impl StubDecoder {
    pub fn new() -> Self {
        Self {
            spec: Spectrum::new(4096),
        }
    }
}

impl Default for StubDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Ft8Decoder for StubDecoder {
    fn decode(&mut self, slot: &Slot) -> Vec<Decode> {
        let n = 4096;
        if slot.samples.len() < n {
            return Vec::new();
        }
        // Analyse a window from the middle of the slot.
        let start = (slot.samples.len() - n) / 2;
        let mags = self.spec.process(&slot.samples[start..start + n]);
        let bin_hz = SAMPLE_RATE as f32 / n as f32;

        // Median magnitude as a crude noise reference.
        let mut sorted = mags.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let med = sorted[sorted.len() / 2].max(1e-9);

        // Local-maximum peak picking across the FT8 audio passband.
        let lo = (200.0 / bin_hz) as usize;
        let hi = ((2800.0 / bin_hz) as usize).min(mags.len() - 2);
        let mut peaks: Vec<(usize, f32)> = Vec::new();
        for i in lo.max(1)..hi {
            if mags[i] > mags[i - 1] && mags[i] >= mags[i + 1] && mags[i] > med * 6.0 {
                peaks.push((i, mags[i]));
            }
        }
        peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        peaks.truncate(6);

        peaks
            .into_iter()
            .map(|(i, m)| Decode {
                utc: slot.utc,
                snr_db: (10.0 * (m / med).log10()) as i32 - 26,
                dt: 0.0,
                freq_hz: i as f32 * bin_hz,
                message: "(stub — MSHV FT8 decoder not yet integrated)".to_string(),
                source: DecodeSource::SingleShot,
                contrib_tags: Vec::new(),
            })
            .collect()
    }

    fn name(&self) -> &str {
        "stub"
    }
}

/// The decoder the application runs.
///
/// With the `mshv` feature this is the real MSHV FT8 multi-decode
/// (`MshvDecoder`); without it, `StubDecoder` — spectral peaks only, no
/// actual decoding. The pipeline calls this and is otherwise feature-agnostic.
pub fn default_decoder() -> Box<dyn Ft8Decoder> {
    #[cfg(feature = "mshv")]
    {
        Box::new(crate::mshv::MshvDecoder::new())
    }
    #[cfg(not(feature = "mshv"))]
    {
        Box::new(StubDecoder::new())
    }
}

/// Spawn a background thread that decodes each incoming slot.
///
/// The decoder runs off the UI thread so a slow decode never stalls the
/// waterfall. Returns the channel of decode batches. The thread ends cleanly
/// when `slot_rx` is closed (i.e. the pipeline stops).
///
/// `dial_hz` is the RF dial frequency (shared, live-updated by the UI) and
/// `pskr_enabled` toggles PSK Reporter uploads. Every standard decode that
/// carries a callsign and grid is queued as a spot; the reporter flushes a
/// datagram every five minutes.
///
/// `store`, when present, captures every decode to the `decodes` table and —
/// if the store has soft capture enabled — every period's soft-LLR evidence
/// to the `periods` table. The store is owned here, on the decoder thread, so
/// the decoder itself stays a clean plug-in that knows nothing of the
/// database; it merely hands soft evidence out via `take_period_soft`.
pub fn spawn_decoder(
    slot_rx: Receiver<Slot>,
    mut decoder: Box<dyn Ft8Decoder>,
    dial_hz: Arc<AtomicU64>,
    pskr_enabled: Arc<AtomicBool>,
    store: Option<crate::store::Store>,
) -> Receiver<Vec<Decode>> {
    use crate::pskreporter::{self, Reporter, Spot};

    let (tx, rx) = unbounded();
    std::thread::spawn(move || {
        // Receiving-station identity — hard-coded for now (GW4WND / IO82KM).
        let mut reporter = Reporter::new("GW4WND", "IO82KM", "");

        while let Ok(slot) = slot_rx.recv() {
            // Step 1: single-shot decode. This also stashes the slot's
            // soft-LLR evidence inside the decoder.
            let mut decodes = decoder.decode(&slot);

            // Step 2: persist the period rows and collect their database ids,
            // in the order `take_period_soft` returns them. Step 3 feeds
            // those ids to the accumulator so accumulated decodes can be
            // linked to the exact periods they combined.
            let mut period_ids: Vec<i64> = Vec::new();
            if let Some(store) = &store {
                for d in &decodes {
                    store.record_decode(d);
                }
                if store.capture_soft() {
                    for p in decoder.take_period_soft() {
                        // A failed insert yields a sentinel id; run_accumulation
                        // checks the count and skips linking if anything is
                        // amiss, so a stray -1 cannot mislink.
                        period_ids.push(store.record_period(&p).unwrap_or(-1));
                    }
                }
            }

            // Step 3: cross-period accumulation, tagged with the period row
            // ids. Each accumulated decode is written and linked to every
            // contributing period via the accum_periods table.
            let accumulated = decoder.run_accumulation(&period_ids);
            if let Some(store) = &store {
                for d in &accumulated {
                    if let Some(decode_id) = store.record_decode(d) {
                        for &pid in &d.contrib_tags {
                            if pid >= 0 {
                                store.record_accum_link(decode_id, pid);
                            }
                        }
                    }
                }
            }
            decodes.extend(accumulated);

            if pskr_enabled.load(Ordering::Relaxed) {
                let dial = dial_hz.load(Ordering::Relaxed);
                let time = slot.utc.timestamp().max(0) as u32;
                for d in &decodes {
                    if let Some((call, grid)) = pskreporter::parse_spot(&d.message) {
                        // Accumulation-only decodes have no calibrated SNR;
                        // sent with SNR 0 on the single sender template.
                        let snr = match d.source {
                            DecodeSource::Accumulated(_) => None,
                            _ => Some(d.snr_db),
                        };
                        reporter.add_spot(Spot {
                            call,
                            grid,
                            freq_hz: (dial + d.freq_hz.round().max(0.0) as u64) as u32,
                            snr,
                            mode: "FT8".to_string(),
                            time,
                        });
                    }
                }
                reporter.tick();
            }

            if tx.send(decodes).is_err() {
                break;
            }
        }

        // Pipeline stopped — mark the session ended.
        if let Some(store) = &store {
            store.close_session();
        }
    });
    rx
}
