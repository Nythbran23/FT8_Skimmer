//! The processing pipeline thread.
//!
//! Takes raw codec-rate samples and:
//!  * resamples to 12 kHz,
//!  * runs a sliding FFT for the waterfall,
//!  * frames UTC-aligned 15 s slots,
//!  * forwards completed slots to the decoder thread.

use crate::dsp::{Resampler, Spectrum};
use crate::slot::{Slot, SlotAssembler, SAMPLE_RATE};
use chrono::{DateTime, Utc};
use crossbeam_channel::{unbounded, Receiver};

/// FFT size for the waterfall (2.93 Hz/bin at 12 kHz).
pub const FFT_N: usize = 4096;
/// Hop between waterfall columns.
pub const HOP: usize = 1024;
/// Number of low bins forwarded for display (0..~3 kHz).
pub const WF_BINS: usize = 1024;

/// Events the pipeline emits to the UI.
pub enum PipelineEvent {
    /// One waterfall column: linear magnitudes (`WF_BINS` long) plus the
    /// wall-clock UTC of the column, so the UI can place slot-boundary
    /// markers on true 15 s marks.
    Spectrum {
        utc: DateTime<Utc>,
        mags: Vec<f32>,
    },
    /// RMS level of the most recent chunk (roughly 0..1).
    Level(f32),
    /// A slot was framed and forwarded to the decoder.
    SlotCaptured {
        utc: DateTime<Utc>,
        samples: usize,
    },
}

/// Spawn the pipeline. Returns (UI event channel, slot channel for the
/// decoder). Both threads downstream terminate cleanly when `raw_rx` closes.
pub fn spawn(
    raw_rx: Receiver<Vec<f32>>,
    in_rate: u32,
) -> (Receiver<PipelineEvent>, Receiver<Slot>) {
    let (ev_tx, ev_rx) = unbounded::<PipelineEvent>();
    let (slot_tx, slot_rx) = unbounded::<Slot>();

    std::thread::spawn(move || {
        let mut resampler = Resampler::for_rate(in_rate, SAMPLE_RATE as u32);
        let mut spectrum = Spectrum::new(FFT_N);
        // Stream origin — the single audio clock both the slot assembler and
        // the waterfall are timed against. Stamping spectrum columns from
        // this (rather than Utc::now()) keeps waterfall slot indices in step
        // with the decode slot indices; wall-clock drift under pipeline
        // latency previously put them out by a slot, so the skimmer could not
        // match a decode to its stripe.
        let stream_start = Utc::now();
        let mut assembler = SlotAssembler::new(stream_start);
        let mut wf_acc: Vec<f32> = Vec::new();
        // 12 kHz samples consumed from the resampled stream so far.
        let mut samples_seen: u64 = 0;

        while let Ok(chunk) = raw_rx.recv() {
            let samples = resampler.process(&chunk);
            if samples.is_empty() {
                continue;
            }

            // Level meter.
            let rms = (samples.iter().map(|x| x * x).sum::<f32>()
                / samples.len() as f32)
                .sqrt();
            let _ = ev_tx.send(PipelineEvent::Level(rms));

            // Sliding-FFT waterfall columns. Each column is timestamped by
            // its position in the audio stream — the same clock as slot.utc.
            wf_acc.extend_from_slice(&samples);
            // Sample index of the oldest sample currently in wf_acc.
            let mut col_start = samples_seen + samples.len() as u64
                - wf_acc.len() as u64;
            while wf_acc.len() >= FFT_N {
                let mags = spectrum.process(&wf_acc[..FFT_N]);
                let col: Vec<f32> = mags.into_iter().take(WF_BINS).collect();
                // Time at the centre of this FFT window.
                let centre = col_start + (FFT_N as u64) / 2;
                let secs = centre as f64 / SAMPLE_RATE as f64;
                let utc = stream_start
                    + chrono::Duration::milliseconds((secs * 1000.0) as i64);
                let _ = ev_tx.send(PipelineEvent::Spectrum { utc, mags: col });
                wf_acc.drain(..HOP);
                col_start += HOP as u64;
            }
            samples_seen += samples.len() as u64;

            // Slot framing.
            for slot in assembler.push(&samples) {
                let _ = ev_tx.send(PipelineEvent::SlotCaptured {
                    utc: slot.utc,
                    samples: slot.samples.len(),
                });
                if slot_tx.send(slot).is_err() {
                    return; // decoder gone
                }
            }
        }
    });

    (ev_rx, slot_rx)
}
