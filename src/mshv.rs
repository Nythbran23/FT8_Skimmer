//! FFI wrapper over MSHV's FT8 decoder.
//!
//! `MshvDecoder` implements [`Ft8Decoder`], so it drops straight into
//! `spawn_decoder` wherever `StubDecoder` was used — the pipeline sees no
//! difference. The C++ side (MSHV's de-Qt'd `DecoderFt8` plus the
//! `cpp/ft8_shim` C ABI) is compiled by `build.rs`.
//!
//! Compiled only with `--features mshv`.
//!
//! Threading: a `MshvDecoder` is driven by exactly one thread — the decoder
//! thread spawned in `decoder::spawn_decoder`. The handle is never shared, so
//! `Send` is sound and `Sync` is neither needed nor implemented.

use crate::accumulate::{Accumulator, LlrSample, LLR_LEN};
use crate::decoder::{Decode, DecodeSource, Ft8Decoder};
use crate::slot::Slot;
use std::collections::HashSet;
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int, c_void};

/// One decoded message — mirrors `Ft8Result` in `cpp/ft8_shim.h` exactly.
#[repr(C)]
struct Ft8Result {
    time: [c_char; 16],
    snr_db: c_int,
    dt: f32,
    freq_hz: c_int,
    message: [c_char; 48],
    aptype: [c_char; 8],
    qual: f32,
}

/// Opaque C++ decoder handle.
#[repr(C)]
struct Ft8DecoderC {
    _private: [u8; 0],
}

/// One candidate's base LLR vector — mirrors `Ft8LlrSample` in `ft8_shim.h`.
#[repr(C)]
struct Ft8LlrSample {
    freq_hz: f32,
    dt: f32,
    sync: c_int,
    llr: [f64; LLR_LEN],
}

type Ft8ResultCb = extern "C" fn(*mut c_void, *const Ft8Result);
type Ft8LlrCb = extern "C" fn(*mut c_void, *const Ft8LlrSample);

extern "C" {
    fn ft8_decoder_new(id: c_int) -> *mut Ft8DecoderC;
    fn ft8_decoder_free(dec: *mut Ft8DecoderC);
    fn ft8_decoder_set_depth(dec: *mut Ft8DecoderC, depth: c_int);
    fn ft8_decoder_decode(
        dec: *mut Ft8DecoderC,
        samples: *const f32,
        n_samples: c_int,
        utc: *const c_char,
        f_lo: f64,
        f_hi: f64,
        f_qso: f64,
        cb: Ft8ResultCb,
        ctx: *mut c_void,
    ) -> c_int;
    fn ft8_decoder_set_llr_callback(
        dec: *mut Ft8DecoderC,
        cb: Option<Ft8LlrCb>,
        ctx: *mut c_void,
    );
    fn ft8_ldpc_try(
        dec: *mut Ft8DecoderC,
        llr174: *const f64,
        msg_out: *mut c_char,
        msg_cap: c_int,
    ) -> c_int;
}

/// Audio passband searched for signals, Hz. The full FT8 sub-band.
const F_LO: f64 = 200.0;
const F_HI: f64 = 3000.0;
/// Nominal QSO frequency, Hz — only steers a-priori passes; decode is
/// full-passband regardless.
const F_QSO: f64 = 1500.0;

/// MSHV FT8 multi-decode, wrapped as an [`Ft8Decoder`].
///
/// Beyond stock single-shot decoding, each slot's per-candidate LLR vectors
/// are fed to an [`Accumulator`]; signals seen weakly across several periods
/// are soft-combined and re-run through the LDPC decoder, surfacing decodes
/// the single-shot path misses.
pub struct MshvDecoder {
    handle: *mut Ft8DecoderC,
    acc: Accumulator,
    /// Soft-LLR evidence from the most recent `decode()`, awaiting collection
    /// by the decoder thread via `take_period_soft`. One entry per candidate
    /// the LLR callback reported.
    period_soft: Vec<crate::store::PeriodSoft>,
    /// The most recent slot's LLR samples, held between `decode()` and
    /// `run_accumulation()`. Same order as `period_soft`, so the caller's
    /// period row ids line up element-for-element.
    pending_samples: Vec<LlrSample>,
    /// Messages found single-shot in the most recent slot — so the
    /// accumulated path does not report them a second time.
    pending_single: HashSet<String>,
    /// UTC of the most recent slot.
    pending_utc: chrono::DateTime<chrono::Utc>,
}

// The handle is owned and used by a single thread (see module docs).
unsafe impl Send for MshvDecoder {}

impl MshvDecoder {
    /// Create a decoder. `depth` is 1 (fast), 2 (normal) or 3 (deep).
    pub fn with_depth(depth: i32) -> Self {
        let handle = unsafe { ft8_decoder_new(0) };
        assert!(!handle.is_null(), "ft8_decoder_new returned null");
        unsafe { ft8_decoder_set_depth(handle, depth as c_int) };
        Self {
            handle,
            acc: Accumulator::new(),
            period_soft: Vec::new(),
            pending_samples: Vec::new(),
            pending_single: HashSet::new(),
            pending_utc: chrono::Utc::now(),
        }
    }

    /// Create a decoder at deep-search depth.
    pub fn new() -> Self {
        Self::with_depth(3)
    }

    /// Run the LDPC decoder on a single 174-element LLR vector.
    ///
    /// Returns `true` if the vector decodes to a CRC-valid FT8 codeword —
    /// i.e. this soft evidence would decode single-shot on its own. Used both
    /// for the `periods.decoded` capture flag and by the `ft8mon-backfill`
    /// tool to recompute that flag for already-captured databases.
    pub fn llr_decodes(&self, llr: &[f64]) -> bool {
        if llr.len() != LLR_LEN {
            return false;
        }
        let mut buf = [0f64; LLR_LEN];
        buf.copy_from_slice(llr);
        ldpc_try(self.handle, &buf).is_some()
    }
}

impl Default for MshvDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for MshvDecoder {
    fn drop(&mut self) {
        unsafe { ft8_decoder_free(self.handle) };
    }
}

/// Context handed across the FFI boundary so the C callback can append
/// decodes and stamp them with the slot's UTC.
struct Collect {
    out: Vec<Decode>,
    slot_utc: chrono::DateTime<chrono::Utc>,
}

/// Invoked by the shim once per decoded message, synchronously, inside
/// `ft8_decoder_decode`.
extern "C" fn collect_cb(ctx: *mut c_void, r: *const Ft8Result) {
    // Safety: `ctx` is the `&mut Collect` we pass below; `r` is a valid
    // `Ft8Result` for the duration of this call. Both upheld by the shim.
    let collect = unsafe { &mut *(ctx as *mut Collect) };
    let r = unsafe { &*r };

    collect.out.push(Decode {
        utc: collect.slot_utc,
        snr_db: r.snr_db,
        dt: r.dt,
        freq_hz: r.freq_hz as f32,
        message: cstr(r.message.as_ptr()),
        source: DecodeSource::SingleShot,
        contrib_tags: Vec::new(),
    });
}

/// Read a NUL-terminated C string into an owned `String` (lossy on non-UTF-8).
fn cstr(ptr: *const c_char) -> String {
    if ptr.is_null() {
        return String::new();
    }
    // Safety: the shim NUL-terminates every char buffer it fills.
    unsafe { CStr::from_ptr(ptr) }.to_string_lossy().into_owned()
}

/// Invoked by the shim once per candidate, with that candidate's base LLR
/// vector. `ctx` is the `&mut Vec<LlrSample>` passed during decode.
extern "C" fn llr_collect_cb(ctx: *mut c_void, s: *const Ft8LlrSample) {
    // Safety: `ctx` is the `&mut Vec<LlrSample>` set below; `s` is valid for
    // the duration of this call. Both upheld by the shim.
    let out = unsafe { &mut *(ctx as *mut Vec<LlrSample>) };
    let s = unsafe { &*s };
    out.push(LlrSample {
        freq_hz: s.freq_hz,
        dt: s.dt,
        sync: s.sync,
        llr: s.llr,
        // Real tag (the slot-local index) is assigned after collection, once
        // the full sample order — and thus the period-row order — is known.
        tag: 0,
    });
}

/// Run the LDPC decoder on an accumulated LLR sum. `Some(msg)` on a
/// CRC-valid decode, `None` otherwise.
fn ldpc_try(handle: *mut Ft8DecoderC, llr: &[f64; LLR_LEN]) -> Option<String> {
    let mut buf = [0u8; 48];
    // Safety: handle is non-null; `llr` is 174 doubles; `buf` is a valid
    // writable buffer of known length.
    let ok = unsafe {
        ft8_ldpc_try(
            handle,
            llr.as_ptr(),
            buf.as_mut_ptr() as *mut c_char,
            buf.len() as c_int,
        )
    };
    if ok == 0 {
        None
    } else {
        Some(cstr(buf.as_ptr() as *const c_char))
    }
}

impl Ft8Decoder for MshvDecoder {
    fn decode(&mut self, slot: &Slot) -> Vec<Decode> {
        let mut collect = Collect {
            out: Vec::new(),
            slot_utc: slot.utc,
        };
        // Filled by `llr_collect_cb` via the raw pointer passed below.
        let mut llr_samples: Vec<LlrSample> = Vec::new();
        // MSHV takes the slot time as a string; HHMMSS is enough for it.
        let utc = CString::new(slot.utc.format("%H%M%S").to_string())
            .unwrap_or_else(|_| CString::new("000000").unwrap());

        // Register the LLR sink, run the decode (which fires both the result
        // and LLR callbacks synchronously), then detach the LLR sink.
        //
        // Safety: handle is non-null; `samples` is a valid slice; `collect`
        // and `llr_samples` outlive the synchronous call.
        unsafe {
            ft8_decoder_set_llr_callback(
                self.handle,
                Some(llr_collect_cb),
                &mut llr_samples as *mut Vec<LlrSample> as *mut c_void,
            );
            ft8_decoder_decode(
                self.handle,
                slot.samples.as_ptr(),
                slot.samples.len() as c_int,
                utc.as_ptr(),
                F_LO,
                F_HI,
                F_QSO,
                collect_cb,
                &mut collect as *mut Collect as *mut c_void,
            );
            ft8_decoder_set_llr_callback(self.handle, None, std::ptr::null_mut());
        }

        let out = collect.out;

        // --- soft-LLR capture --------------------------------------------
        // Stash one PeriodSoft per candidate for the `periods` table.
        //
        // The `decoded` flag means "this period's own soft evidence decodes
        // single-shot" — so it is determined by actually running the LDPC
        // decoder on each candidate's LLR vector, not by matching the
        // candidate against the result list. Coordinate matching is unsound
        // here: the LLR callback fires pre-decode with the *sync-stage*
        // (f1, xdt), while a `Decode` carries MSHV's post-decode *refined*
        // (freq, dt). FT8's DT refinement routinely shifts the estimate by
        // more than the accumulator's 0.30 s tolerance, so the match would
        // silently fail — which is exactly why every captured period landed
        // with decoded=0. Decoding the period's own LLR is exact and immune
        // to that drift, and `ldpc_try` is cheap on a single 174-vector.
        self.period_soft.clear();
        self.period_soft.reserve(llr_samples.len());
        for s in &llr_samples {
            let decoded = self.llr_decodes(&s.llr);
            self.period_soft.push(crate::store::PeriodSoft {
                utc: slot.utc,
                freq_hz: s.freq_hz,
                dt: s.dt,
                sync: s.sync,
                llr: s.llr.to_vec(),
                decoded,
            });
        }

        // Hold this slot's samples and single-shot messages for
        // `run_accumulation`, which is called once the caller has written
        // the `periods` rows and can supply their database ids as tags.
        self.pending_samples = llr_samples;
        self.pending_single = out.iter().map(|d| d.message.clone()).collect();
        self.pending_utc = slot.utc;

        out
    }

    fn name(&self) -> &str {
        "mshv-ft8"
    }

    fn take_period_soft(&mut self) -> Vec<crate::store::PeriodSoft> {
        std::mem::take(&mut self.period_soft)
    }

    fn run_accumulation(&mut self, period_ids: &[i64]) -> Vec<Decode> {
        // Tag each held sample with its `periods` row id. The samples were
        // stashed in the same order as `period_soft`, hence the same order
        // the caller wrote the rows, so ids line up element-for-element.
        let mut samples = std::mem::take(&mut self.pending_samples);
        if period_ids.len() != samples.len() {
            // The caller did not write a row per period (capture-soft off,
            // or a store error). Without ids the links cannot be made, so
            // accumulation is skipped for this slot rather than mis-linked.
            return Vec::new();
        }
        for (s, &id) in samples.iter_mut().zip(period_ids) {
            s.tag = id;
        }

        let single = std::mem::take(&mut self.pending_single);
        let utc = self.pending_utc;
        let mut out = Vec::new();

        // FT8 slot-sequence parity: even UTC slot (:00/:30) -> 0,
        // odd (:15/:45) -> 1. Keeps each accumulation track within one
        // sequence — adjacent slots never carry two overs of one sender.
        let parity = ((utc.timestamp().div_euclid(15)) & 1) as u8;
        self.acc.next_period(parity);
        let touched = self.acc.add(&samples);
        for idx in touched {
            let depth = self.acc.depth(idx);
            if depth < 2 {
                continue; // only one period so far — nothing to combine
            }
            let summed = self.acc.summed_llr(idx);
            if let Some(msg) = ldpc_try(self.handle, &summed) {
                // note_decode records the message and tells us whether it is
                // new for this track (suppresses period-after-period repeats).
                let is_new = self.acc.note_decode(idx, &msg);
                if single.contains(&msg) || !is_new {
                    continue;
                }
                out.push(Decode {
                    utc,
                    snr_db: 0, // not meaningful for an accumulated LLR sum
                    dt: self.acc.track_dt(idx),
                    freq_hz: self.acc.track_freq(idx),
                    message: msg,
                    source: DecodeSource::Accumulated(depth as u32),
                    // The `periods` row ids of every contributing period —
                    // including those captured in earlier slots.
                    contrib_tags: self.acc.track_tags(idx),
                });
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::slot::{Slot, SLOT_SAMPLES};

    /// Exercises the whole Rust -> shim -> MSHV path: allocate the (~20 MB)
    /// decoder, run a full slot through `ft8_decode`, free it. A slot of
    /// silence must decode to nothing without crashing.
    #[test]
    fn decodes_silence_cleanly() {
        let mut dec = MshvDecoder::new();
        let slot = Slot {
            utc: chrono::Utc::now(),
            samples: vec![0.0f32; SLOT_SAMPLES],
        };
        let decodes = dec.decode(&slot);
        assert!(decodes.is_empty(), "silence yielded {} decode(s)", decodes.len());
    }
}
