//! ft8mon — FT8 multi-decode RX monitor.
//!
//! The crate splits into two halves:
//!  * the **core** (this library): audio capture, resampling, UTC slot
//!    framing, the decoder trait, and the processing pipeline. No GUI deps,
//!    no FT8 modem — fully unit-testable on its own.
//!  * the **GUI** (`src/app.rs`, feature `gui`): an egui front-end with a
//!    live waterfall and a multi-decode table.
//!
//! The FT8 decoder itself is intentionally a trait (`decoder::Ft8Decoder`).
//! `StubDecoder` ships as a proof-of-life placeholder; the real engine is the
//! instrumented MSHV `ft8b()` exposed over FFI, dropped in here later.

pub mod audio;
pub mod decoder;
pub mod dsp;
pub mod pipeline;
pub mod pskreporter;
pub mod slot;
pub mod store;

/// FFI wrapper over MSHV's FT8 decoder. Built only with `--features mshv`.
#[cfg(feature = "mshv")]
pub mod mshv;

/// Cross-period soft-LLR accumulation. Built only with `--features mshv`.
#[cfg(feature = "mshv")]
pub mod accumulate;
