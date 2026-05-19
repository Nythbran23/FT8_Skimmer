//! DSP building blocks: windowing, FFT magnitude spectra, and sample-rate
//! conversion from the codec rate down to the 12 kHz the decoder needs.

use rustfft::{num_complex::Complex, Fft, FftPlanner};
use std::sync::Arc;

/// Hann window of length `n`.
pub fn hann(n: usize) -> Vec<f32> {
    if n <= 1 {
        return vec![1.0; n];
    }
    (0..n)
        .map(|i| {
            let v = (std::f32::consts::PI * i as f32 / (n as f32 - 1.0)).sin();
            v * v
        })
        .collect()
}

/// Windowed FFT magnitude spectrum.
pub struct Spectrum {
    fft: Arc<dyn Fft<f32>>,
    n: usize,
    win: Vec<f32>,
    buf: Vec<Complex<f32>>,
}

impl Spectrum {
    pub fn new(n: usize) -> Self {
        let fft = FftPlanner::<f32>::new().plan_fft_forward(n);
        Self {
            fft,
            n,
            win: hann(n),
            buf: vec![Complex::new(0.0, 0.0); n],
        }
    }

    /// Magnitude (linear) for bins `0..n/2`. `samples` shorter than `n` is
    /// zero-padded; longer is truncated.
    pub fn process(&mut self, samples: &[f32]) -> Vec<f32> {
        for i in 0..self.n {
            let s = samples.get(i).copied().unwrap_or(0.0);
            self.buf[i] = Complex::new(s * self.win[i], 0.0);
        }
        self.fft.process(&mut self.buf);
        let scale = 2.0 / self.n as f32;
        self.buf[..self.n / 2].iter().map(|c| c.norm() * scale).collect()
    }
}

/// Windowed-sinc low-pass FIR with unity DC gain.
fn design_lowpass(num_taps: usize, fc: f32) -> Vec<f32> {
    let mid = (num_taps - 1) as f32 / 2.0;
    let win = hann(num_taps);
    let mut h: Vec<f32> = (0..num_taps)
        .map(|k| {
            let x = k as f32 - mid;
            let sinc = if x.abs() < 1e-6 {
                2.0 * fc
            } else {
                (2.0 * std::f32::consts::PI * fc * x).sin() / (std::f32::consts::PI * x)
            };
            sinc * win[k]
        })
        .collect();
    let sum: f32 = h.iter().sum();
    if sum.abs() > 1e-9 {
        for v in &mut h {
            *v /= sum;
        }
    }
    h
}

/// Integer-factor decimator (anti-alias FIR + downsample), stateful across
/// arbitrary chunk sizes via a sample-history buffer and a phase offset.
pub struct Decimator {
    taps: Vec<f32>,
    factor: usize,
    hist: Vec<f32>, // last taps.len()-1 input samples
    offset: usize,  // index of first output sample within next input chunk
}

impl Decimator {
    pub fn new(factor: usize) -> Self {
        assert!(factor >= 2);
        let num_taps = 8 * factor + 1;
        let fc = 0.45 / factor as f32; // cutoff just below the new Nyquist
        Self {
            taps: design_lowpass(num_taps, fc),
            factor,
            hist: vec![0.0; num_taps - 1],
            offset: 0,
        }
    }

    pub fn process(&mut self, input: &[f32]) -> Vec<f32> {
        let nt = self.taps.len();
        // Virtual stream s = hist ++ input; an output centred at input
        // position i reads s[i ..= i+nt-1].
        let mut s = Vec::with_capacity(self.hist.len() + input.len());
        s.extend_from_slice(&self.hist);
        s.extend_from_slice(input);

        let mut out = Vec::with_capacity(input.len() / self.factor + 1);
        let mut i = self.offset;
        while i < input.len() {
            let base = i + nt - 1;
            let mut acc = 0.0;
            for t in 0..nt {
                acc += self.taps[t] * s[base - t];
            }
            out.push(acc);
            i += self.factor;
        }
        self.offset = i - input.len();
        let keep = nt - 1;
        self.hist = s[s.len().saturating_sub(keep)..].to_vec();
        if self.hist.len() < keep {
            let mut pad = vec![0.0; keep - self.hist.len()];
            pad.extend_from_slice(&self.hist);
            self.hist = pad;
        }
        out
    }
}

/// Stateful linear-interpolation resampler for non-integer rate ratios.
/// Adequate for the waterfall and a first decoder bring-up; swap in a
/// polyphase / `rubato` resampler before trusting odd-rate decoder feeds.
pub struct LinearResampler {
    step: f64, // input samples advanced per output sample
    pos: f64,  // fractional read position within `tail`
    tail: Vec<f32>,
}

impl LinearResampler {
    pub fn new(in_rate: f64, out_rate: f64) -> Self {
        Self {
            step: in_rate / out_rate,
            pos: 0.0,
            tail: Vec::new(),
        }
    }

    pub fn process(&mut self, input: &[f32]) -> Vec<f32> {
        let mut data = std::mem::take(&mut self.tail);
        data.extend_from_slice(input);
        let mut out = Vec::new();
        let mut pos = self.pos;
        while (pos.floor() as usize) + 1 < data.len() {
            let i = pos.floor() as usize;
            let f = (pos - i as f64) as f32;
            out.push(data[i] * (1.0 - f) + data[i + 1] * f);
            pos += self.step;
        }
        let consumed = pos.floor() as usize;
        self.pos = pos - consumed as f64;
        self.tail = data[consumed.min(data.len())..].to_vec();
        out
    }
}

/// Codec-rate -> 12 kHz conversion, picking the cheapest correct method.
pub enum Resampler {
    Passthrough,
    Decimate(Decimator),
    Linear(LinearResampler),
}

impl Resampler {
    pub fn for_rate(in_rate: u32, out_rate: u32) -> Self {
        if in_rate == out_rate {
            Resampler::Passthrough
        } else if in_rate > out_rate && in_rate % out_rate == 0 {
            Resampler::Decimate(Decimator::new((in_rate / out_rate) as usize))
        } else {
            Resampler::Linear(LinearResampler::new(in_rate as f64, out_rate as f64))
        }
    }

    pub fn process(&mut self, input: &[f32]) -> Vec<f32> {
        match self {
            Resampler::Passthrough => input.to_vec(),
            Resampler::Decimate(d) => d.process(input),
            Resampler::Linear(l) => l.process(input),
        }
    }

    pub fn describe(&self) -> &'static str {
        match self {
            Resampler::Passthrough => "passthrough",
            Resampler::Decimate(_) => "integer decimation (FIR)",
            Resampler::Linear(_) => "linear (non-integer ratio)",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decimator_output_count_is_consistent() {
        // 48 kHz -> 12 kHz: 4:1, regardless of how input is chunked.
        let mut d = Decimator::new(4);
        let mut total = 0;
        for chunk in [1000usize, 333, 4096, 17].iter() {
            total += d.process(&vec![0.0; *chunk]).len();
        }
        let expected = (1000 + 333 + 4096 + 17) / 4;
        assert!((total as i64 - expected as i64).abs() <= 1);
    }

    #[test]
    fn decimator_passes_dc() {
        let mut d = Decimator::new(4);
        // Steady DC of 1.0 should come out near 1.0 once the FIR fills.
        let mut last = 0.0;
        for _ in 0..20 {
            for v in d.process(&vec![1.0; 4096]) {
                last = v;
            }
        }
        assert!((last - 1.0).abs() < 0.02, "DC gain off: {last}");
    }

    #[test]
    fn linear_resampler_rate_ratio() {
        // 44_100 -> 12_000 should yield ~ in_len * 12000/44100 samples.
        let mut r = LinearResampler::new(44_100.0, 12_000.0);
        let n_in = 44_100;
        let out = r.process(&vec![0.0; n_in]);
        let expected = n_in as f64 * 12_000.0 / 44_100.0;
        assert!((out.len() as f64 - expected).abs() < 2.0);
    }
}
