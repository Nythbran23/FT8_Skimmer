//! Audio capture from the radio's USB codec via cpal.
//!
//! The cpal `Stream` is `!Send`, so it is created and owned entirely inside a
//! dedicated capture thread; only `Vec<f32>` sample chunks cross the channel
//! boundary. Stereo inputs are down-mixed to channel 0 (the codec is mono in
//! practice). Stopping drops the stream, which closes the channel and lets
//! the downstream pipeline / decoder threads terminate on their own.

use anyhow::{anyhow, Result};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::SampleFormat;
use crossbeam_channel::{unbounded, Receiver, Sender};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

/// Enumerate input devices. Returns (display name, device).
pub fn list_devices() -> Vec<(String, cpal::Device)> {
    let host = cpal::default_host();
    let mut out = Vec::new();
    if let Ok(devices) = host.input_devices() {
        for d in devices {
            let name = d.name().unwrap_or_else(|_| "<unknown>".to_string());
            out.push((name, d));
        }
    }
    out
}

/// Handle that keeps the capture thread alive; stops it on drop or `stop()`.
pub struct AudioHandle {
    running: Arc<AtomicBool>,
    join: Option<JoinHandle<()>>,
}

impl AudioHandle {
    pub fn stop(mut self) {
        self.shutdown();
    }
    fn shutdown(&mut self) {
        self.running.store(false, Ordering::SeqCst);
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

impl Drop for AudioHandle {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// Choose a supported input config, preferring rates that divide 12 kHz.
fn pick_config(device: &cpal::Device) -> Result<cpal::SupportedStreamConfig> {
    let prefs = [48_000u32, 24_000, 12_000, 96_000, 44_100];
    let ranges: Vec<_> = device
        .supported_input_configs()
        .map_err(|e| anyhow!("supported_input_configs: {e}"))?
        .collect();
    for &want in &prefs {
        for r in &ranges {
            let fmt_ok = matches!(
                r.sample_format(),
                SampleFormat::F32 | SampleFormat::I16 | SampleFormat::U16
            );
            if r.channels() >= 1
                && r.min_sample_rate().0 <= want
                && want <= r.max_sample_rate().0
                && fmt_ok
            {
                return Ok(r.with_sample_rate(cpal::SampleRate(want)));
            }
        }
    }
    device
        .default_input_config()
        .map_err(|e| anyhow!("default_input_config: {e}"))
}

fn err_cb(e: cpal::StreamError) {
    eprintln!("[audio] stream error: {e}");
}

/// Start capturing from `device`. Returns the raw-sample receiver (mono,
/// at the codec's native rate), that native rate, and a lifetime handle.
pub fn start(device: cpal::Device) -> Result<(Receiver<Vec<f32>>, u32, AudioHandle)> {
    let chosen = pick_config(&device)?;
    let rate = chosen.sample_rate().0;
    let (tx, rx) = unbounded::<Vec<f32>>();
    let running = Arc::new(AtomicBool::new(true));
    let run_thread = running.clone();

    let join = std::thread::spawn(move || {
        let config: cpal::StreamConfig = chosen.config();
        let channels = config.channels as usize;
        let fmt = chosen.sample_format();

        let stream = build_stream(&device, &config, fmt, channels, tx);
        let stream = match stream {
            Ok(s) => s,
            Err(e) => {
                eprintln!("[audio] could not build stream: {e}");
                return;
            }
        };
        if let Err(e) = stream.play() {
            eprintln!("[audio] could not start stream: {e}");
            return;
        }
        while run_thread.load(Ordering::SeqCst) {
            std::thread::sleep(Duration::from_millis(100));
        }
        drop(stream); // closes the channel
    });

    Ok((rx, rate, AudioHandle { running, join: Some(join) }))
}

fn build_stream(
    device: &cpal::Device,
    config: &cpal::StreamConfig,
    fmt: SampleFormat,
    channels: usize,
    tx: Sender<Vec<f32>>,
) -> Result<cpal::Stream> {
    let stream = match fmt {
        SampleFormat::F32 => {
            let tx = tx.clone();
            device.build_input_stream(
                config,
                move |data: &[f32], _: &cpal::InputCallbackInfo| {
                    let mono: Vec<f32> = data.chunks(channels).map(|c| c[0]).collect();
                    let _ = tx.send(mono);
                },
                err_cb,
                None,
            )?
        }
        SampleFormat::I16 => {
            let tx = tx.clone();
            device.build_input_stream(
                config,
                move |data: &[i16], _: &cpal::InputCallbackInfo| {
                    let mono: Vec<f32> = data
                        .chunks(channels)
                        .map(|c| c[0] as f32 / 32768.0)
                        .collect();
                    let _ = tx.send(mono);
                },
                err_cb,
                None,
            )?
        }
        SampleFormat::U16 => {
            let tx = tx.clone();
            device.build_input_stream(
                config,
                move |data: &[u16], _: &cpal::InputCallbackInfo| {
                    let mono: Vec<f32> = data
                        .chunks(channels)
                        .map(|c| (c[0] as f32 - 32768.0) / 32768.0)
                        .collect();
                    let _ = tx.send(mono);
                },
                err_cb,
                None,
            )?
        }
        other => return Err(anyhow!("unsupported sample format: {other:?}")),
    };
    Ok(stream)
}
