//! egui front-end: device picker, live waterfall, multi-decode table.
//!
//! This module is feature-gated (`gui`). It is the only part of the crate
//! that depends on egui/eframe. Build it with the default features on a
//! current Rust toolchain.

use ft8mon::audio::{self, AudioHandle};
use ft8mon::decoder::{self, Decode, DecodeSource};
use ft8mon::pipeline::{self, PipelineEvent, WF_BINS};
use chrono::{DateTime, Utc};
use crossbeam_channel::Receiver;
use eframe::egui::{self, Color32};
use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;

/// Rows of waterfall history kept on screen.
const WF_HEIGHT: usize = 320;
/// Cap on the decode list.
const MAX_DECODES: usize = 500;

/// Top of the audio passband shown on the skimmer frequency axis, Hz.
const SKIMMER_MAX_HZ: f32 = 3000.0;
/// Maximum recent 15 s slots the skimmer waterfall buffer ever retains.
/// The view draws some or all of these (see `App::skimmer_slots`), so
/// changing the display count is instant and never loses history.
const SKIMMER_SLOTS_MAX: usize = 24;
/// Slot-count choices offered by the view's selector button.
const SKIMMER_SLOT_CHOICES: [usize; 3] = [8, 16, 24];
/// Frequency bins in a skimmer waterfall stripe (0..SKIMMER_MAX_HZ).
/// WF_BINS at 2.93 Hz/bin already spans 0–3000 Hz, so the stripe reuses them.
const SKIMMER_BINS: usize = WF_BINS;
/// Time sub-columns per slot. Keeping several real FFT-time columns per slot
/// — rather than averaging the whole slot into one — preserves the time
/// axis, so a steady signal reads as a continuous horizontal trace and noise
/// flickers between columns (the WSJT-X waterfall model). One column per slot
/// collapsed that axis and made noise look like fixed streaks.
const SKIMMER_SUBCOLS: usize = 12;

/// Which main view is showing. Switchable live; purely a render choice —
/// nothing downstream (capture, decode, store, spotting) depends on it.
#[derive(Clone, Copy, PartialEq, Eq)]
enum ViewMode {
    /// Horizontal waterfall + full decode table (the original layout).
    Classic,
    /// Experimental vertical-spectrum band-activity strip: frequency on the
    /// vertical axis, recent slots as columns, each decode a mark inline
    /// with the trace, message text alongside.
    Skimmer,
}

/// One completed slot's waterfall data — `SKIMMER_SUBCOLS` time sub-columns,
/// each a full spectrum, so the slot retains its internal time axis.
struct SkimmerStripe {
    /// Slot index (UTC seconds / 15).
    slot: i64,
    /// `SKIMMER_SUBCOLS` sub-columns, oldest first; each is `SKIMMER_BINS`
    /// magnitudes, bin 0 = ~0 Hz.
    subcols: Vec<Vec<f32>>,
}

/// The skimmer view's own waterfall buffer.
///
/// Independent of the classic horizontal waterfall. The current slot is
/// accumulated as `SKIMMER_SUBCOLS` time sub-columns: each incoming FFT
/// column is added into the sub-column matching its position within the
/// 15 s slot. At the slot boundary the sub-column sums are averaged and the
/// finished [`SkimmerStripe`] is pushed; only the last [`SKIMMER_SLOTS_MAX`] are
/// kept. Retaining the sub-columns preserves the time axis a single averaged
/// stripe threw away.
struct SkimmerWaterfall {
    /// Completed slot stripes, oldest first.
    stripes: VecDeque<SkimmerStripe>,
    /// Per-sub-column running sums for the current (incomplete) slot.
    cur_sums: Vec<Vec<f32>>,
    /// Per-sub-column count of FFT columns folded in so far.
    cur_counts: Vec<u32>,
    /// Slot index the current accumulation belongs to.
    cur_slot: i64,
}

impl SkimmerWaterfall {
    fn new() -> Self {
        Self {
            stripes: VecDeque::with_capacity(SKIMMER_SLOTS_MAX + 1),
            cur_sums: vec![vec![0.0; SKIMMER_BINS]; SKIMMER_SUBCOLS],
            cur_counts: vec![0; SKIMMER_SUBCOLS],
            cur_slot: i64::MIN,
        }
    }

    /// Fold one FFT magnitude column (timestamped) into the current slot,
    /// into the sub-column matching its time position within the slot.
    /// Crossing a slot boundary flushes the previous slot into a stripe.
    fn push_column(&mut self, utc: DateTime<Utc>, mags: &[f32]) {
        let secs = utc.timestamp();
        let slot = secs.div_euclid(15);
        if slot != self.cur_slot {
            self.flush();
            self.cur_slot = slot;
        }
        // Fractional position 0..1 through the 15 s slot -> sub-column index.
        let into_slot = (secs.rem_euclid(15) as f64
            + utc.timestamp_subsec_millis() as f64 / 1000.0)
            / 15.0;
        let sub = ((into_slot * SKIMMER_SUBCOLS as f64) as usize)
            .min(SKIMMER_SUBCOLS - 1);
        let n = mags.len().min(SKIMMER_BINS);
        let dst = &mut self.cur_sums[sub];
        for i in 0..n {
            dst[i] += mags[i];
        }
        self.cur_counts[sub] += 1;
    }

    /// Finish the current slot: average each sub-column's sum and push the
    /// completed stripe. A sub-column that received no columns is left zero.
    fn flush(&mut self) {
        if self.cur_counts.iter().any(|&c| c > 0) && self.cur_slot != i64::MIN {
            let mut subcols: Vec<Vec<f32>> = Vec::with_capacity(SKIMMER_SUBCOLS);
            for s in 0..SKIMMER_SUBCOLS {
                let c = self.cur_counts[s];
                if c > 0 {
                    let inv = 1.0 / c as f32;
                    subcols.push(self.cur_sums[s].iter().map(|v| v * inv).collect());
                } else {
                    subcols.push(vec![0.0; SKIMMER_BINS]);
                }
            }
            self.stripes.push_back(SkimmerStripe {
                slot: self.cur_slot,
                subcols,
            });
            while self.stripes.len() > SKIMMER_SLOTS_MAX {
                self.stripes.pop_front();
            }
        }
        for sum in self.cur_sums.iter_mut() {
            for v in sum.iter_mut() {
                *v = 0.0;
            }
        }
        for c in self.cur_counts.iter_mut() {
            *c = 0;
        }
    }

    fn clear(&mut self) {
        self.stripes.clear();
        for sum in self.cur_sums.iter_mut() {
            for v in sum.iter_mut() {
                *v = 0.0;
            }
        }
        for c in self.cur_counts.iter_mut() {
            *c = 0;
        }
        self.cur_slot = i64::MIN;
    }
}

/// Live capture session: the threads and channels feeding the UI.
struct RunHandles {
    _audio: AudioHandle,
    pipe_rx: Receiver<PipelineEvent>,
    decode_rx: Receiver<Vec<Decode>>,
    in_rate: u32,
}

pub struct App {
    devices: Vec<(String, cpal::Device)>,
    selected: usize,
    dial_mhz: String,

    /// RF dial frequency in Hz, shared live with the decoder thread for
    /// PSK Reporter spotting.
    dial_hz: Arc<AtomicU64>,
    /// PSK Reporter upload enable, shared live with the decoder thread.
    pskr_enabled: Arc<AtomicBool>,
    /// UI mirror of `pskr_enabled` (the checkbox binds to this).
    pskr_ui: bool,
    /// Whether to capture soft-LLR evidence to the `periods` table. Read at
    /// Start to construct the store; the checkbox is disabled while running.
    capture_soft: bool,

    running: Option<RunHandles>,

    // Waterfall state.
    waterfall: VecDeque<Vec<Color32>>,
    /// UTC of each waterfall row, same order as `waterfall` (index 0 = newest).
    /// Used to place slot-boundary markers on true 15 s marks.
    wf_times: VecDeque<DateTime<Utc>>,
    wf_texture: Option<egui::TextureHandle>,
    wf_dirty: bool,
    noise_floor_db: f32,

    // Decode + status state.
    decodes: Vec<Decode>,
    slot_count: usize,
    level: f32,
    status: String,
    /// Which main view is showing — toggled live in the top bar.
    view_mode: ViewMode,
    /// The skimmer view's own frequency-vertical waterfall buffer.
    skimmer_wf: SkimmerWaterfall,
    /// How many recent slots the skimmer view draws (the buffer always
    /// retains `SKIMMER_SLOTS_MAX`; this just chooses how many to show).
    skimmer_slots: usize,
}

impl App {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let devices = audio::list_devices();
        Self {
            devices,
            selected: 0,
            dial_mhz: "144.174".to_string(),
            dial_hz: Arc::new(AtomicU64::new(144_174_000)),
            pskr_enabled: Arc::new(AtomicBool::new(true)),
            pskr_ui: true,
            capture_soft: true,
            running: None,
            waterfall: VecDeque::with_capacity(WF_HEIGHT),
            wf_times: VecDeque::with_capacity(WF_HEIGHT),
            wf_texture: None,
            wf_dirty: false,
            noise_floor_db: -50.0,
            decodes: Vec::new(),
            slot_count: 0,
            level: 0.0,
            status: "idle — pick the radio's USB codec input and press Start".into(),
            view_mode: ViewMode::Skimmer,
            skimmer_wf: SkimmerWaterfall::new(),
            skimmer_slots: 8,
        }
    }

    fn start(&mut self) {
        if self.devices.is_empty() {
            self.status = "no input devices found".into();
            return;
        }
        let device = self.devices[self.selected].1.clone();
        match audio::start(device) {
            Ok((raw_rx, in_rate, audio_handle)) => {
                let (pipe_rx, slot_rx) = pipeline::spawn(raw_rx, in_rate);
                // Open the capture database under ~/.ft8mon/. A failure here
                // (e.g. unwritable home) must not stop monitoring — log it
                // and run without persistence.
                let store = {
                    let dev = self.devices[self.selected].0.clone();
                    let mut db = dirs_home();
                    db.push(".ft8mon");
                    db.push("ft8mon.db");
                    match ft8mon::store::Store::open(
                        &db,
                        Some(&dev),
                        12_000,
                        self.capture_soft,
                    ) {
                        Ok(s) => Some(s),
                        Err(e) => {
                            eprintln!("[store] disabled — {e}");
                            None
                        }
                    }
                };
                let decode_rx = decoder::spawn_decoder(
                    slot_rx,
                    decoder::default_decoder(),
                    self.dial_hz.clone(),
                    self.pskr_enabled.clone(),
                    store,
                );
                self.running = Some(RunHandles {
                    _audio: audio_handle,
                    pipe_rx,
                    decode_rx,
                    in_rate,
                });
                self.waterfall.clear();
                self.wf_times.clear();
                self.skimmer_wf.clear();
                self.wf_dirty = true;
                self.slot_count = 0;
                self.status = format!(
                    "capturing — {} Hz codec -> 12 kHz",
                    in_rate
                );
            }
            Err(e) => {
                self.status = format!("start failed: {e}");
            }
        }
    }

    fn stop(&mut self) {
        // Dropping RunHandles stops the capture thread; the pipeline and
        // decoder threads then end on their own as the channels close.
        self.running = None;
        self.status = "stopped".into();
    }

    /// Convert one FFT magnitude column into a coloured waterfall row.
    fn push_waterfall(&mut self, utc: DateTime<Utc>, mags: &[f32]) {
        let db: Vec<f32> = mags
            .iter()
            .map(|m| 20.0 * (m + 1e-9).log10())
            .collect();

        // Track the noise floor via the 15th percentile of this column.
        let mut sorted = db.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let p15 = sorted[sorted.len() * 15 / 100];
        self.noise_floor_db = self.noise_floor_db * 0.95 + p15 * 0.05;

        let span = 45.0; // dB of dynamic range mapped into the colormap
        let row: Vec<Color32> = db
            .iter()
            .map(|&d| colormap(((d - self.noise_floor_db) / span).clamp(0.0, 1.0)))
            .collect();

        self.waterfall.push_front(row);
        self.wf_times.push_front(utc);
        while self.waterfall.len() > WF_HEIGHT {
            self.waterfall.pop_back();
            self.wf_times.pop_back();
        }
        self.wf_dirty = true;
    }

    /// Draw faint warm-grey lines across the waterfall at every UTC 15 s slot
    /// boundary (`:00 :15 :30 :45`), each tagged with its second-of-minute.
    ///
    /// Rows scroll downward — `wf_times[0]` is the newest row at the top of
    /// `rect`. A boundary is drawn between two adjacent rows whenever the
    /// 15 s slot index changes from one to the next, so the line tracks the
    /// real instant as it ages down the display.
    fn paint_slot_markers(&self, ui: &egui::Ui, rect: egui::Rect) {
        if self.wf_times.len() < 2 {
            return;
        }
        let painter = ui.painter_at(rect);
        let n = self.wf_times.len() as f32;
        let row_h = rect.height() / n;
        let line = Color32::from_rgba_unmultiplied(200, 200, 175, 70);
        let text = Color32::from_rgba_unmultiplied(210, 210, 185, 160);

        // Slot index of a timestamp: whole 15 s blocks since the epoch.
        let slot_of = |t: &DateTime<Utc>| t.timestamp() / 15;

        for i in 0..self.wf_times.len() - 1 {
            // Row i is newer (higher up) than row i+1.
            let newer = &self.wf_times[i];
            let older = &self.wf_times[i + 1];
            if slot_of(newer) == slot_of(older) {
                continue;
            }
            // Boundary sits between the two rows.
            let y = rect.top() + (i as f32 + 1.0) * row_h;
            painter.line_segment(
                [egui::pos2(rect.left(), y), egui::pos2(rect.right(), y)],
                egui::Stroke::new(1.0, line),
            );
            // Tag with the boundary second (the newer row's slot start).
            let secs = (slot_of(newer) * 15).rem_euclid(60);
            painter.text(
                egui::pos2(rect.left() + 4.0, y - 1.0),
                egui::Align2::LEFT_BOTTOM,
                format!(":{:02}", secs),
                egui::FontId::monospace(10.0),
                text,
            );
        }
    }

    fn refresh_texture(&mut self, ctx: &egui::Context) {
        if !self.wf_dirty || self.waterfall.is_empty() {
            return;
        }
        let h = self.waterfall.len();
        let mut pixels = Vec::with_capacity(WF_BINS * h);
        for row in &self.waterfall {
            pixels.extend_from_slice(row);
        }
        let image = egui::ColorImage {
            size: [WF_BINS, h],
            pixels,
        };
        match &mut self.wf_texture {
            Some(tex) => tex.set(image, egui::TextureOptions::NEAREST),
            None => {
                self.wf_texture = Some(ctx.load_texture(
                    "waterfall",
                    image,
                    egui::TextureOptions::NEAREST,
                ));
            }
        }
        self.wf_dirty = false;
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Drain background channels. Clone the receivers out first so the
        // borrow of `self.running` does not collide with `&mut self` calls.
        if let Some((pipe_rx, decode_rx)) = self
            .running
            .as_ref()
            .map(|r| (r.pipe_rx.clone(), r.decode_rx.clone()))
        {
            while let Ok(ev) = pipe_rx.try_recv() {
                match ev {
                    PipelineEvent::Spectrum { utc, mags } => {
                        self.push_waterfall(utc, &mags);
                        self.skimmer_wf.push_column(utc, &mags);
                    }
                    PipelineEvent::Level(l) => self.level = l,
                    PipelineEvent::SlotCaptured { .. } => self.slot_count += 1,
                }
            }
            while let Ok(mut batch) = decode_rx.try_recv() {
                self.decodes.append(&mut batch);
                if self.decodes.len() > MAX_DECODES {
                    let drop = self.decodes.len() - MAX_DECODES;
                    self.decodes.drain(..drop);
                }
            }
        }
        self.refresh_texture(ctx);

        let running = self.running.is_some();

        egui::TopBottomPanel::top("controls").show(ctx, |ui| {
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Input:");
                let current = self
                    .devices
                    .get(self.selected)
                    .map(|d| d.0.as_str())
                    .unwrap_or("<none>");
                egui::ComboBox::from_id_salt("device")
                    .selected_text(current)
                    .width(280.0)
                    .show_ui(ui, |ui| {
                        for (i, (name, _)) in self.devices.iter().enumerate() {
                            ui.selectable_value(&mut self.selected, i, name);
                        }
                    });

                ui.separator();
                ui.label("Dial MHz:");
                ui.add(
                    egui::TextEdit::singleline(&mut self.dial_mhz)
                        .desired_width(80.0),
                );
                // Keep the shared dial frequency current for PSK Reporter.
                if let Ok(mhz) = self.dial_mhz.trim().parse::<f64>() {
                    if mhz > 0.0 {
                        self.dial_hz
                            .store((mhz * 1e6).round() as u64, Ordering::Relaxed);
                    }
                }
                if ui.checkbox(&mut self.pskr_ui, "PSK Reporter").changed() {
                    self.pskr_enabled.store(self.pskr_ui, Ordering::Relaxed);
                }
                // Soft-LLR capture is fixed for a run — the store is built at
                // Start — so the control is disabled while capturing.
                ui.add_enabled(
                    self.running.is_none(),
                    egui::Checkbox::new(&mut self.capture_soft, "Capture soft"),
                );

                ui.separator();
                // View toggle — purely a render choice, free to switch any
                // time, capturing or not.
                ui.selectable_value(&mut self.view_mode, ViewMode::Classic, "Classic");
                ui.selectable_value(&mut self.view_mode, ViewMode::Skimmer, "Skimmer");
                // Slot-count selector — only relevant to the skimmer view.
                if self.view_mode == ViewMode::Skimmer {
                    ui.label("slots:");
                    for n in SKIMMER_SLOT_CHOICES {
                        ui.selectable_value(&mut self.skimmer_slots, n, n.to_string());
                    }
                }

                ui.separator();
                if running {
                    if ui.button("Stop").clicked() {
                        self.stop();
                    }
                } else if ui.button("Start").clicked() {
                    self.start();
                }
            });
            ui.add_space(2.0);
            ui.horizontal(|ui| {
                ui.label("Level:");
                ui.add(
                    egui::ProgressBar::new(self.level.clamp(0.0, 1.0))
                        .desired_width(140.0),
                );
                ui.separator();
                ui.label(format!("slots: {}", self.slot_count));
                ui.separator();
                ui.label(format!("decodes: {}", self.decodes.len()));
                if let Some(r) = &self.running {
                    ui.separator();
                    ui.label(format!("codec: {} Hz", r.in_rate));
                }
            });
            ui.add_space(2.0);
            ui.label(egui::RichText::new(&self.status).weak());
            ui.add_space(4.0);
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            match self.view_mode {
                ViewMode::Classic => {
                    ui.heading("Waterfall  (0 – 3 kHz audio passband)");
                    let wf_height = 300.0_f32;
                    if let Some(tex) = &self.wf_texture {
                        let width = ui.available_width();
                        let resp = ui.add(
                            egui::Image::new(egui::load::SizedTexture::new(
                                tex.id(),
                                egui::vec2(width, wf_height),
                            ))
                            .maintain_aspect_ratio(false),
                        );
                        self.paint_slot_markers(ui, resp.rect);
                    } else {
                        ui.allocate_space(egui::vec2(ui.available_width(), wf_height));
                        ui.weak("waterfall starts once capture is running");
                    }

                    ui.separator();
                    ui.heading("Decodes");
                    decode_table(ui, &self.decodes);
                }
                ViewMode::Skimmer => {
                    skimmer_view(ui, &self.decodes, &self.skimmer_wf, self.skimmer_slots);
                }
            }
        });

        // Keep the waterfall live.
        ctx.request_repaint_after(std::time::Duration::from_millis(50));
    }
}

/// Render the decode list, newest first.
fn decode_table(ui: &mut egui::Ui, decodes: &[Decode]) {
    use egui_extras::{Column, TableBuilder};

    TableBuilder::new(ui)
        .striped(true)
        .resizable(true)
        .column(Column::auto().at_least(70.0)) // UTC
        .column(Column::auto().at_least(44.0)) // dB
        .column(Column::auto().at_least(44.0)) // DT
        .column(Column::auto().at_least(64.0)) // Freq
        .column(Column::exact(250.0)) // Message — fixed; fits the widest
                                      // standard message and hashed-call form
        .column(Column::remainder()) // Src
        .header(20.0, |mut header| {
            for label in ["UTC", "dB", "DT", "Freq", "Message", "Src"] {
                header.col(|ui| {
                    ui.strong(label);
                });
            }
        })
        .body(|mut body| {
            for d in decodes.iter().rev() {
                body.row(18.0, |mut row| {
                    row.col(|ui| {
                        ui.monospace(d.utc.format("%H:%M:%S").to_string());
                    });
                    row.col(|ui| {
                        ui.monospace(format!("{:+}", d.snr_db));
                    });
                    row.col(|ui| {
                        ui.monospace(format!("{:+.1}", d.dt));
                    });
                    row.col(|ui| {
                        ui.monospace(format!("{:.0}", d.freq_hz));
                    });
                    row.col(|ui| {
                        ui.label(&d.message);
                    });
                    row.col(|ui| {
                        let txt = d.source.badge();
                        let color = match d.source {
                            DecodeSource::SingleShot => Color32::GRAY,
                            DecodeSource::Accumulated(_) => {
                                Color32::from_rgb(80, 200, 120)
                            }
                            DecodeSource::Both(_) => {
                                Color32::from_rgb(230, 200, 60)
                            }
                        };
                        ui.colored_label(color, txt);
                    });
                });
            }
        });
}

/// Experimental band-activity ("skimmer") view.
///
/// A frequency-vertical waterfall: the audio passband (0 Hz at the bottom,
/// `SKIMMER_MAX_HZ` at the top) runs up the screen, and the last
/// `SKIMMER_SLOTS` completed 15 s slots are drawn as side-by-side spectral
/// stripes with the newest at the right. The skimmer keeps its own waterfall
/// buffer ([`SkimmerWaterfall`]) — one averaged stripe per slot — so it is
/// fully self-contained.
///
/// Decode dots are painted over the waterfall at each decode's exact
/// (slot, audio-frequency) position. Decodes are grouped into tracks by
/// callsign and joined by a thin line, so a station repeating across slots
/// is a horizontal run of dots — and a frequency change shows as a visible
/// step. Each track carries one persistent label in the right margin (latest
/// message, de-collided vertically), so every dot has an identity, not just
/// the newest slot's. Accumulation decodes are ringed and their labels
/// greened.
fn skimmer_view(
    ui: &mut egui::Ui,
    decodes: &[Decode],
    wf: &SkimmerWaterfall,
    display_slots: usize,
) {
    ui.heading("Band activity");
    ui.add_space(4.0);

    if wf.stripes.is_empty() {
        ui.weak("band activity appears here once the first slot completes");
        return;
    }

    // Group decodes by slot index so each can be drawn over its stripe.
    use std::collections::BTreeMap;
    let mut dec_by_slot: BTreeMap<i64, Vec<&Decode>> = BTreeMap::new();
    for d in decodes {
        dec_by_slot
            .entry(d.utc.timestamp().div_euclid(15))
            .or_default()
            .push(d);
    }

    // The buffer retains up to SKIMMER_SLOTS_MAX stripes; draw only the most
    // recent `display_slots` of them. Switching the count is therefore
    // instant and lossless — the history is always there in the buffer.
    let show = display_slots.min(wf.stripes.len()).max(1);
    let visible: Vec<&SkimmerStripe> =
        wf.stripes.iter().rev().take(show).rev().collect();

    // Reserve the canvas. The waterfall grid sits on the left, the message
    // margin fills the rest; with more slots the grid widens and the margin
    // narrows accordingly.
    let avail = ui.available_size();
    let canvas_h = avail.y.max(360.0);
    let (rect, _resp) =
        ui.allocate_exact_size(egui::vec2(avail.x, canvas_h), egui::Sense::hover());
    let painter = ui.painter_at(rect);

    let bg = Color32::from_rgb(12, 14, 22);
    painter.rect_filled(rect, 0.0, bg);

    // Layout: frequency-label gutter, then the waterfall grid (32 px per
    // slot), then the message margin taking whatever width is left.
    let freq_gutter = 48.0;
    let stripe_w = 32.0_f32;
    let grid_w = stripe_w * visible.len() as f32;
    let grid = egui::Rect::from_min_max(
        egui::pos2(rect.left() + freq_gutter, rect.top() + 4.0),
        egui::pos2(rect.left() + freq_gutter + grid_w, rect.bottom() - 4.0),
    );

    // Frequency <-> y. Bin 0 (~0 Hz) sits at the bottom.
    let freq_to_y = |hz: f32| {
        let frac = (hz / SKIMMER_MAX_HZ).clamp(0.0, 1.0);
        grid.bottom() - frac * grid.height()
    };

    // --- waterfall stripes ------------------------------------------------
    // Each slot is drawn as SKIMMER_SUBCOLS thin time sub-columns, oldest at
    // the slot's left edge. A steady signal therefore reads as a continuous
    // horizontal trace across a slot's sub-columns; noise flickers between
    // them — the WSJT-X waterfall model, restored by keeping the time axis.
    //
    // Gain is data-measured: the colour range is taken from percentiles of
    // *all the magnitudes actually on screen this frame*, so the waterfall
    // tracks the real signal range and can never collapse to flat dark (the
    // failure mode of a fixed assumed floor).
    let rows = grid.height().round().max(1.0) as usize;

    // Gather every visible sub-column's dB values to set the colour range.
    let mut all_db: Vec<f32> = Vec::new();
    for stripe in &visible {
        for sc in &stripe.subcols {
            for &m in sc {
                all_db.push(20.0 * (m + 1e-9).log10());
            }
        }
    }
    let (lo, hi) = if all_db.is_empty() {
        (-50.0, 0.0)
    } else {
        all_db.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let p = |frac: f32| all_db[((all_db.len() - 1) as f32 * frac) as usize];
        // 30th percentile = noise floor, 99.5th = brightest — a touch of
        // headroom so a single hot bin does not wash the scale out.
        let lo = p(0.30);
        (lo, p(0.995).max(lo + 6.0))
    };
    let span = hi - lo;

    let sub_w = stripe_w / SKIMMER_SUBCOLS as f32;
    for (col, stripe) in visible.iter().enumerate() {
        let slot_x0 = grid.left() + col as f32 * stripe_w;
        for (s, sc) in stripe.subcols.iter().enumerate() {
            let db: Vec<f32> =
                sc.iter().map(|m| 20.0 * (m + 1e-9).log10()).collect();
            let x0 = slot_x0 + s as f32 * sub_w;
            for r in 0..rows {
                // Row 0 is the top of the grid = SKIMMER_MAX_HZ.
                let frac = 1.0 - r as f32 / rows as f32;
                let bin = ((frac * (SKIMMER_BINS - 1) as f32).round() as usize)
                    .min(SKIMMER_BINS - 1);
                let t = ((db[bin] - lo) / span).clamp(0.0, 1.0);
                let y = grid.top() + r as f32;
                painter.rect_filled(
                    egui::Rect::from_min_max(
                        egui::pos2(x0, y),
                        egui::pos2(x0 + sub_w, y + 1.0),
                    ),
                    0.0,
                    colormap(t),
                );
            }
        }
    }

    // --- frequency grid lines + labels every 500 Hz ----------------------
    let label_col = Color32::from_rgb(120, 128, 145);
    let line_col = Color32::from_rgba_unmultiplied(200, 200, 175, 60);
    let mut hz = 0.0;
    while hz <= SKIMMER_MAX_HZ {
        let y = freq_to_y(hz);
        painter.line_segment(
            [egui::pos2(grid.left(), y), egui::pos2(grid.right(), y)],
            egui::Stroke::new(1.0, line_col),
        );
        painter.text(
            egui::pos2(rect.left() + 4.0, y),
            egui::Align2::LEFT_CENTER,
            format!("{:.0}", hz),
            egui::FontId::monospace(10.0),
            label_col,
        );
        hz += 500.0;
    }

    // Slot index -> the centre x of its stripe column, for the visible set.
    let oldest = visible.first().unwrap().slot;
    let slot_to_x = |slot: i64| {
        // Visible stripes are packed left-to-right; find the column.
        visible
            .iter()
            .position(|s| s.slot == slot)
            .map(|c| grid.left() + (c as f32 + 0.5) * stripe_w)
    };

    // --- group decodes into tracks ---------------------------------------
    // A track is one station followed across slots, keyed by the *sender's*
    // callsign — the shared rule used for PSK Reporter spotting, so the two
    // never disagree on who sent a message. Keying on the sender means a
    // QSO between two stations is correctly two separate tracks (each steady
    // at its own frequency), not one track zig-zagging between them. A
    // message with no recognisable sender is grouped by its raw text.
    let track_key = |msg: &str| -> String {
        ft8mon::pskreporter::sender_callsign(msg)
            .unwrap_or_else(|| msg.to_string())
    };
    // call -> decodes for that call, kept in slot order.
    use std::collections::HashMap;
    let mut tracks: HashMap<String, Vec<&Decode>> = HashMap::new();
    for (&slot, ds) in &dec_by_slot {
        if slot < oldest {
            continue;
        }
        for d in ds {
            tracks.entry(track_key(&d.message)).or_default().push(d);
        }
    }
    for v in tracks.values_mut() {
        v.sort_by_key(|d| d.utc.timestamp());
    }

    // --- decode dots ------------------------------------------------------
    // Decodes are drawn as hollow white rings, not filled discs: the signal
    // shows through, so the mark never competes with the waterfall colour —
    // no fill hue can clash with the navy-amber colormap. An accumulation
    // decode gets a second, outer ring.
    for ds in tracks.values() {
        // Connecting line along the track, so a frequency change or drift
        // reads as a continuous path. Drawn as a dark halo with a lighter
        // core on top, so it stays visible over both the dark noise floor
        // and the bright signal blobs.
        for pair in ds.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            if let (Some(xa), Some(xb)) = (
                slot_to_x(a.utc.timestamp().div_euclid(15)),
                slot_to_x(b.utc.timestamp().div_euclid(15)),
            ) {
                let p0 = egui::pos2(xa, freq_to_y(a.freq_hz));
                let p1 = egui::pos2(xb, freq_to_y(b.freq_hz));
            
                painter.line_segment(
                    [p0, p1],
                    egui::Stroke::new(1.0, Color32::from_rgb(180, 186, 200)),
                );
            }
        }
        for d in ds {
            let Some(x) = slot_to_x(d.utc.timestamp().div_euclid(15)) else {
                continue;
            };
            let y = freq_to_y(d.freq_hz);
            let accumulated = !matches!(d.source, DecodeSource::SingleShot);
            let r = if accumulated { 4.0 } else { 3.0 };
            // Filled neutral-grey dot with a dark rim: the grey has no hue to
            // clash with the colormap, the rim keeps it visible on bright
            // signal blobs. An accumulation decode gets an outer ring.
            painter.circle_filled(
                egui::pos2(x, y),
                r + 1.0,
                Color32::from_rgb(15, 17, 24),
            );
            painter.circle_filled(
                egui::pos2(x, y),
                r,
                Color32::from_rgb(165, 170, 178),
            );
            if accumulated {
                painter.circle_stroke(
                    egui::pos2(x, y),
                    r + 3.0,
                    egui::Stroke::new(1.5, Color32::from_rgb(165, 170, 178)),
                );
            }
        }
    }

    // --- one persistent label per track ----------------------------------
    // Each track gets a single label in the right margin, anchored to its
    // most recent decode and joined to it by a leader line. Labels are
    // de-collided vertically so a busy band stays legible. The label shows
    // the track's latest message — if a station's message changes, the
    // label follows the change.
    struct TrackLabel<'a> {
        latest: &'a Decode,
        want_y: f32,
    }
    let mut labels: Vec<TrackLabel> = tracks
        .values()
        .filter_map(|ds| {
            ds.last().map(|d| TrackLabel {
                latest: d,
                want_y: freq_to_y(d.freq_hz),
            })
        })
        .collect();
    // Sort by desired y and push apart so text rows do not overlap.
    labels.sort_by(|a, b| a.want_y.partial_cmp(&b.want_y).unwrap());
    let row_h = 14.0_f32;
    let mut placed_y: Vec<f32> = Vec::with_capacity(labels.len());
    let mut last_y = f32::MIN;
    for l in &labels {
        let y = l.want_y.max(last_y + row_h);
        placed_y.push(y);
        last_y = y;
    }
    let label_x = grid.right() + 8.0;
    for (l, &y) in labels.iter().zip(&placed_y) {
        let d = l.latest;
        let anchor_y = freq_to_y(d.freq_hz);
        // Leader: from the spectrum edge across to the (de-collided) label
        // row. Brighter and thicker than before — the old thin dark-grey
        // line was barely visible against the panel background.
        painter.line_segment(
            [
                egui::pos2(grid.right(), anchor_y),
                egui::pos2(label_x, y),
            ],
            egui::Stroke::new(1.6, Color32::from_rgb(140, 148, 168)),
        );
        let accumulated = !matches!(d.source, DecodeSource::SingleShot);
        let col = if accumulated {
            Color32::from_rgb(120, 230, 150)
        } else {
            Color32::from_rgb(220, 224, 235)
        };
        painter.text(
            egui::pos2(label_x + 4.0, y),
            egui::Align2::LEFT_CENTER,
            format!("{}   {:.0} Hz  {:+} dB", d.message, d.freq_hz, d.snr_db),
            egui::FontId::proportional(11.0),
            col,
        );
    }

    ui.add_space(2.0);
    ui.weak(format!(
        "{} of {} slot(s) shown \u{00b7} {} track(s) \u{00b7} \
         green ring/label = accumulation decode",
        visible.len(),
        wf.stripes.len(),
        tracks.len(),
    ));
}

/// Map a normalised intensity (0..1) to a waterfall colour.
fn colormap(t: f32) -> Color32 {
    let t = t.clamp(0.0, 1.0);
    // dark navy -> blue -> teal -> amber -> near-white
    let stops: [(f32, (i32, i32, i32)); 5] = [
        (0.00, (4, 6, 28)),
        (0.35, (22, 42, 140)),
        (0.60, (30, 158, 168)),
        (0.80, (232, 200, 44)),
        (1.00, (255, 255, 240)),
    ];
    for w in stops.windows(2) {
        let (t0, c0) = w[0];
        let (t1, c1) = w[1];
        if t <= t1 {
            let f = ((t - t0) / (t1 - t0)).clamp(0.0, 1.0);
            let lerp = |a: i32, b: i32| (a as f32 + (b as f32 - a as f32) * f) as u8;
            return Color32::from_rgb(
                lerp(c0.0, c1.0),
                lerp(c0.1, c1.1),
                lerp(c0.2, c1.2),
            );
        }
    }
    Color32::WHITE
}

/// Home directory, for locating `~/.ft8mon/`. Falls back to the current
/// directory if `$HOME` is unset, so the store still opens *somewhere*.
fn dirs_home() -> PathBuf {
    std::env::var_os("HOME")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."))
}
