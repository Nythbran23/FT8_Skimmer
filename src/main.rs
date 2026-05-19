//! ft8mon entry point.
//!
//! With the default `gui` feature this launches the egui application. Built
//! `--no-default-features` it is a headless stub — that build exists so the
//! core library can be type-checked and unit-tested on toolchains older than
//! egui's MSRV.

#[cfg(feature = "gui")]
mod app;

#[cfg(feature = "gui")]
fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([1180.0, 780.0])
            .with_min_inner_size([820.0, 560.0])
            .with_title("FT8 Skimmer — FT8 multi-decode RX monitor"),
        ..Default::default()
    };
    eframe::run_native(
        "ft8mon",
        native_options,
        Box::new(|cc| Ok(Box::new(app::App::new(cc)))),
    )
}

#[cfg(not(feature = "gui"))]
fn main() {
    eprintln!(
        "ft8 skimmer built without the GUI. Rebuild with the default features:\n\
         \n    cargo run --release\n"
    );
}
