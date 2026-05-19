//! build.rs — compiles the MSHV FT8 decoder + the C-ABI shim into one static
//! library, linked into ft8mon when the `mshv` feature is enabled.
//!
//! The feature is OFF by default, so a plain `cargo build` needs neither the
//! MSHV source nor Qt. To build the real decoder:
//!
//!     cargo build --features mshv
//!
//! Requirements (only with `--features mshv`):
//!   * MSHV source, patched with `mshv_ffi.patch`, located via the MSHV_SRC
//!     environment variable — point it at `.../MSHV_2765/src`.
//!   * Qt5 (Core + dev headers) and FFTW3:
//!       Linux : apt install qtbase5-dev libfftw3-dev
//!       macOS : brew install qt@5 fftw
//!
//! Only **Qt5Core** is linked — the decoder is de-Qt'd. QtGui/QtWidgets
//! headers are pulled in solely to parse one stray include in MSHV's
//! monolithic header tree; nothing from them is linked.

use std::env;
use std::path::PathBuf;

// The exact MSHV translation units the FT8 RX decode path needs, relative
// to MSHV_SRC. Established by walking the link closure of decoderft8.cpp.
const MSHV_SOURCES: &[&str] = &[
    "HvDecoderMs/decoderft8.cpp",
    "HvDecoderMs/decoderft8var.cpp",
    "HvDecoderMs/decoderpom.cpp",
    "HvMsPlayer/libsound/genpom.cpp",
    "HvMsPlayer/libsound/HvGenFt8/gen_ft8.cpp",
    "HvMsPlayer/libsound/HvPackUnpackMsg/pack_unpack_msg77.cpp",
];

fn main() {
    // Feature gate — do nothing unless `mshv` is enabled.
    if env::var_os("CARGO_FEATURE_MSHV").is_none() {
        return;
    }

    println!("cargo:rerun-if-changed=cpp/ft8_shim.cpp");
    println!("cargo:rerun-if-changed=cpp/ft8_shim.h");
    println!("cargo:rerun-if-changed=cpp/c99_complex_shim.h");
    println!("cargo:rerun-if-env-changed=MSHV_SRC");

    let mshv = PathBuf::from(env::var("MSHV_SRC").unwrap_or_else(|_| {
        panic!(
            "\n\nMSHV_SRC is not set. The `mshv` feature needs the patched \
             MSHV source tree.\n\
             \n  1. Unzip the MSHV source.\
             \n  2. Apply the patch:\
             \n       cd MSHV_2765/src && patch -p1 < /path/to/ft8mon/mshv_ffi.patch\
             \n  3. Build:\
             \n       MSHV_SRC=/abs/path/to/MSHV_2765/src cargo build --features mshv\n"
        )
    }));
    assert!(
        mshv.join("HvDecoderMs/decoderft8.cpp").is_file(),
        "MSHV_SRC = {} does not look like an MSHV src/ directory \
         (HvDecoderMs/decoderft8.cpp not found)",
        mshv.display()
    );

    // --- make Homebrew's keg-only Qt visible to pkg-config on macOS -------
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let mut qt_framework_dir: Option<String> = None;
    if target_os == "macos" {
        for pkg in ["qt@5", "fftw"] {
            if let Some(prefix) = brew_prefix(pkg) {
                let pc = format!("{prefix}/lib/pkgconfig");
                let cur = env::var("PKG_CONFIG_PATH").unwrap_or_default();
                env::set_var(
                    "PKG_CONFIG_PATH",
                    if cur.is_empty() { pc } else { format!("{pc}:{cur}") },
                );
                if pkg == "qt@5" {
                    // Qt ships as macOS frameworks; this dir holds
                    // QtCore.framework etc. so the compiler's framework
                    // lookup resolves the <QtCore/...> includes that Qt's
                    // own headers use internally.
                    qt_framework_dir = Some(format!("{prefix}/lib"));
                }
            }
        }
    }

    // Harvest Qt include paths (Core+Gui+Widgets) WITHOUT emitting link
    // directives — needed only to parse MSHV's headers.
    let qt_includes = pkg_config::Config::new()
        .cargo_metadata(false)
        .probe("Qt5Widgets")
        .map(|lib| lib.include_paths)
        .unwrap_or_default();

    // --- compile the decoder + shim into one static library --------------
    let mut build = cc::Build::new();
    build
        .cpp(true)
        .flag("-std=gnu++17")
        .flag_if_supported("-w") // MSHV is warning-noisy; keep cargo output clean
        .define("_LINUX_", None) // POSIX code path (harmless / correct on macOS too)
        .include(mshv.join("HvDecoderMs"))
        .include("cpp");
    for inc in &qt_includes {
        build.include(inc);
    }
    // macOS: add the Qt framework search path so the <QtCore/qchar.h>-style
    // includes inside Qt's own headers resolve via framework lookup.
    if let Some(fwdir) = &qt_framework_dir {
        build.flag(format!("-F{fwdir}"));
    }
    // macOS: force-include the C99 <complex.h> shim ahead of every TU —
    // Apple's libc++ omits the C99 complex API that MSHV's code relies on.
    if target_os == "macos" {
        let manifest = env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
        build.flag("-include");
        build.flag(format!("{manifest}/cpp/c99_complex_shim.h"));
    }
    for src in MSHV_SOURCES {
        let path = mshv.join(src);
        assert!(path.is_file(), "missing MSHV source: {}", path.display());
        build.file(path);
    }
    build.file("cpp/ft8_shim.cpp");
    build.compile("ft8mshv"); // -> libft8mshv.a + its link directive

    // --- link Qt5Core + FFTW3 -------------------------------------------
    pkg_config::Config::new().probe("Qt5Core").expect(
        "Qt5Core not found via pkg-config \
         (Linux: apt install qtbase5-dev — macOS: brew install qt@5)",
    );
    if pkg_config::Config::new().probe("fftw3").is_err() {
        // pkg-config miss — fall back to a bare link directive.
        println!("cargo:rustc-link-lib=fftw3");
    }
}

/// `brew --prefix <pkg>`, if Homebrew is installed.
fn brew_prefix(pkg: &str) -> Option<String> {
    let out = std::process::Command::new("brew")
        .args(["--prefix", pkg])
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    let s = String::from_utf8(out.stdout).ok()?.trim().to_string();
    if s.is_empty() {
        None
    } else {
        Some(s)
    }
}
