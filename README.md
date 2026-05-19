# FT8 Skimmer

A receive-only FT8 band monitor: multi-decode, cross-period soft-LLR
accumulation, a vertical-spectrum band-activity view, and PSK Reporter
spotting.

FT8 Skimmer is built around MSHV's FT8 decoder, wrapped over a C ABI. Beyond
ordinary single-shot decoding it accumulates the per-candidate soft-LLR
vectors across periods and re-runs the LDPC decoder on the sum, recovering
decodes the single-shot path misses — the gain follows the usual
`10·log10(N)` combining law.

## Views

- **Classic** — a conventional horizontal waterfall plus a decode table.
- **Skimmer** — frequency on the vertical axis, recent 15 s slots as
  side-by-side spectral stripes, each decode a mark on its trace with the
  station's callsign labelled alongside. A station is followed across slots
  as a single track, so a frequency change or drift reads as a continuous
  path. Selectable 8 / 16 / 24 slots.

## Building

The real decoder links MSHV's de-Qt'd `DecoderFt8` through the C shim in
`cpp/`, compiled by `build.rs`. It needs the patched MSHV source tree, located
via the `MSHV_SRC` environment variable:

```
MSHV_SRC=/path/to/mshv-src/src cargo build --release --features mshv
```

Without `--features mshv` the project builds with a stub decoder — useful for
type-checking and running the unit tests on toolchains older than egui's MSRV:

```
cargo test --no-default-features
```

## Capture and analysis

With "Capture soft" enabled, every decode and every period's soft-LLR evidence
is written to a SQLite database at `~/.ft8mon/ft8mon.db` (WAL mode, queryable
live). Accumulation decodes are linked to their exact contributing periods via
the `accum_periods` table.

- `ft8mon-backfill` — recomputes the `periods.decoded` flag in an existing
  database.
- `ft8mon_analyse.py` — read-only accumulation-gain analysis of the database.

## Status

Experimental, under active development. Receive only — there is no transmit
capability.

## Licence

GPL-3.0. The FT8 decoder links MSHV, which is GPL-licensed.
