//! `ft8mon-backfill` — recompute the `periods.decoded` flag in an existing
//! capture database.
//!
//! Early capture builds set `periods.decoded` by matching a candidate's
//! sync-stage `(freq, dt)` against the post-decode refined coordinates of the
//! single-shot decodes. FT8's DT refinement shifts the estimate by more than
//! the match tolerance, so the flag was always written `0`. The fix is to
//! decide `decoded` by actually running LDPC on the period's own stored LLR
//! vector — and because that is a pure function of the blob, databases
//! captured under the old logic can be repaired in place rather than
//! re-recorded.
//!
//! Usage:
//!     ft8mon-backfill [PATH]
//!
//! PATH defaults to `~/.ft8mon/ft8mon.db`. The database is updated in place;
//! take a copy first if you want a before/after comparison.

use std::path::PathBuf;

use anyhow::{Context, Result};
use ft8mon::mshv::MshvDecoder;
use ft8mon::store::LLR_LEN;
use rusqlite::Connection;

fn main() -> Result<()> {
    let path = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(default_db_path);

    println!("[backfill] opening {}", path.display());
    let conn = Connection::open(&path)
        .with_context(|| format!("open database {}", path.display()))?;

    // One decoder instance — only its LDPC path is exercised.
    let decoder = MshvDecoder::new();

    // Pull every period's id and LLR blob.
    let rows: Vec<(i64, Vec<u8>)> = {
        let mut stmt = conn
            .prepare("SELECT id, llr FROM periods ORDER BY id")
            .context("prepare period scan")?;
        let mapped = stmt
            .query_map([], |r| Ok((r.get::<_, i64>(0)?, r.get::<_, Vec<u8>>(1)?)))
            .context("query periods")?;
        let mut v = Vec::new();
        for row in mapped {
            v.push(row.context("read period row")?);
        }
        v
    };
    println!("[backfill] {} period row(s) to examine", rows.len());

    let mut decoded_count = 0usize;
    let mut changed = 0usize;
    let mut skipped = 0usize;

    // Wrap the rewrite in a transaction — all-or-nothing, and far faster.
    conn.execute_batch("BEGIN")?;
    for (id, blob) in &rows {
        // Each blob is LLR_LEN little-endian f32 values.
        if blob.len() != LLR_LEN * 4 {
            eprintln!(
                "[backfill] period {id}: unexpected blob length {} — skipped",
                blob.len()
            );
            skipped += 1;
            continue;
        }
        let llr: Vec<f64> = blob
            .chunks_exact(4)
            .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]) as f64)
            .collect();

        let decoded = decoder.llr_decodes(&llr);
        if decoded {
            decoded_count += 1;
        }

        // Read the stored flag so we only count genuine corrections.
        let prev: i64 = conn.query_row(
            "SELECT decoded FROM periods WHERE id = ?1",
            [id],
            |r| r.get(0),
        )?;
        let new = decoded as i64;
        if prev != new {
            conn.execute(
                "UPDATE periods SET decoded = ?1 WHERE id = ?2",
                rusqlite::params![new, id],
            )?;
            changed += 1;
        }
    }
    conn.execute_batch("COMMIT")?;

    println!(
        "[backfill] done — {decoded_count} period(s) decode single-shot, \
         {changed} flag(s) corrected, {skipped} skipped"
    );
    Ok(())
}

/// `~/.ft8mon/ft8mon.db`, matching the path the app uses.
fn default_db_path() -> PathBuf {
    let mut p = std::env::var_os("HOME")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."));
    p.push(".ft8mon");
    p.push("ft8mon.db");
    p
}
