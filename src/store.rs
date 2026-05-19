//! SQLite persistence for ft8mon.
//!
//! Captures both the *decoded* results and the *soft* evidence behind them,
//! so a monitoring run can be analysed offline — per-station SNR over time,
//! decode rates, and in particular the real-world gain of cross-period
//! accumulation measured against the 10·log10(N) prediction.
//!
//! The database follows the same shape as the FSK441Plus and MSK144plus
//! stores: a `Store` wrapping a `Connection`, opened in WAL mode so the file
//! can be queried with `sqlite3` or DBeaver while ft8mon is still writing.
//!
//! Three tables:
//!
//!   sessions  — one row per app launch.
//!   decodes   — one row per emitted decode (single-shot or accumulated).
//!   periods   — one row per 15 s period the MSHV LLR callback fired, each
//!               carrying the 174-element soft-LLR vector as a BLOB. This is
//!               the raw material for the accumulation study, and crucially
//!               it records the *negative* cases too: periods that never
//!               produced a decode by any route.
//!
//! The `periods` table is the bulky one — on a busy band, dozens of LLR
//! vectors per slot. Each LLR is stored as `f32` (174 × 4 = 696 bytes/row):
//! accumulation gain is set by LLR sign and rough magnitude, so the extra
//! precision of `f64` would buy nothing and double the file. Capture of the
//! `periods` table is gated by a flag on `Store` so a lean run can skip it.

use std::path::Path;

use anyhow::{Context, Result};
use chrono::{DateTime, Utc};
use rusqlite::{params, Connection};

use crate::decoder::{Decode, DecodeSource};

/// Number of LLR values in an FT8 codeword — 91 message+CRC bits expand to
/// 174 transmitted bits under the (174,91) LDPC code.
pub const LLR_LEN: usize = 174;

/// A 15 s period's soft evidence, as handed up by the MSHV LLR callback.
#[derive(Clone, Debug)]
pub struct PeriodSoft {
    /// UTC of the slot this period belongs to.
    pub utc: DateTime<Utc>,
    /// Candidate audio frequency, Hz.
    pub freq_hz: f32,
    /// Candidate time offset, seconds.
    pub dt: f32,
    /// MSHV sync metric for the candidate.
    pub sync: i32,
    /// The 174-element scaled soft-LLR vector.
    pub llr: Vec<f64>,
    /// True if this period produced a CRC-valid decode single-shot.
    pub decoded: bool,
}

/// SQLite-backed capture store. One per app run.
pub struct Store {
    conn: Connection,
    session_id: i64,
    /// When false, `record_period` is a no-op — a lean run that keeps the
    /// `decodes` table but skips the bulky soft-LLR capture.
    capture_soft: bool,
}

impl Store {
    /// Open (creating if needed) the database at `path` and start a session.
    ///
    /// `capture_soft` gates the `periods` table: pass `true` to capture the
    /// soft-LLR evidence, `false` to log decodes only.
    pub fn open(
        path: &Path,
        device: Option<&str>,
        sample_rate: u32,
        capture_soft: bool,
    ) -> Result<Self> {
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)
                .with_context(|| format!("create DB directory {}", dir.display()))?;
        }
        let conn = Connection::open(path)
            .with_context(|| format!("open database {}", path.display()))?;

        conn.execute_batch(
            "
            PRAGMA journal_mode = WAL;
            PRAGMA synchronous  = NORMAL;
            PRAGMA foreign_keys = ON;

            CREATE TABLE IF NOT EXISTS sessions (
                id          INTEGER PRIMARY KEY AUTOINCREMENT,
                started_at  TEXT    NOT NULL,
                ended_at    TEXT,
                sample_rate INTEGER NOT NULL,
                device      TEXT,
                notes       TEXT
            );

            CREATE TABLE IF NOT EXISTS decodes (
                id           INTEGER PRIMARY KEY AUTOINCREMENT,
                session_id   INTEGER NOT NULL REFERENCES sessions(id),
                utc          TEXT    NOT NULL,
                snr_db       INTEGER NOT NULL,
                dt           REAL    NOT NULL,
                freq_hz      REAL    NOT NULL,
                message      TEXT    NOT NULL,
                -- 'single' | 'accum' | 'both'
                source       TEXT    NOT NULL,
                -- combined period count for accum/both, else NULL
                depth        INTEGER
            );

            CREATE INDEX IF NOT EXISTS idx_decodes_session
                ON decodes(session_id, utc);
            CREATE INDEX IF NOT EXISTS idx_decodes_message
                ON decodes(message);
            CREATE INDEX IF NOT EXISTS idx_decodes_source
                ON decodes(source);

            CREATE TABLE IF NOT EXISTS periods (
                id           INTEGER PRIMARY KEY AUTOINCREMENT,
                session_id   INTEGER NOT NULL REFERENCES sessions(id),
                utc          TEXT    NOT NULL,
                freq_hz      REAL    NOT NULL,
                dt           REAL    NOT NULL,
                sync         INTEGER NOT NULL,
                -- 1 if this period decoded single-shot, else 0
                decoded      INTEGER NOT NULL,
                -- 174 little-endian f32 LLR values (696 bytes)
                llr          BLOB    NOT NULL
            );

            CREATE INDEX IF NOT EXISTS idx_periods_session
                ON periods(session_id, utc);
            CREATE INDEX IF NOT EXISTS idx_periods_decoded
                ON periods(decoded);

            -- Links an accumulation decode to the exact periods whose
            -- soft-LLR vectors were summed to produce it. One row per
            -- (decode, contributing period) pair. This is what makes the
            -- accumulation-gain study rigorous: the contributing periods are
            -- recorded at capture time, not reconstructed afterwards by
            -- proximity guessing.
            CREATE TABLE IF NOT EXISTS accum_periods (
                decode_id  INTEGER NOT NULL REFERENCES decodes(id),
                period_id  INTEGER NOT NULL REFERENCES periods(id),
                PRIMARY KEY (decode_id, period_id)
            );

            CREATE INDEX IF NOT EXISTS idx_accum_decode
                ON accum_periods(decode_id);
            CREATE INDEX IF NOT EXISTS idx_accum_period
                ON accum_periods(period_id);
            ",
        )
        .context("initialise database schema")?;

        conn.execute(
            "INSERT INTO sessions (started_at, sample_rate, device) VALUES (?1, ?2, ?3)",
            params![Utc::now().to_rfc3339(), sample_rate, device],
        )
        .context("insert session row")?;
        let session_id = conn.last_insert_rowid();

        Ok(Store {
            conn,
            session_id,
            capture_soft,
        })
    }

    /// The current session's row id.
    pub fn session_id(&self) -> i64 {
        self.session_id
    }

    /// Whether soft-LLR capture is enabled for this run.
    pub fn capture_soft(&self) -> bool {
        self.capture_soft
    }

    /// Record one emitted decode; returns its row id (for `accum_periods`
    /// links) or `None` on error. Errors are logged, never propagated — a
    /// database hiccup must not take down the decode pipeline.
    pub fn record_decode(&self, d: &Decode) -> Option<i64> {
        let (source, depth): (&str, Option<i64>) = match d.source {
            DecodeSource::SingleShot => ("single", None),
            DecodeSource::Accumulated(n) => ("accum", Some(n as i64)),
            DecodeSource::Both(n) => ("both", Some(n as i64)),
        };
        let r = self.conn.execute(
            "INSERT INTO decodes
               (session_id, utc, snr_db, dt, freq_hz, message, source, depth)
             VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8)",
            params![
                self.session_id,
                d.utc.to_rfc3339(),
                d.snr_db,
                d.dt,
                d.freq_hz,
                d.message,
                source,
                depth,
            ],
        );
        match r {
            Ok(_) => Some(self.conn.last_insert_rowid()),
            Err(e) => {
                eprintln!("[store] record_decode failed: {e}");
                None
            }
        }
    }

    /// Link an accumulation decode to one of its contributing periods.
    pub fn record_accum_link(&self, decode_id: i64, period_id: i64) {
        let r = self.conn.execute(
            "INSERT OR IGNORE INTO accum_periods (decode_id, period_id)
             VALUES (?1, ?2)",
            params![decode_id, period_id],
        );
        if let Err(e) = r {
            eprintln!("[store] record_accum_link failed: {e}");
        }
    }

    /// Record one period's soft-LLR evidence; returns its row id, or `None`
    /// when soft capture is disabled or the insert failed. Errors are logged,
    /// never propagated.
    pub fn record_period(&self, p: &PeriodSoft) -> Option<i64> {
        if !self.capture_soft {
            return None;
        }
        // Pack the LLR vector as little-endian f32.
        let mut blob = Vec::with_capacity(LLR_LEN * 4);
        for &v in p.llr.iter().take(LLR_LEN) {
            blob.extend_from_slice(&(v as f32).to_le_bytes());
        }
        // Pad if the decoder handed us a short vector (should not happen).
        while blob.len() < LLR_LEN * 4 {
            blob.extend_from_slice(&0f32.to_le_bytes());
        }
        let r = self.conn.execute(
            "INSERT INTO periods
               (session_id, utc, freq_hz, dt, sync, decoded, llr)
             VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7)",
            params![
                self.session_id,
                p.utc.to_rfc3339(),
                p.freq_hz,
                p.dt,
                p.sync,
                p.decoded as i64,
                blob,
            ],
        );
        match r {
            Ok(_) => Some(self.conn.last_insert_rowid()),
            Err(e) => {
                eprintln!("[store] record_period failed: {e}");
                None
            }
        }
    }

    /// Mark the session ended. Best-effort; call on a clean shutdown.
    pub fn close_session(&self) {
        let r = self.conn.execute(
            "UPDATE sessions SET ended_at = ?1 WHERE id = ?2",
            params![Utc::now().to_rfc3339(), self.session_id],
        );
        if let Err(e) = r {
            eprintln!("[store] close_session failed: {e}");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decoder::{Decode, DecodeSource};

    fn temp_db() -> std::path::PathBuf {
        let mut p = std::env::temp_dir();
        let uniq = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        p.push(format!("ft8mon_test_{uniq}.db"));
        p
    }

    fn a_decode(msg: &str, source: DecodeSource) -> Decode {
        Decode {
            utc: Utc::now(),
            snr_db: -12,
            dt: 0.2,
            freq_hz: 1160.0,
            message: msg.to_string(),
            source,
            contrib_tags: Vec::new(),
        }
    }

    #[test]
    fn schema_and_session_created() {
        let path = temp_db();
        let store = Store::open(&path, Some("USB Audio CODEC"), 12000, true).unwrap();
        assert!(store.session_id() >= 1);
        // All four tables exist.
        let n: i64 = store
            .conn
            .query_row(
                "SELECT count(*) FROM sqlite_master
                 WHERE type='table'
                   AND name IN ('sessions','decodes','periods','accum_periods')",
                [],
                |r| r.get(0),
            )
            .unwrap();
        assert_eq!(n, 4);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn accum_link_round_trips() {
        let path = temp_db();
        let store = Store::open(&path, None, 12000, true).unwrap();
        // A couple of periods, then an accum decode linked to both.
        let llr = vec![0.1f64; LLR_LEN];
        let mk = |dec| PeriodSoft {
            utc: Utc::now(),
            freq_hz: 1500.0,
            dt: 0.0,
            sync: 3,
            llr: llr.clone(),
            decoded: dec,
        };
        let p1 = store.record_period(&mk(false)).unwrap();
        let p2 = store.record_period(&mk(false)).unwrap();
        let did = store
            .record_decode(&a_decode("CQ M8FQT IO93", DecodeSource::Accumulated(2)))
            .unwrap();
        store.record_accum_link(did, p1);
        store.record_accum_link(did, p2);
        // The decode links to exactly its two contributing periods.
        let linked: i64 = store
            .conn
            .query_row(
                "SELECT count(*) FROM accum_periods WHERE decode_id = ?1",
                [did],
                |r| r.get(0),
            )
            .unwrap();
        assert_eq!(linked, 2);
        // The PRIMARY KEY makes a duplicate link a no-op.
        store.record_accum_link(did, p1);
        let still: i64 = store
            .conn
            .query_row(
                "SELECT count(*) FROM accum_periods WHERE decode_id = ?1",
                [did],
                |r| r.get(0),
            )
            .unwrap();
        assert_eq!(still, 2);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn decode_round_trips() {
        let path = temp_db();
        let store = Store::open(&path, None, 12000, true).unwrap();
        store.record_decode(&a_decode("CQ G0VXM IO93", DecodeSource::SingleShot));
        store.record_decode(&a_decode("CQ M8LBY IO91", DecodeSource::Accumulated(3)));

        let (msg, src, depth): (String, String, Option<i64>) = store
            .conn
            .query_row(
                "SELECT message, source, depth FROM decodes WHERE source='accum'",
                [],
                |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?)),
            )
            .unwrap();
        assert_eq!(msg, "CQ M8LBY IO91");
        assert_eq!(src, "accum");
        assert_eq!(depth, Some(3));
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn period_blob_is_174_f32() {
        let path = temp_db();
        let store = Store::open(&path, None, 12000, true).unwrap();
        let llr: Vec<f64> = (0..LLR_LEN).map(|i| (i as f64) * 0.01 - 0.5).collect();
        store.record_period(&PeriodSoft {
            utc: Utc::now(),
            freq_hz: 1160.0,
            dt: 0.1,
            sync: 7,
            llr: llr.clone(),
            decoded: false,
        });
        let blob: Vec<u8> = store
            .conn
            .query_row("SELECT llr FROM periods", [], |r| r.get(0))
            .unwrap();
        assert_eq!(blob.len(), LLR_LEN * 4);
        // First value survives the f64 -> f32 round trip.
        let first = f32::from_le_bytes([blob[0], blob[1], blob[2], blob[3]]);
        assert!((first - (-0.5f32)).abs() < 1e-6);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn capture_soft_flag_gates_periods() {
        let path = temp_db();
        let store = Store::open(&path, None, 12000, false).unwrap();
        assert!(!store.capture_soft());
        store.record_period(&PeriodSoft {
            utc: Utc::now(),
            freq_hz: 1000.0,
            dt: 0.0,
            sync: 1,
            llr: vec![0.0; LLR_LEN],
            decoded: true,
        });
        let n: i64 = store
            .conn
            .query_row("SELECT count(*) FROM periods", [], |r| r.get(0))
            .unwrap();
        assert_eq!(n, 0, "periods must stay empty when capture_soft is false");
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn close_session_sets_ended_at() {
        let path = temp_db();
        let store = Store::open(&path, None, 12000, true).unwrap();
        store.close_session();
        let ended: Option<String> = store
            .conn
            .query_row(
                "SELECT ended_at FROM sessions WHERE id=?1",
                params![store.session_id()],
                |r| r.get(0),
            )
            .unwrap();
        assert!(ended.is_some());
        let _ = std::fs::remove_file(&path);
    }
}
