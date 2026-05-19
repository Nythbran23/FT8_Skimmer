/* ft8_shim.h — C ABI over MSHV's DecoderFt8.
 *
 * Wraps the (de-Qt'd) MSHV FT8 decoder behind a plain C interface so Rust
 * can bind it with a stable ABI. All Qt types stay on the C++ side of this
 * boundary; nothing Qt crosses into Rust.
 *
 * The MSHV decoder is GPLv3 — anything linked against this shim inherits
 * GPLv3 terms.
 */
#ifndef FT8_SHIM_H
#define FT8_SHIM_H

#ifdef __cplusplus
extern "C" {
#endif

/* One decoded FT8 message. Plain old data — safe to share with Rust. */
typedef struct {
    char  time[16];     /* slot time as MSHV reports it (e.g. "HHMMSS") */
    int   snr_db;       /* SNR in dB, 2500 Hz reference                 */
    float dt;           /* time offset, seconds                         */
    int   freq_hz;      /* audio frequency in the passband, Hz          */
    char  message[48];  /* decoded message text                         */
    char  aptype[8];    /* a-priori type tag, e.g. "AP1" / "?"          */
    float qual;         /* decode quality 0..1                          */
} Ft8Result;

/* Opaque decoder handle. */
typedef struct Ft8Decoder Ft8Decoder;

/* Result callback — invoked once per decode, synchronously, during
 * ft8_decoder_decode(). `r` is valid only for the duration of the call. */
typedef void (*Ft8ResultCb)(void *ctx, const Ft8Result *r);

/* One candidate's base (non-AP) LLR vector — the raw material for
 * cross-period soft accumulation. Delivered once per candidate. */
typedef struct {
    float  freq_hz;     /* audio frequency, Hz                   */
    float  dt;          /* time offset, seconds                  */
    int    sync;        /* sync-metric rank (lower = stronger)   */
    double llr[174];    /* base log-likelihood ratios, 174 bits  */
} Ft8LlrSample;

/* LLR callback — invoked once per candidate during ft8_decoder_decode(),
 * with that candidate's base LLR vector. `s` is valid only for the call. */
typedef void (*Ft8LlrCb)(void *ctx, const Ft8LlrSample *s);

/* Create / destroy. `id` is MSHV's decoder id (0 is fine for a single
 * decoder). The handle is heap-allocated — DecoderFt8 is ~20 MB. */
Ft8Decoder *ft8_decoder_new(int id);
void        ft8_decoder_free(Ft8Decoder *dec);

/* Decode depth: 1 = fast, 2 = normal, 3 = deep (more passes, slower). */
void ft8_decoder_set_depth(Ft8Decoder *dec, int depth);

/* Decode one 15 s slot.
 *
 *   samples   : mono audio, 12 kHz. Ideally 180000 samples; shorter is
 *               zero-padded, longer is truncated.
 *   n_samples : length of `samples`.
 *   utc       : slot time string passed through to the decoder.
 *   f_lo,f_hi : audio frequency search range, Hz (e.g. 200 .. 3000).
 *   f_qso     : nominal QSO frequency, Hz (e.g. 1500).
 *   cb,ctx    : result callback, invoked once per decoded message.
 *
 * Returns the number of messages decoded.
 */
int ft8_decoder_decode(Ft8Decoder *dec,
                       const float *samples, int n_samples,
                       const char *utc,
                       double f_lo, double f_hi, double f_qso,
                       Ft8ResultCb cb, void *ctx);

/* Register a callback that receives every candidate's base LLR vector during
 * ft8_decoder_decode(). Pass cb = NULL to disable. This is the data source
 * for cross-period soft accumulation. */
void ft8_decoder_set_llr_callback(Ft8Decoder *dec, Ft8LlrCb cb, void *ctx);

/* Run the LDPC+OSD decoder (CRC-14 validated internally) on a supplied
 * 174-element LLR vector — e.g. an accumulated cross-period LLR sum. On a
 * CRC-valid decode, writes the message text into msg_out (NUL-terminated, up
 * to msg_cap bytes) and returns 1. Returns 0 otherwise. */
int ft8_ldpc_try(Ft8Decoder *dec, const double *llr174,
                 char *msg_out, int msg_cap);

#ifdef __cplusplus
}
#endif

#endif /* FT8_SHIM_H */
