/* ft8_shim.cpp — implementation of the C ABI over MSHV's DecoderFt8.
 *
 * Build dependencies (all on the C++ side):
 *   MSHV sources : decoderft8.cpp, decoderft8var.cpp, decoderpom.cpp,
 *                  genpom.cpp, gen_ft8.cpp, pack_unpack_msg77.cpp
 *   libraries    : Qt5Core, fftw3
 *
 * Requires the two MSHV patches (see ft8mon's mshv_ffi.patch):
 *   - DecoderFt8 de-QObject'd
 *   - its result signal replaced by SetResultCallback()/Ft8ResultFn
 */
#include "ft8_shim.h"
#include "decoderms.h"

#include <QString>
#include <QStringList>
#include <QByteArray>
#include <vector>
#include <cstring>

/* MSHV `dd` is conventionally int16-scaled (WSJT-X reads i2 wave files).
 * Our pipeline hands over f32 in roughly [-1, 1], so scale up to match. */
static const double SAMPLE_SCALE = 32768.0;
static const int    SLOT_SAMPLES = 15 * 12000; /* 180000 */

struct Ft8Decoder {
    DecoderFt8 *dec;
    std::vector<double> dd;   /* reusable 12 kHz double scratch buffer */
    int depth;
    Ft8LlrCb llr_cb;          /* per-candidate LLR sink (may be null)  */
    void    *llr_ctx;
};

/* Per-decode context handed to the MSHV-side trampoline. */
struct DecodeCall {
    Ft8ResultCb cb;
    void       *ctx;
    int         count;
};

static void copy_str(char *dst, size_t cap, const QString &s) {
    QByteArray b = s.trimmed().toUtf8();
    size_t n = (size_t)b.size();
    if (n > cap - 1) n = cap - 1;
    if (n) std::memcpy(dst, b.constData(), n);
    dst[n] = '\0';
}

/* MSHV calls this once per decoded message. The 8-field QStringList layout
 * is fixed by decoderft8.cpp's emit site:
 *   [0] time  [1] snr  [2] dt  [3] df(rel)  [4] message
 *   [5] aptype  [6] qual  [7] audio freq (Hz) */
static void mshv_trampoline(void *ctx, const QStringList &f) {
    DecodeCall *call = static_cast<DecodeCall *>(ctx);
    Ft8Result r;
    std::memset(&r, 0, sizeof(r));

    copy_str(r.time,    sizeof(r.time),    f.value(0));
    r.snr_db  = f.value(1).toInt();
    r.dt      = f.value(2).toFloat();
    copy_str(r.message, sizeof(r.message), f.value(4));
    copy_str(r.aptype,  sizeof(r.aptype),  f.value(5));
    r.qual    = f.value(6).toFloat();
    r.freq_hz = f.value(7).toInt();

    call->count++;
    if (call->cb) call->cb(call->ctx, &r);
}

/* MSHV calls this once per candidate inside ft8b(), with the base LLR vector.
 * `ctx` is the Ft8Decoder handle, so we can reach the user's LLR callback. */
static void mshv_llr_trampoline(void *ctx, double freq_hz, double dt,
                                int sync, const double *llr174) {
    Ft8Decoder *h = static_cast<Ft8Decoder *>(ctx);
    if (!h || !h->llr_cb || !llr174) return;
    Ft8LlrSample s;
    s.freq_hz = (float)freq_hz;
    s.dt      = (float)dt;
    s.sync    = sync;
    for (int i = 0; i < 174; ++i) s.llr[i] = llr174[i];
    h->llr_cb(h->llr_ctx, &s);
}

extern "C" {

Ft8Decoder *ft8_decoder_new(int id) {
    Ft8Decoder *h = new Ft8Decoder;
    h->dec     = new DecoderFt8(id);   /* ~20 MB — must be heap */
    h->depth   = 3;
    h->llr_cb  = nullptr;
    h->llr_ctx = nullptr;
    h->dd.resize(SLOT_SAMPLES, 0.0);
    return h;
}

void ft8_decoder_free(Ft8Decoder *h) {
    if (!h) return;
    delete h->dec;
    delete h;
}

void ft8_decoder_set_depth(Ft8Decoder *h, int depth) {
    if (h) h->depth = depth;
}

int ft8_decoder_decode(Ft8Decoder *h,
                       const float *samples, int n_samples,
                       const char *utc,
                       double f_lo, double f_hi, double f_qso,
                       Ft8ResultCb cb, void *ctx) {
    if (!h || !samples) return 0;

    /* Fill the 180000-sample double buffer: scale, zero-pad / truncate. */
    h->dd.assign(SLOT_SAMPLES, 0.0);
    int n = n_samples < SLOT_SAMPLES ? n_samples : SLOT_SAMPLES;
    for (int i = 0; i < n; ++i)
        h->dd[i] = (double)samples[i] * SAMPLE_SCALE;

    DecodeCall call;
    call.cb    = cb;
    call.ctx   = ctx;
    call.count = 0;

    h->dec->SetResultCallback(mshv_trampoline, &call);
    h->dec->SetLlrCallback(h->llr_cb ? mshv_llr_trampoline : nullptr, h);
    h->dec->SetNewP(true);                       /* reset per-slot state  */
    h->dec->SetStDecoderDeep(h->depth);
    h->dec->SetStApDecode(true);                 /* a-priori passes on    */
    h->dec->SetStDecode(QString::fromUtf8(utc ? utc : "000000"), 0, false);

    bool have_dec = false;
    h->dec->ft8_decode(h->dd.data(), SLOT_SAMPLES,
                       f_lo, f_hi, f_qso, have_dec,
                       /*id3dec*/ 1, /*w_f00*/ f_lo, /*w_f01*/ f_hi);

    /* Detach the callbacks so a stale `call` is never reachable. */
    h->dec->SetResultCallback(nullptr, nullptr);
    h->dec->SetLlrCallback(nullptr, nullptr);
    return call.count;
}

void ft8_decoder_set_llr_callback(Ft8Decoder *h, Ft8LlrCb cb, void *ctx) {
    if (!h) return;
    h->llr_cb  = cb;
    h->llr_ctx = ctx;
}

int ft8_ldpc_try(Ft8Decoder *h, const double *llr174,
                 char *msg_out, int msg_cap) {
    if (!h || !llr174 || !msg_out || msg_cap <= 0) return 0;
    QString msg;
    int    nharderrors = -1;
    double dmin        = 0.0;
    bool ok = h->dec->ldpc_try(llr174, msg, nharderrors, dmin);
    if (!ok) { msg_out[0] = '\0'; return 0; }
    copy_str(msg_out, (size_t)msg_cap, msg);
    return 1;
}

} /* extern "C" */
