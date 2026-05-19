/* c99_complex_shim.h — macOS / clang compatibility shim for the MSHV decoder.
 *
 * MSHV's FT8 decoder is C-with-classes: it expects the C99 <complex.h> API
 * (the `complex` keyword, the `I` imaginary unit, creal/cimag/cabs/conj) and
 * the C math/stdlib functions to be visible in the *global* namespace — the
 * way glibc's headers expose them in C++ mode. That is why the decoder builds
 * as-is with GCC on Linux.
 *
 * Apple's libc++ does not: its <complex.h> merely forwards to C++ <complex>,
 * and global C library names are not guaranteed. So a clang build fails with
 * "use of undeclared identifier 'I'", "'log10' was not declared", etc.
 *
 * build.rs force-includes this header (clang -include) ahead of every MSHV
 * translation unit on macOS. It does NOT replace any system header — it only
 * pulls the C library headers explicitly and supplies the four C99 complex
 * functions MSHV uses, backed by clang's __builtin_* complex intrinsics. The
 * decoder math therefore stays bit-identical to the GCC/Linux build.
 *
 * No-op on non-Apple platforms: glibc already provides all of this.
 */
#ifndef C99_COMPLEX_SHIM_H
#define C99_COMPLEX_SHIM_H

#if defined(__APPLE__) && defined(__cplusplus)

/* MSHV's C-style code calls these unqualified, in the global namespace, and
 * relies on them arriving transitively. Pull them explicitly — all idempotent. */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Include the C++ <complex> template header NOW, while `complex` is still a
 * plain identifier, so its include guard is set. Apple's <complex.h> — which
 * MSHV pulls in via decoderms.h — includes <complex> unconditionally; once it
 * is already guarded here, that re-include is skipped and the `complex` macro
 * defined below cannot corrupt `class complex` inside it. (glibc's <complex.h>
 * never drags in <complex>, which is why this is only needed on Apple.) */
#include <complex>

/* The C99 `complex` type-specifier (e.g. `double complex`). */
#ifndef complex
#define complex _Complex
#endif

/* C99 imaginary unit. Define _Complex_I as well: the bundled fftw3.h keys its
 * `fftw_complex == double _Complex` typedef on _Complex_I being defined, and
 * MSHV passes `double _Complex *` straight to the fftw_plan_* functions. */
#ifndef _Complex_I
#define _Complex_I (__builtin_complex(0.0f, 1.0f))
#endif
#ifndef I
#define I _Complex_I
#endif

/* MSHV bundles a flat Boost tree (it has gcc/macos/libcpp configs but no clang
 * compiler config). Point Boost's compiler config at the bundled gcc.hpp —
 * clang is GCC-compatible, and that is all <boost/crc.hpp> needs. */
#ifndef BOOST_COMPILER_CONFIG
#define BOOST_COMPILER_CONFIG "gcc.hpp"
#endif

/* C99 complex accessors/ops, mapped to clang builtins. `static inline` gives
 * each translation unit its own copy — no link-time duplication, and a plain
 * `::conj(double _Complex)` that overload resolution prefers over the
 * std::conj template (which cannot match a _Complex operand). */
static inline double          creal(double _Complex z) { return __builtin_creal(z); }
static inline double          cimag(double _Complex z) { return __builtin_cimag(z); }
static inline double          cabs (double _Complex z) { return __builtin_cabs(z);  }
static inline double _Complex conj (double _Complex z) { return __builtin_conj(z);  }

#endif /* __APPLE__ && __cplusplus */

#endif /* C99_COMPLEX_SHIM_H */
