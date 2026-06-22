/*
 * cplx.h — tiny portable complex type for the FMCW signal chain.
 *
 * We deliberately use a plain {re, im} struct of floats rather than C99 _Complex:
 *   - portable across the old MinGW host toolchain (avoids <complex.h>/tgmath quirks),
 *   - reads like real embedded DSP code (the IWR6843 HWA/DSP work on interleaved
 *     I/Q sample pairs of exactly this shape),
 *   - keeps the data layout explicit, which matters when this ports to the device.
 *
 * On-target, these ops map onto the C674x DSP intrinsics / HWA; on host they are
 * straight scalar C. The algorithm using them is identical either way.
 */
#ifndef CPLX_H
#define CPLX_H

#include <math.h>

#ifndef RC_PI
#define RC_PI 3.14159265358979323846
#endif

typedef struct { float re, im; } cplx;

static inline cplx cplx_make(float re, float im) { cplx c = { re, im }; return c; }

static inline cplx cplx_add(cplx a, cplx b) { return cplx_make(a.re + b.re, a.im + b.im); }
static inline cplx cplx_sub(cplx a, cplx b) { return cplx_make(a.re - b.re, a.im - b.im); }

static inline cplx cplx_mul(cplx a, cplx b) {
    return cplx_make(a.re * b.re - a.im * b.im,
                     a.re * b.im + a.im * b.re);
}

static inline cplx cplx_scale(cplx a, float s) { return cplx_make(a.re * s, a.im * s); }
static inline cplx cplx_conj(cplx a) { return cplx_make(a.re, -a.im); }

/* squared magnitude (power) — cheap, no sqrt; what range-Doppler/CFAR work on */
static inline float cplx_mag2(cplx a) { return a.re * a.re + a.im * a.im; }
static inline float cplx_mag(cplx a) { return sqrtf(cplx_mag2(a)); }

/* phase angle in radians */
static inline float cplx_arg(cplx a) { return atan2f(a.im, a.re); }

/* unit phasor e^{j*theta} — the building block of every synthetic sinusoid */
static inline cplx cplx_expj(float theta) { return cplx_make(cosf(theta), sinf(theta)); }

#endif /* CPLX_H */
