/*
 * synth_target.c — generate the physically-correct FMCW beat signal.
 *
 * For a target at range R, radial velocity v, azimuth theta, amplitude a, the
 * de-chirped IF sample at virtual element m, chirp c, ADC sample n is:
 *
 *     s = a * exp( j * 2*pi * ( f_beat * t_n  +  f_doppler * (c * Tc) )
 *                  + j * 2*pi * d_lambda * sin(theta) * m )
 *
 *   f_beat    = 2 * slope * R / c        (range  -> beat frequency, within chirp)
 *   f_doppler = 2 * v / lambda           (velocity-> phase advance across chirps)
 *   t_n       = n / Fs                   (fast-time sample instant)
 *   d_lambda  = element spacing in wavelengths (angle -> phase across the array)
 *
 * This is the textbook FMCW return; the same constants the processing chain uses
 * to invert it live in radar_config.h, so recovering R/v/theta closes the loop
 * against physics. TDM-MIMO is abstracted: the buffer presents the de-multiplexed
 * virtual array (on-target the data path performs TDM demux + Doppler comp).
 */
#include "synth_target.h"

/* --- deterministic RNG: xorshift32 + Box-Muller (seeded, reproducible) --- */
static uint32_t xs_state;
static void rng_seed(uint32_t s) { xs_state = s ? s : 0xA5A5A5A5u; }
static uint32_t xs_next(void) {
    uint32_t x = xs_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    xs_state = x;
    return x;
}
static double rng_uniform(void) {           /* (0,1) */
    return (xs_next() + 1.0) / 4294967297.0;
}
static double rng_gauss(void) {             /* N(0,1) via Box-Muller */
    double u1 = rng_uniform(), u2 = rng_uniform();
    return sqrt(-2.0 * log(u1)) * cos(2.0 * RC_PI * u2);
}

void synth_generate_frame(cplx *frame,
                          const target_t *targets, int ntargets,
                          double noise_sigma, uint32_t seed) {
    rng_seed(seed);

    for (int m = 0; m < RC_NUM_VIRT; ++m) {
        for (int c = 0; c < RC_N_CHIRPS; ++c) {
            for (int n = 0; n < RC_N_SAMPLES; ++n) {
                cplx acc = cplx_make(0.0f, 0.0f);
                double t_n = (double)n / RC_FS;
                double t_slow = (double)c * RC_CHIRP_PERIOD;
                for (int k = 0; k < ntargets; ++k) {
                    const target_t *tg = &targets[k];
                    double f_beat = 2.0 * RC_SLOPE * tg->range_m / RC_C;
                    double f_dopp = 2.0 * tg->velocity_mps / RC_LAMBDA;
                    double sin_th = sin(tg->azimuth_deg * RC_PI / 180.0);
                    double phase = 2.0 * RC_PI * (f_beat * t_n + f_dopp * t_slow)
                                 + 2.0 * RC_PI * RC_ELEMENT_SPACING * sin_th * (double)m;
                    cplx ph = cplx_expj((float)phase);
                    acc = cplx_add(acc, cplx_scale(ph, (float)tg->rcs));
                }
                if (noise_sigma > 0.0) {
                    acc.re += (float)(noise_sigma * rng_gauss());
                    acc.im += (float)(noise_sigma * rng_gauss());
                }
                frame[synth_idx(m, c, n)] = acc;
            }
        }
    }
}
