/*
 * range_proc.c — range-FFT across each chirp's fast-time samples.
 *
 * Physics: after de-chirping, a target at range R produces a constant-frequency
 * beat tone f_b = 2*slope*R/c within each chirp. FFT-ing the N_SAMPLES of a chirp
 * turns that tone into a peak at the range bin for R. A Hann window is applied
 * first to suppress spectral sidelobes (so a strong reflector doesn't mask a
 * weak neighbour) — standard in real radar; it widens the main lobe slightly but
 * leaves the peak bin (hence the recovered range) unchanged.
 *
 * On the IWR6843 this is the HWA's range-FFT pass; here it is the portable fft().
 */
#include "range_proc.h"
#include "fft.h"
#include "synth_target.h"   /* for synth_idx frame layout (shared ADC layout) */

void range_proc_run(const cplx *frame, cplx *range_cube) {
    cplx buf[RC_RANGE_FFT];

    for (int m = 0; m < RC_NUM_VIRT; ++m) {
        for (int c = 0; c < RC_N_CHIRPS; ++c) {
            /* windowed copy of this chirp's ADC samples */
            for (int n = 0; n < RC_N_SAMPLES; ++n) {
                double w = 0.5 - 0.5 * cos(2.0 * RC_PI * n / (RC_N_SAMPLES - 1));
                buf[n] = cplx_scale(frame[synth_idx(m, c, n)], (float)w);
            }
            for (int n = RC_N_SAMPLES; n < RC_RANGE_FFT; ++n)
                buf[n] = cplx_make(0.0f, 0.0f);     /* zero-pad to FFT length */

            fft_forward(buf, RC_RANGE_FFT);

            for (int r = 0; r < RC_RANGE_FFT; ++r)
                range_cube[range_cube_idx(m, c, r)] = buf[r];
        }
    }
}
