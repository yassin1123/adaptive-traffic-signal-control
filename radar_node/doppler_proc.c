/*
 * doppler_proc.c — Doppler-FFT across the chirp dimension, per range bin.
 *
 * For a fixed virtual element and range bin, the sequence of complex values
 * across chirps carries the Doppler phase rotation of any reflector in that bin.
 * FFT-ing it resolves radial velocity. A Hann window across chirps suppresses
 * Doppler sidelobes. The natural FFT bin order is kept; the fftshift / sign
 * convention (positive velocity -> low positive bin) lives in
 * rc_velocity_of_bin() so the whole codebase shares one mapping.
 *
 * On the IWR6843 this is the HWA's second FFT pass; here it is portable fft().
 */
#include "doppler_proc.h"
#include "range_proc.h"
#include "fft.h"

void doppler_proc_run(const cplx *range_cube, cplx *rd_cube) {
    cplx buf[RC_DOPPLER_FFT];

    for (int m = 0; m < RC_NUM_VIRT; ++m) {
        for (int r = 0; r < RC_RANGE_FFT; ++r) {
            /* gather this range bin across all chirps, windowed */
            for (int c = 0; c < RC_N_CHIRPS; ++c) {
                double w = 0.5 - 0.5 * cos(2.0 * RC_PI * c / (RC_N_CHIRPS - 1));
                buf[c] = cplx_scale(range_cube[range_cube_idx(m, c, r)], (float)w);
            }
            for (int c = RC_N_CHIRPS; c < RC_DOPPLER_FFT; ++c)
                buf[c] = cplx_make(0.0f, 0.0f);     /* zero-pad to FFT length */

            fft_forward(buf, RC_DOPPLER_FFT);

            for (int d = 0; d < RC_DOPPLER_FFT; ++d)
                rd_cube[rd_cube_idx(m, r, d)] = buf[d];
        }
    }
}

void doppler_proc_power_map(const cplx *rd_cube, float *map) {
    for (int r = 0; r < RC_RANGE_FFT; ++r) {
        for (int d = 0; d < RC_DOPPLER_FFT; ++d) {
            float p = 0.0f;
            for (int m = 0; m < RC_NUM_VIRT; ++m)
                p += cplx_mag2(rd_cube[rd_cube_idx(m, r, d)]);
            map[rd_map_idx(r, d)] = p;
        }
    }
}
