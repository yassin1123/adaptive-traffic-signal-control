/*
 * doppler_proc.h — Doppler-FFT stage (DSS / HWA data-path stage 2).
 *
 * Across the chirps of a frame, a moving target's phase rotates at the Doppler
 * frequency f_d = 2v/lambda. FFT-ing each range bin across the chirp dimension
 * resolves that into a velocity bin, yielding a 2-D range-Doppler map per virtual
 * element. Non-coherent integration across the virtual array gives the power map
 * that CFAR (Phase 3) thresholds.
 */
#ifndef DOPPLER_PROC_H
#define DOPPLER_PROC_H

#include "cplx.h"
#include "radar_config.h"

#define RD_CUBE_LEN (RC_NUM_VIRT * RC_RANGE_FFT * RC_DOPPLER_FFT)
#define RD_MAP_LEN  (RC_RANGE_FFT * RC_DOPPLER_FFT)

static inline int rd_cube_idx(int m, int r, int d) {
    return (m * RC_RANGE_FFT + r) * RC_DOPPLER_FFT + d;
}
static inline int rd_map_idx(int r, int d) {
    return r * RC_DOPPLER_FFT + d;
}

/* range_cube -> per-element range-Doppler cube (complex). */
void doppler_proc_run(const cplx *range_cube, cplx *rd_cube);

/* rd_cube -> non-coherently integrated power map (sum of |.|^2 over elements). */
void doppler_proc_power_map(const cplx *rd_cube, float *map);

#endif /* DOPPLER_PROC_H */
