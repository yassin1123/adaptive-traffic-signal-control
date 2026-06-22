/*
 * range_proc.h — Range-FFT stage (DSS / HWA data-path stage 1).
 *
 * For each virtual element and each chirp, FFT the fast-time ADC samples. A beat
 * frequency (set by target range) becomes a peak in a range bin. Output is the
 * "range cube": [virtual element][chirp][range bin] of complex range profiles,
 * which the Doppler stage then transforms across the chirp dimension.
 */
#ifndef RANGE_PROC_H
#define RANGE_PROC_H

#include "cplx.h"
#include "radar_config.h"

#define RANGE_CUBE_LEN (RC_NUM_VIRT * RC_N_CHIRPS * RC_RANGE_FFT)

static inline int range_cube_idx(int m, int c, int r) {
    return (m * RC_N_CHIRPS + c) * RC_RANGE_FFT + r;
}

/* frame: [virt][chirp][sample] (SYNTH_FRAME_LEN). range_cube: RANGE_CUBE_LEN. */
void range_proc_run(const cplx *frame, cplx *range_cube);

#endif /* RANGE_PROC_H */
