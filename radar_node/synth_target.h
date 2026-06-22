/*
 * synth_target.h — host-only synthetic IQ generator (the ground truth).
 *
 * Real RF capture is out of scope; instead we synthesise the exact IF/beat
 * signal a set of KNOWN targets would produce. Because we injected the targets,
 * every downstream stage has a known-correct answer to assert against.
 *
 * Frame buffer layout (the same shape the on-target ADC DMA delivers, post
 * TDM-demux): [virtual element][chirp][ADC sample], row-major.
 */
#ifndef SYNTH_TARGET_H
#define SYNTH_TARGET_H

#include <stdint.h>
#include "cplx.h"
#include "radar_config.h"

/* A known target. (range, radial velocity, azimuth, amplitude). `rcs` is used
 * as a linear amplitude scale standing in for sqrt(RCS)*path-loss. */
typedef struct {
    double range_m;        /* metres                                   */
    double velocity_mps;   /* radial velocity, +ve = receding (Doppler)*/
    double azimuth_deg;    /* boresight = 0 deg                        */
    double rcs;            /* linear reflection amplitude              */
} target_t;

/* Total complex samples in one frame buffer. */
#define SYNTH_FRAME_LEN (RC_NUM_VIRT * RC_N_CHIRPS * RC_N_SAMPLES)

/* Index of (virtual element m, chirp c, ADC sample n) in the frame buffer. */
static inline int synth_idx(int m, int c, int n) {
    return (m * RC_N_CHIRPS + c) * RC_N_SAMPLES + n;
}

/* Fill `frame` (length SYNTH_FRAME_LEN) with the superposed beat signal of all
 * `ntargets` targets plus seeded complex-Gaussian noise of std-dev noise_sigma.
 * Deterministic for a given seed. */
void synth_generate_frame(cplx *frame,
                          const target_t *targets, int ntargets,
                          double noise_sigma, uint32_t seed);

#endif /* SYNTH_TARGET_H */
