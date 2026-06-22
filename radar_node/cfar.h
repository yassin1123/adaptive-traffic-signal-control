/*
 * cfar.h — Constant False Alarm Rate detection over the range-Doppler map.
 *
 * CFAR picks real targets out of noise with an ADAPTIVE threshold: for each
 * cell-under-test it estimates the local noise from surrounding TRAINING cells
 * (skipping GUARD cells next to the CUT so target energy doesn't leak into the
 * estimate) and flags a detection only if the CUT exceeds threshold = alpha *
 * noise_estimate. Because the threshold tracks the local noise, the false-alarm
 * probability stays ~constant (= Pfa) as the noise floor rises — the whole point.
 *
 * Two estimators (spec §3):
 *   CA-CFAR (cell-averaging): noise = mean(training). Best in homogeneous noise.
 *   OS-CFAR (ordered-statistic): noise = k-th sorted training cell. Robust when a
 *     second target sits in the training window (cell-averaging would be inflated
 *     by it and mask the real target); OS rejects such outliers.
 *
 * 2-D window over (range, Doppler). Doppler wraps (velocity is periodic); range
 * does not, so cells within (GUARD+TRAIN) of the range edges are not tested.
 * This is the MSS post-processing of the HWA's detection layer on-target.
 */
#ifndef CFAR_H
#define CFAR_H

#include "radar_config.h"

typedef enum { CFAR_CA, CFAR_OS } cfar_kind;

typedef struct {
    int range_bin;
    int doppler_bin;
    float power;
} detection_t;

/* Number of training cells in the 2-D window (computed from config). */
int cfar_num_training(void);

/* Threshold multipliers derived from the target Pfa. */
double cfar_alpha_ca(int n_train);
double cfar_alpha_os(int n_train, int k_rank);

/* Run CFAR over `map` (RD_MAP_LEN). Writes up to `max_out` detections; returns
 * the number found. */
int cfar_detect(const float *map, cfar_kind kind,
                detection_t *out, int max_out);

#endif /* CFAR_H */
