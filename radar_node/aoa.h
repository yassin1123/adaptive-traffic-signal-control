/*
 * aoa.h — Angle-of-arrival (azimuth) estimation across the virtual array.
 *
 * A detection localises a target in range and Doppler; AoA adds azimuth, turning
 * a (range, velocity) cell into a point in space (range, velocity, angle) — the
 * input clustering/tracking needs to place objects on intersection approaches.
 *
 * Method: the NUM_VIRT virtual-array elements (TX x RX, TDM-MIMO) see the same
 * return with a progressive phase 2*pi*(d/lambda)*sin(theta)*m across elements m.
 * An angle-FFT across the elements turns that spatial phase ramp into a peak
 * whose bin maps to sin(theta), hence azimuth (rc_azimuth_of_bin). With an
 * 8-element half-wavelength array the unambiguous field of view is +/-90 deg and
 * the angular resolution is ~1/(M*d_lambda) in sin(theta) (~14 deg at boresight).
 *
 * On-target the virtual array is assembled by TDM demux with Doppler-phase
 * compensation in the data path; on host the synthetic frame already presents the
 * Doppler-compensated virtual array, so the angle-FFT applies directly.
 */
#ifndef AOA_H
#define AOA_H

#include "cplx.h"
#include "radar_config.h"
#include "cfar.h"          /* detection_t */

/* A detection promoted to physical coordinates — the chain's "point in space". */
typedef struct {
    double range_m;
    double velocity_mps;
    double azimuth_deg;
    float  power;
} radar_point_t;

/* Estimate azimuth (degrees) at a range-Doppler cell from the per-element cube. */
double aoa_azimuth(const cplx *rd_cube, int range_bin, int doppler_bin);

/* Promote a CFAR detection to a (range, velocity, azimuth, power) point. */
radar_point_t aoa_make_point(const cplx *rd_cube, const detection_t *det);

#endif /* AOA_H */
