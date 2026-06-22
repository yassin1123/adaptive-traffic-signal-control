/*
 * aoa.c — azimuth estimation by angle-FFT over the virtual array.
 */
#include "aoa.h"
#include "fft.h"
#include "doppler_proc.h"   /* rd_cube_idx */

double aoa_azimuth(const cplx *rd_cube, int range_bin, int doppler_bin) {
    cplx buf[RC_ANGLE_FFT];

    /* gather the spatial snapshot: one complex sample per virtual element at this
     * range-Doppler cell (rectangular weighting — only 8 elements, so we keep the
     * full aperture for the best peak accuracy). */
    for (int m = 0; m < RC_NUM_VIRT; ++m)
        buf[m] = rd_cube[rd_cube_idx(m, range_bin, doppler_bin)];
    for (int m = RC_NUM_VIRT; m < RC_ANGLE_FFT; ++m)
        buf[m] = cplx_make(0.0f, 0.0f);                 /* zero-pad for interpolation */

    fft_forward(buf, RC_ANGLE_FFT);

    /* peak across the angle spectrum -> spatial frequency -> azimuth */
    int best = 0; float bestp = -1.0f;
    for (int a = 0; a < RC_ANGLE_FFT; ++a) {
        float p = cplx_mag2(buf[a]);
        if (p > bestp) { bestp = p; best = a; }
    }
    return rc_azimuth_of_bin(best);
}

radar_point_t aoa_make_point(const cplx *rd_cube, const detection_t *det) {
    radar_point_t pt;
    pt.range_m = rc_range_of_bin(det->range_bin);
    pt.velocity_mps = rc_velocity_of_bin(det->doppler_bin);
    pt.azimuth_deg = aoa_azimuth(rd_cube, det->range_bin, det->doppler_bin);
    pt.power = det->power;
    return pt;
}
