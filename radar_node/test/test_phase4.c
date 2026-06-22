/*
 * test_phase4.c — Phase 4 host verification: angle-of-arrival.
 *
 * Inject targets at KNOWN azimuths (distinct ranges so they detect separately),
 * run the chain through CFAR, estimate azimuth per detection from the virtual
 * array, and assert the recovered angle in DEGREES matches the injected angle
 * within tolerance. Output is the (range, velocity, azimuth) point per target —
 * exactly what Phase 5 clusters into objects.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../radar_config.h"
#include "../adc_source.h"
#include "../range_proc.h"
#include "../doppler_proc.h"
#include "../cfar.h"
#include "../aoa.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { \
    int ok_ = (cond); printf("  [%s] ", ok_ ? "PASS" : "FAIL"); \
    printf(__VA_ARGS__); printf("\n"); if (!ok_) g_fail++; \
} while (0)

#define MAX_DET 30000
static int range_bin_of(double R) { return (int)lround(R / rc_range_of_bin(1)); }
static int dopp_bin_of(double v) {
    int s = (int)lround(v / RC_VEL_RES);
    return ((s % RC_DOPPLER_FFT) + RC_DOPPLER_FFT) % RC_DOPPLER_FFT;
}
static int dopp_dist(int a, int b) {
    int dd = abs(a - b);
    return dd > RC_DOPPLER_FFT / 2 ? RC_DOPPLER_FFT - dd : dd;
}

int main(void) {
    printf("=== PHASE 4 - angle-of-arrival (azimuth) ===\n\n");
    printf("  virtual array: %d elements @ %.1f lambda -> FoV +/-90 deg, "
           "ang. res ~%.0f deg @ boresight\n\n",
           RC_NUM_VIRT, RC_ELEMENT_SPACING,
           asin(1.0 / (RC_NUM_VIRT * RC_ELEMENT_SPACING)) * 180.0 / RC_PI);

    /* known azimuths across the field of view; distinct ranges */
    target_t scene[] = {
        { 20.0,  +8.0,   0.0, 1.0 },
        { 40.0,  +8.0, +20.0, 1.0 },
        { 60.0,  -5.0, -30.0, 1.0 },
        { 80.0, +12.0, +40.0, 1.0 },
    };
    int nt = 4;

    cplx *frame = malloc(sizeof(cplx) * SYNTH_FRAME_LEN);
    cplx *rc = malloc(sizeof(cplx) * RANGE_CUBE_LEN);
    cplx *rd = malloc(sizeof(cplx) * RD_CUBE_LEN);
    float *map = malloc(sizeof(float) * RD_MAP_LEN);
    detection_t *det = malloc(sizeof(detection_t) * MAX_DET);

    adc_synth_ctx ctx = { scene, nt, 0.5, 2024, 0 };
    adc_source src = adc_source_synth(&ctx);
    src.fill(&src, frame);
    range_proc_run(frame, rc);
    doppler_proc_run(rc, rd);
    doppler_proc_power_map(rd, map);
    int nd = cfar_detect(map, CFAR_CA, det, MAX_DET);

    printf("  detections -> (range, velocity, azimuth) vs injected truth:\n");
    double tol_deg = 5.0;
    for (int t = 0; t < nt; ++t) {
        int tr = range_bin_of(scene[t].range_m);
        int tdv = dopp_bin_of(scene[t].velocity_mps);
        /* peak CFAR detection for this target */
        detection_t best = { 0, 0, -1.0f };
        for (int i = 0; i < nd; ++i)
            if (abs(det[i].range_bin - tr) <= 3 && dopp_dist(det[i].doppler_bin, tdv) <= 3
                && det[i].power > best.power)
                best = det[i];

        radar_point_t pt = aoa_make_point(rd, &best);
        int ok_r = fabs(pt.range_m - scene[t].range_m) <= 1.5 * RC_RANGE_RES;
        int ok_v = fabs(pt.velocity_mps - scene[t].velocity_mps) <= 1.5 * RC_VEL_RES;
        int ok_a = fabs(pt.azimuth_deg - scene[t].azimuth_deg) <= tol_deg;
        CHECK(ok_r && ok_v && ok_a,
              "truth(R=%.1f v=%+.1f az=%+.1f) -> (R=%.2f v=%+.2f az=%+.2f)  [err az=%+.2f deg]",
              scene[t].range_m, scene[t].velocity_mps, scene[t].azimuth_deg,
              pt.range_m, pt.velocity_mps, pt.azimuth_deg,
              pt.azimuth_deg - scene[t].azimuth_deg);
    }

    free(frame); free(rc); free(rd); free(map); free(det);
    printf("\n=== PHASE 4 GATE: %s (%d failures) ===\n",
           g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
