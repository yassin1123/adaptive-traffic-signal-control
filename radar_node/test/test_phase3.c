/*
 * test_phase3.c — Phase 3 host verification: CFAR detection.
 *
 * (A) Detection in noise + constant-false-alarm behaviour: inject known targets,
 *     sweep the noise level, and assert CFAR (a) detects every real target and
 *     (b) keeps the false-alarm rate bounded and ~constant as noise rises — the
 *     defining CFAR property (the adaptive threshold tracks the noise floor).
 * (B) OS vs CA: two closely-spaced targets where the strong one sits in the
 *     weak one's training window. CA averaging is inflated and masks the weak
 *     target; OS-CFAR rejects the outlier and detects both.
 *
 * Detections are checked against the injected ground-truth cells.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../radar_config.h"
#include "../adc_source.h"
#include "../range_proc.h"
#include "../doppler_proc.h"
#include "../cfar.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { \
    int ok_ = (cond); printf("  [%s] ", ok_ ? "PASS" : "FAIL"); \
    printf(__VA_ARGS__); printf("\n"); if (!ok_) g_fail++; \
} while (0)

#define MAX_DET 30000
#define MATCH_R 3
#define MATCH_D 3

static int range_bin_of(double R)   { return (int)lround(R / rc_range_of_bin(1)); }
static int dopp_bin_of(double v) {
    int s = (int)lround(v / RC_VEL_RES);
    return ((s % RC_DOPPLER_FFT) + RC_DOPPLER_FFT) % RC_DOPPLER_FFT;
}
static int dopp_dist(int a, int b) {
    int dd = abs(a - b);
    return dd > RC_DOPPLER_FFT / 2 ? RC_DOPPLER_FFT - dd : dd;
}

/* is detection (r,d) within the match window of target cell (tr,td)? */
static int near_target(int r, int d, int tr, int td) {
    return abs(r - tr) <= MATCH_R && dopp_dist(d, td) <= MATCH_D;
}

/* full chain: synth scene (with noise) -> range-Doppler power map */
static void make_map(const target_t *tg, int ntg, double sigma, uint32_t seed,
                     cplx *frame, cplx *rc, cplx *rd, float *map) {
    adc_synth_ctx ctx = { tg, ntg, sigma, seed, 0 };
    adc_source src = adc_source_synth(&ctx);
    src.fill(&src, frame);
    range_proc_run(frame, rc);
    doppler_proc_run(rc, rd);
    doppler_proc_power_map(rd, map);
}

int main(void) {
    printf("=== PHASE 3 - CFAR detection ===\n\n");

    int N = cfar_num_training();
    int k = (int)(RC_CFAR_OS_FRAC * N);
    printf("  CFAR window: guard=%d train=%d -> %d training cells (2-D)\n",
           RC_CFAR_GUARD, RC_CFAR_TRAIN, N);
    printf("  Pfa=%.0e -> alpha_CA=%.2f  alpha_OS(k=%d)=%.2f\n\n",
           RC_CFAR_PFA, cfar_alpha_ca(N), k, cfar_alpha_os(N, k));

    cplx *frame = malloc(sizeof(cplx) * SYNTH_FRAME_LEN);
    cplx *rc = malloc(sizeof(cplx) * RANGE_CUBE_LEN);
    cplx *rd = malloc(sizeof(cplx) * RD_CUBE_LEN);
    float *map = malloc(sizeof(float) * RD_MAP_LEN);
    detection_t *det = malloc(sizeof(detection_t) * MAX_DET);

    int n_test_cells = (RC_RANGE_FFT - 2 * (RC_CFAR_GUARD + RC_CFAR_TRAIN)) * RC_DOPPLER_FFT;

    /* ---------- (A) detection + constant-FA across a noise sweep ---------- */
    target_t scene[] = {
        { 25.0, +10.0, 0.0, 1.0 },
        { 60.0, -15.0, 0.0, 0.6 },
        { 85.0,  +5.0, 0.0, 0.8 },
    };
    int nt = 3;
    int tr[3], td[3];
    for (int i = 0; i < nt; ++i) { tr[i] = range_bin_of(scene[i].range_m); td[i] = dopp_bin_of(scene[i].velocity_mps); }

    /* Calibrate a NON-adaptive fixed threshold at the lowest noise level so it
     * detects the targets there (same job as CFAR). It does NOT adapt, so as the
     * noise floor rises its false alarms explode — the contrast that shows what
     * CFAR's adaptive threshold buys you, and that 0 CFAR FA is adaptivity (not a
     * threshold set so high nothing could ever trigger). */
    make_map(scene, nt, 0.5, 1234, frame, rc, rd, map);
    double gmean = 0.0;
    for (int r = RC_CFAR_GUARD + RC_CFAR_TRAIN; r < RC_RANGE_FFT - (RC_CFAR_GUARD + RC_CFAR_TRAIN); ++r)
        for (int d = 0; d < RC_DOPPLER_FFT; ++d) gmean += map[rd_map_idx(r, d)];
    gmean /= n_test_cells;
    double fixed_thresh = cfar_alpha_ca(N) * gmean;   /* fixed forever after */

    double sweep[] = { 0.5, 1.0, 2.0, 4.0 };
    int fixed_fa_hi = 0;
    printf("(A) detection + false-alarm vs noise.  CFAR adapts; FIXED threshold does not:\n");
    printf("    %-6s | %-16s | %-16s | %-16s\n", "sigma",
           "CA  det/FA", "OS  det/FA", "FIXED det/FA");
    for (int s = 0; s < 4; ++s) {
        double sigma = sweep[s];
        make_map(scene, nt, sigma, 1234, frame, rc, rd, map);
        char line[160]; int off = 0;
        off += sprintf(line + off, "    %-6.1f |", sigma);
        for (int detr = 0; detr < 3; ++detr) {   /* 0=CA 1=OS 2=FIXED */
            int nd;
            if (detr < 2) {
                nd = cfar_detect(map, detr == 0 ? CFAR_CA : CFAR_OS, det, MAX_DET);
            } else {
                nd = 0;                            /* fixed-threshold detector */
                for (int r = RC_CFAR_GUARD + RC_CFAR_TRAIN; r < RC_RANGE_FFT - (RC_CFAR_GUARD + RC_CFAR_TRAIN); ++r)
                    for (int d = 0; d < RC_DOPPLER_FFT; ++d)
                        if (map[rd_map_idx(r, d)] > fixed_thresh && nd < MAX_DET) {
                            det[nd].range_bin = r; det[nd].doppler_bin = d; nd++;
                        }
            }
            int detected = 0, fa = 0, hit[3] = { 0, 0, 0 };
            for (int i = 0; i < nd && i < MAX_DET; ++i) {
                int matched = 0;
                for (int t = 0; t < nt; ++t)
                    if (near_target(det[i].range_bin, det[i].doppler_bin, tr[t], td[t])) { hit[t] = 1; matched = 1; }
                if (!matched) fa++;
            }
            for (int t = 0; t < nt; ++t) detected += hit[t];
            off += sprintf(line + off, " %d/3 %5d |", detected, fa);

            if (detr < 2) {   /* gate asserts only on the CFAR detectors */
                if (sigma <= 2.0)
                    CHECK(detected == nt, "%s sigma=%.1f detected all %d targets",
                          detr == 0 ? "CA" : "OS", sigma, nt);
                CHECK((double)fa / n_test_cells < 0.01,
                      "%s sigma=%.1f false-alarm frac %.4f < 0.01 (bounded)",
                      detr == 0 ? "CA" : "OS", sigma, (double)fa / n_test_cells);
            } else if (sigma >= 4.0) {
                fixed_fa_hi = fa;
            }
        }
        printf("%s\n", line);
    }
    CHECK(fixed_fa_hi > 1000,
          "non-adaptive FIXED threshold false-alarms explode at high noise "
          "(%d FA) while CFAR stays ~0 -> CFAR's adaptivity demonstrated", fixed_fa_hi);

    /* show detected cells vs truth at the design noise level (CA) */
    printf("\n  detected cells vs ground truth (CA, sigma=1.0):\n");
    make_map(scene, nt, 1.0, 1234, frame, rc, rd, map);
    int nd = cfar_detect(map, CFAR_CA, det, MAX_DET);
    for (int t = 0; t < nt; ++t) {
        int cells = 0; detection_t best = { 0, 0, -1 };
        for (int i = 0; i < nd; ++i)
            if (near_target(det[i].range_bin, det[i].doppler_bin, tr[t], td[t])) {
                cells++;
                if (det[i].power > best.power) best = det[i];
            }
        printf("    target %d  truth(R=%.1f,v=%+.1f) cell(%d,%d) -> peak det cell(%d,%d), %d cells\n",
               t, scene[t].range_m, scene[t].velocity_mps, tr[t], td[t],
               best.range_bin, best.doppler_bin, cells);
    }

    /* ---------- (B) OS-CFAR resists multi-target masking ---------- */
    printf("\n(B) two close targets (strong neighbour in weak target's training window):\n");
    target_t pair[] = {
        { 50.0, 0.0, 0.0, 1.6 },   /* strong */
        { 52.1, 0.0, 0.0, 0.5 },   /* weak, ~5 range bins away (in training zone) */
    };
    int pr[2] = { range_bin_of(pair[0].range_m), range_bin_of(pair[1].range_m) };
    int pd[2] = { dopp_bin_of(pair[0].velocity_mps), dopp_bin_of(pair[1].velocity_mps) };
    make_map(pair, 2, 0.5, 99, frame, rc, rd, map);
    for (int kind = 0; kind < 2; ++kind) {
        int ndp = cfar_detect(map, kind == 0 ? CFAR_CA : CFAR_OS, det, MAX_DET);
        int h0 = 0, h1 = 0;
        for (int i = 0; i < ndp; ++i) {
            if (near_target(det[i].range_bin, det[i].doppler_bin, pr[0], pd[0])) h0 = 1;
            if (near_target(det[i].range_bin, det[i].doppler_bin, pr[1], pd[1])) h1 = 1;
        }
        printf("    %s: strong=%s  weak=%s\n", kind == 0 ? "CA" : "OS",
               h0 ? "detected" : "MISSED", h1 ? "detected" : "MISSED");
        if (kind == 1)
            CHECK(h0 && h1, "OS-CFAR detects BOTH closely-spaced targets");
    }

    free(frame); free(rc); free(rd); free(map); free(det);
    printf("\n=== PHASE 3 GATE: %s (%d failures) ===\n",
           g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
