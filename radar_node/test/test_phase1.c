/*
 * test_phase1.c — Phase 1 host verification.
 *
 * Proves the foundation the whole chain rests on:
 *   1. radar_config self-consistency asserts pass; print derived quantities.
 *   2. The FFT lands a known tone in the known bin (and round-trips).
 *   3. The synthetic generator's RANGE physics are correct — verified in PHYSICAL
 *      units: inject a target at a known range, range-FFT one chirp, convert the
 *      peak bin back to METRES via the config's derived constants, and assert it
 *      matches the injected range. This closes the loop against physics, not
 *      against shared conventions (the foundation the gates depend on).
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../radar_config.h"
#include "../cplx.h"
#include "../fft.h"
#include "../synth_target.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { \
    int ok_ = (cond); \
    printf("  [%s] ", ok_ ? "PASS" : "FAIL"); \
    printf(__VA_ARGS__); printf("\n"); \
    if (!ok_) g_fail++; \
} while (0)

static int peak_bin(const cplx *x, int lo, int hi) {
    int best = lo; float bestp = -1.0f;
    for (int k = lo; k < hi; ++k) {
        float p = cplx_mag2(x[k]);
        if (p > bestp) { bestp = p; best = k; }
    }
    return best;
}

int main(void) {
    printf("=== PHASE 1 - config / FFT / synthetic IQ ===\n\n");

    /* ---- 1. config ---- */
    printf("config self-consistency:\n");
    radar_config_validate();           /* asserts internally */
    CHECK(1, "radar_config_validate() asserts passed");
    radar_config_print();
    printf("\n");

    /* ---- 2. FFT of a known tone ---- */
    printf("FFT of a known tone (N=%d):\n", RC_RANGE_FFT);
    const int N = RC_RANGE_FFT;
    const int k0 = 40;                  /* inject a pure tone at bin 40 */
    cplx *x = malloc(sizeof(cplx) * N);
    cplx *orig = malloc(sizeof(cplx) * N);
    for (int n = 0; n < N; ++n) {
        double ph = 2.0 * RC_PI * k0 * n / N;
        x[n] = cplx_expj((float)ph);
        orig[n] = x[n];
    }
    fft_forward(x, N);
    int kp = peak_bin(x, 0, N);
    CHECK(kp == k0, "tone at bin %d -> peak at bin %d", k0, kp);

    /* inverse round-trip error */
    fft_inverse(x, N);
    double err = 0.0;
    for (int n = 0; n < N; ++n) {
        err += fabs(x[n].re - orig[n].re) + fabs(x[n].im - orig[n].im);
    }
    err /= N;
    CHECK(err < 1e-4, "inverse FFT round-trip mean abs error = %.2e", err);
    free(x); free(orig);
    printf("\n");

    /* ---- 3. synthetic IQ range physics, verified in METRES ---- */
    printf("synthetic IQ range physics (recovered range vs injected, no noise):\n");
    cplx *frame = malloc(sizeof(cplx) * SYNTH_FRAME_LEN);
    cplx *chirp = malloc(sizeof(cplx) * RC_RANGE_FFT);
    double test_ranges[] = { 20.0, 40.0, 80.0 };
    double tol = RC_RANGE_RES;          /* one range bin */
    for (int t = 0; t < 3; ++t) {
        target_t tgt = { test_ranges[t], 0.0, 0.0, 1.0 };
        synth_generate_frame(frame, &tgt, 1, 0.0, 12345);
        /* take virtual element 0, chirp 0, all ADC samples; range-FFT it */
        for (int n = 0; n < RC_N_SAMPLES; ++n)
            chirp[n] = frame[synth_idx(0, 0, n)];
        for (int n = RC_N_SAMPLES; n < RC_RANGE_FFT; ++n)
            chirp[n] = cplx_make(0.0f, 0.0f);
        fft_forward(chirp, RC_RANGE_FFT);
        /* complex IQ -> no Hermitian symmetry: positive ranges span ALL bins */
        int kr = peak_bin(chirp, 0, RC_RANGE_FFT);
        double recovered = rc_range_of_bin(kr);
        CHECK(fabs(recovered - test_ranges[t]) <= tol,
              "injected %.1f m -> bin %d -> recovered %.2f m (tol +/-%.2f m)",
              test_ranges[t], kr, recovered, tol);
    }
    free(frame); free(chirp);

    printf("\n=== PHASE 1 GATE: %s (%d failures) ===\n",
           g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
