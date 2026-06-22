/*
 * test_phase2.c — Phase 2 host verification: range-FFT + Doppler-FFT.
 *
 * Inject targets at KNOWN (range, velocity) through the adc_source seam, run the
 * range and Doppler stages, then assert the peak cells convert back — via the
 * config's derived constants — to the injected range AND velocity (sign included)
 * in PHYSICAL units. Also renders the range-Doppler map so the targets are
 * visibly at the right cells.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../radar_config.h"
#include "../adc_source.h"
#include "../range_proc.h"
#include "../doppler_proc.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { \
    int ok_ = (cond); printf("  [%s] ", ok_ ? "PASS" : "FAIL"); \
    printf(__VA_ARGS__); printf("\n"); if (!ok_) g_fail++; \
} while (0)

typedef struct { int r, d; float p; } cell_t;

/* Greedy top-K peak finder with local suppression (so one widened peak isn't
 * counted as several). Pure search over the whole map — it does NOT look where
 * targets were injected. */
static int find_peaks(const float *map, cell_t *out, int K, int rsupp, int dsupp) {
    int found = 0;
    while (found < K) {
        int br = -1, bd = -1; float bp = -1.0f;
        for (int r = 0; r < RC_RANGE_FFT; ++r) {
            for (int d = 0; d < RC_DOPPLER_FFT; ++d) {
                float p = map[rd_map_idx(r, d)];
                if (p <= bp) continue;
                int suppressed = 0;
                for (int i = 0; i < found; ++i) {
                    int dr = abs(out[i].r - r);
                    int dd = abs(out[i].d - d);
                    if (dd > RC_DOPPLER_FFT / 2) dd = RC_DOPPLER_FFT - dd; /* wrap */
                    if (dr <= rsupp && dd <= dsupp) { suppressed = 1; break; }
                }
                if (!suppressed) { bp = p; br = r; bd = d; }
            }
        }
        if (br < 0) break;
        out[found].r = br; out[found].d = bd; out[found].p = bp;
        found++;
    }
    return found;
}

static void print_rd_map(const float *map) {
    const int ROWS = 22, DSTEP = 2;            /* downsample range rows; doppler /2 */
    const char *ramp = " .:-=+*#@";
    float mx = 0.0f;
    for (int i = 0; i < RD_MAP_LEN; ++i) if (map[i] > mx) mx = map[i];
    if (mx <= 0.0f) mx = 1.0f;

    printf("  range-Doppler map (rows=range, cols=velocity; '@'=strong)\n");
    printf("        vel:%+5.0f%*s0%*s%+5.0f m/s\n",
           -RC_MAX_VEL, (RC_DOPPLER_FFT / DSTEP) / 2 - 6, "",
           (RC_DOPPLER_FFT / DSTEP) / 2 - 4, "", RC_MAX_VEL);
    int rstep = RC_RANGE_FFT / ROWS;
    for (int ri = 0; ri < ROWS; ++ri) {
        double rng = rc_range_of_bin(ri * rstep + rstep / 2);
        printf("  %5.1f m |", rng);
        for (int s = 0; s < RC_DOPPLER_FFT; s += DSTEP) {
            int d = (s + RC_DOPPLER_FFT / 2) % RC_DOPPLER_FFT;   /* fftshift: 0 vel centre */
            float blk = 0.0f;
            for (int rr = 0; rr < rstep; ++rr) {
                float v = map[rd_map_idx(ri * rstep + rr, d)];
                if (v > blk) blk = v;
            }
            int lvl = (int)(8.0 * sqrt(blk / mx));   /* sqrt for visual dynamic range */
            if (lvl > 8) lvl = 8;
            putchar(ramp[lvl]);
        }
        printf("|\n");
    }
}

int main(void) {
    printf("=== PHASE 2 - range-FFT + Doppler-FFT ===\n\n");

    /* Two targets: distinct ranges, opposite-sign velocities (covers the Doppler
     * sign convention). No noise — CFAR/noise is Phase 3. */
    target_t targets[] = {
        { 30.0, +12.0, 0.0, 1.0 },
        { 55.0,  -8.0, 0.0, 1.0 },
    };
    int ntgt = 2;

    adc_synth_ctx ctx = { targets, ntgt, 0.0, 7, 0 };
    adc_source src = adc_source_synth(&ctx);

    cplx *frame = malloc(sizeof(cplx) * SYNTH_FRAME_LEN);
    cplx *rcube = malloc(sizeof(cplx) * RANGE_CUBE_LEN);
    cplx *rdcube = malloc(sizeof(cplx) * RD_CUBE_LEN);
    float *map = malloc(sizeof(float) * RD_MAP_LEN);

    src.fill(&src, frame);                 /* adc_source seam: synthetic this build */
    range_proc_run(frame, rcube);
    doppler_proc_run(rcube, rdcube);
    doppler_proc_power_map(rdcube, map);

    print_rd_map(map);
    printf("\n");

    /* find the two strongest, well-separated peaks and match to truth */
    cell_t peaks[8];
    int np = find_peaks(map, peaks, ntgt, 4, 6);
    CHECK(np == ntgt, "found %d distinct peaks (expected %d)", np, ntgt);

    double tol_r = 1.5 * RC_RANGE_RES;
    double tol_v = 1.5 * RC_VEL_RES;
    printf("  recovered detections vs injected truth (physical units):\n");
    for (int t = 0; t < ntgt; ++t) {
        /* match injected target t to the nearest found peak */
        int best = -1; double bestd = 1e9;
        for (int i = 0; i < np; ++i) {
            double rr = rc_range_of_bin(peaks[i].r);
            double vv = rc_velocity_of_bin(peaks[i].d);
            double dd = fabs(rr - targets[t].range_m) + fabs(vv - targets[t].velocity_mps);
            if (dd < bestd) { bestd = dd; best = i; }
        }
        double rr = rc_range_of_bin(peaks[best].r);
        double vv = rc_velocity_of_bin(peaks[best].d);
        int ok_r = fabs(rr - targets[t].range_m) <= tol_r;
        int ok_v = fabs(vv - targets[t].velocity_mps) <= tol_v;
        CHECK(ok_r && ok_v,
              "target %d: injected (R=%.1f m, v=%+.1f m/s) -> "
              "cell(r=%d,d=%d) -> (R=%.2f m, v=%+.2f m/s)",
              t, targets[t].range_m, targets[t].velocity_mps,
              peaks[best].r, peaks[best].d, rr, vv);
    }

    free(frame); free(rcube); free(rdcube); free(map);
    printf("\n=== PHASE 2 GATE: %s (%d failures) ===\n",
           g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
