/*
 * main_host.c — END-TO-END host verification of the full FMCW sensing chain.
 *
 * Runs a realistic multi-frame synthetic scene through the ENTIRE pipeline
 *   adc_source -> range-FFT -> Doppler-FFT -> CFAR -> AoA -> cluster -> track
 *   -> zones/classify -> node_output
 * and asserts, at each stage, the result against the injected ground truth, in
 * physical units. One command, all stages, deterministic.
 *
 * HONEST FRAMING: this is the firmware LOGIC + full chain, verified on the host
 * against SYNTHETIC IQ ground truth. It has NOT been run on real RF hardware.
 * The chain is identical to the on-target build — only the adc_source (synthetic
 * vs real ADC) and the FFT backend (portable vs HWA) differ behind their
 * abstractions. See PORTABILITY.md for the IWR6843 mapping.
 *
 * The pipeline_frame() function below IS the per-frame data path; on-target it is
 * driven by the SDK frame-done interrupt instead of this host loop.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "radar_config.h"
#include "adc_source.h"
#include "range_proc.h"
#include "doppler_proc.h"
#include "cfar.h"
#include "aoa.h"
#include "cluster.h"
#include "tracker.h"
#include "zones.h"
#include "node_output.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { \
    int ok_ = (cond); printf("  [%s] ", ok_ ? "PASS" : "FAIL"); \
    printf(__VA_ARGS__); printf("\n"); if (!ok_) g_fail++; \
} while (0)

#define MAXPTS 30000

/* working buffers for one pipeline instance */
typedef struct {
    cplx *frame, *rc, *rd;
    float *map;
    detection_t *det;
    radar_point_t *pts;
} bufs_t;

static void bufs_alloc(bufs_t *b) {
    b->frame = malloc(sizeof(cplx) * SYNTH_FRAME_LEN);
    b->rc = malloc(sizeof(cplx) * RANGE_CUBE_LEN);
    b->rd = malloc(sizeof(cplx) * RD_CUBE_LEN);
    b->map = malloc(sizeof(float) * RD_MAP_LEN);
    b->det = malloc(sizeof(detection_t) * MAXPTS);
    b->pts = malloc(sizeof(radar_point_t) * MAXPTS);
}
static void bufs_free(bufs_t *b) {
    free(b->frame); free(b->rc); free(b->rd); free(b->map); free(b->det); free(b->pts);
}

/* THE per-frame data path (identical host/target). Returns objects this frame. */
static int pipeline_frame(bufs_t *b, adc_source *src, object_t *objs, int max_objs,
                          int *out_ndet) {
    src->fill(src, b->frame);                       /* ADC seam: synth (host)   */
    range_proc_run(b->frame, b->rc);                /* range-FFT  (HWA)         */
    doppler_proc_run(b->rc, b->rd);                 /* Doppler-FFT (HWA)        */
    doppler_proc_power_map(b->rd, b->map);          /* non-coherent integration */
    int nd = cfar_detect(b->map, CFAR_CA, b->det, MAXPTS);   /* detection       */
    for (int i = 0; i < nd; ++i)
        b->pts[i] = aoa_make_point(b->rd, &b->det[i]);       /* angle -> points  */
    if (out_ndet) *out_ndet = nd;
    return cluster_dbscan(b->pts, nd, objs, max_objs);       /* objects          */
}

/* ---- the scene: 3 vehicles (one moving) + 1 pedestrian, over time ---- */
#define N_FRAMES 8
static int fill_scene(int f, target_t *t) {
    double dt = RC_FRAME_INTERVAL;
    /* veh A: approach_E, approaching (range shrinks across frames) */
    t[0] = (target_t){ 55.0 - 12.0 * dt * f, -12.0, +20.0, 1.0 };
    /* veh B: approach_E */
    t[1] = (target_t){ 40.0,  +8.0, +35.0, 1.0 };
    /* veh C: approach_W */
    t[2] = (target_t){ 45.0, +10.0, -55.0, 1.0 };
    /* pedestrian: approach_S, slow + weak */
    t[3] = (target_t){ 16.0,  +1.0, -15.0, 0.4 };
    return 4;
}

static int range_bin_of(double R) { return (int)lround(R / rc_range_of_bin(1)); }
static int dopp_bin_of(double v) {
    int s = (int)lround(v / RC_VEL_RES);
    return ((s % RC_DOPPLER_FFT) + RC_DOPPLER_FFT) % RC_DOPPLER_FFT;
}
static int dopp_dist(int a, int b) {
    int dd = abs(a - b); return dd > RC_DOPPLER_FFT / 2 ? RC_DOPPLER_FFT - dd : dd;
}

/* run the whole scene; do asserts on the first pass; return final JSON */
static void run_scene(int do_asserts, char *json_out, int json_len) {
    bufs_t b; bufs_alloc(&b);
    object_t objs[TRACK_MAX];
    tracker_t trk; tracker_init(&trk);
    target_t scene[8];
    int a_id = -1;

    node_output_t out;
    for (int f = 0; f < N_FRAMES; ++f) {
        int n = fill_scene(f, scene);
        adc_synth_ctx ctx = { scene, n, 0.5, 500 + f, 0 };  /* fixed seeds -> deterministic */
        adc_source src = adc_source_synth(&ctx);
        int ndet = 0;
        int nobj = pipeline_frame(&b, &src, objs, TRACK_MAX, &ndet);
        tracker_update(&trk, objs, nobj, RC_FRAME_INTERVAL);

        if (do_asserts && f == 0) {
            /* stage spot-check: target A recovered in physical units */
            int tr = range_bin_of(scene[0].range_m), tdv = dopp_bin_of(scene[0].velocity_mps);
            detection_t best = { 0, 0, -1 };
            for (int i = 0; i < ndet; ++i)
                if (abs(b.det[i].range_bin - tr) <= 3 && dopp_dist(b.det[i].doppler_bin, tdv) <= 3
                    && b.det[i].power > best.power) best = b.det[i];
            radar_point_t p = aoa_make_point(b.rd, &best);
            CHECK(fabs(p.range_m - scene[0].range_m) < 1.0 &&
                  fabs(p.velocity_mps - scene[0].velocity_mps) < 1.0 &&
                  fabs(p.azimuth_deg - scene[0].azimuth_deg) < 5.0,
                  "stages range/Doppler/AoA: target A truth(R=%.1f v=%+.1f az=%+.1f) "
                  "-> (R=%.2f v=%+.2f az=%+.2f)",
                  scene[0].range_m, scene[0].velocity_mps, scene[0].azimuth_deg,
                  p.range_m, p.velocity_mps, p.azimuth_deg);
            CHECK(nobj == 4, "stage cluster: %d detections -> %d objects (expected 4)", ndet, nobj);
        }
        if (do_asserts) {
            /* track persistence: the moving vehicle A keeps one stable id */
            track_t *ta = NULL; double bestd = 8.0;
            double R = scene[0].range_m;
            for (int i = 0; i < trk.ntracks; ++i) {
                if (!trk.tracks[i].active) continue;
                double d = fabs(trk.tracks[i].range_m - R) + fabs(trk.tracks[i].azimuth_deg - 20.0);
                if (d < bestd) { bestd = d; ta = &trk.tracks[i]; }
            }
            if (ta) {
                if (a_id < 0) a_id = ta->id;
                if (f == N_FRAMES - 1)
                    CHECK(ta->id == a_id && fabs(ta->velocity_mps - (-12.0)) < 1.5,
                          "stage tracker: vehicle A one persistent track (id=%d) "
                          "follows to R=%.1f with v=%+.2f m/s", a_id, ta->range_m, ta->velocity_mps);
            }
        }

        if (f == N_FRAMES - 1)
            zones_build_report(&trk, (unsigned)f, f * RC_FRAME_INTERVAL, &out);
    }

    if (do_asserts) {
        int zE = zones_assign(20, 55), zW = zones_assign(-55, 45),
            zS = zones_assign(-15, 16), zN = zones_assign(70, 30);
        printf("\n  final node output (the brain's input):\n");
        node_output_print(&out, stdout);
        CHECK(out.approaches[zE].vehicle_count == 2, "stage zones: approach_E has 2 vehicles (got %d)",
              out.approaches[zE].vehicle_count);
        CHECK(out.approaches[zW].vehicle_count == 1, "stage zones: approach_W has 1 vehicle (got %d)",
              out.approaches[zW].vehicle_count);
        CHECK(out.approaches[zS].pedestrian_count == 1 && out.approaches[zS].vehicle_count == 0,
              "stage classify: approach_S has 1 pedestrian, 0 vehicles (got %d ped, %d veh)",
              out.approaches[zS].pedestrian_count, out.approaches[zS].vehicle_count);
        CHECK(out.approaches[zN].vehicle_count == 0 && out.approaches[zN].pedestrian_count == 0,
              "stage zones: approach_N empty");
    }

    node_output_to_json(&out, json_out, json_len);
    bufs_free(&b);
}

int main(void) {
    printf("=== mmWave radar node - END-TO-END host verification ===\n");
    printf("(firmware logic + full FMCW chain, verified on synthetic IQ; not run on RF hardware)\n\n");

    radar_config_validate();
    CHECK(1, "config self-consistency asserts passed");
    radar_config_print();

    printf("\nrunning %d-frame synthetic scene through the full chain:\n", N_FRAMES);
    char json1[2048], json2[2048];
    run_scene(1, json1, sizeof(json1));

    /* determinism: a second identical run must produce identical output */
    run_scene(0, json2, sizeof(json2));
    CHECK(strcmp(json1, json2) == 0, "deterministic: identical output on re-run");

    printf("\n  node output JSON (sensing->brain contract):\n  %s\n", json1);

    printf("\n=== END-TO-END GATE: %s (%d failures) ===\n",
           g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
