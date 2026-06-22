/*
 * test_phase5.c — Phase 5 host verification: clustering + tracking.
 *
 * (A) One target produces many CFAR detections -> DBSCAN collapses them to ONE
 *     object at the right place. Three separated targets -> three objects.
 * (B) A target moving radially across frames -> the tracker keeps ONE persistent
 *     track that follows its position and reports the correct velocity.
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
#include "../cluster.h"
#include "../tracker.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { \
    int ok_ = (cond); printf("  [%s] ", ok_ ? "PASS" : "FAIL"); \
    printf(__VA_ARGS__); printf("\n"); if (!ok_) g_fail++; \
} while (0)

#define MAXPTS 30000

typedef struct { cplx *frame, *rc, *rd; float *map; detection_t *det; radar_point_t *pts; } bufs_t;

/* full chain: scene (+noise) -> CFAR detections -> (range,vel,azimuth) points */
static int chain_to_points(bufs_t *b, const target_t *tg, int ntg,
                           double sigma, uint32_t seed) {
    adc_synth_ctx ctx = { tg, ntg, sigma, seed, 0 };
    adc_source src = adc_source_synth(&ctx);
    src.fill(&src, b->frame);
    range_proc_run(b->frame, b->rc);
    doppler_proc_run(b->rc, b->rd);
    doppler_proc_power_map(b->rd, b->map);
    int nd = cfar_detect(b->map, CFAR_CA, b->det, MAXPTS);
    for (int i = 0; i < nd; ++i) b->pts[i] = aoa_make_point(b->rd, &b->det[i]);
    return nd;
}

int main(void) {
    printf("=== PHASE 5 - clustering + tracking ===\n\n");

    bufs_t b;
    b.frame = malloc(sizeof(cplx) * SYNTH_FRAME_LEN);
    b.rc = malloc(sizeof(cplx) * RANGE_CUBE_LEN);
    b.rd = malloc(sizeof(cplx) * RD_CUBE_LEN);
    b.map = malloc(sizeof(float) * RD_MAP_LEN);
    b.det = malloc(sizeof(detection_t) * MAXPTS);
    b.pts = malloc(sizeof(radar_point_t) * MAXPTS);
    object_t objs[TRACK_MAX];

    /* ---------- (A1) many detections collapse to one object ---------- */
    printf("(A1) one target -> many detections -> one object:\n");
    target_t one[] = { { 40.0, +6.0, +10.0, 1.0 } };
    int npts = chain_to_points(&b, one, 1, 0.5, 11);
    int nobj = cluster_dbscan(b.pts, npts, objs, TRACK_MAX);
    CHECK(nobj == 1, "%d CFAR detections collapsed to %d object", npts, nobj);
    if (nobj >= 1) {
        CHECK(objs[0].npoints > 3, "object built from %d detections (many->one)", objs[0].npoints);
        CHECK(fabs(objs[0].range_m - 40.0) < 1.0 &&
              fabs(objs[0].velocity_mps - 6.0) < 1.0 &&
              fabs(objs[0].azimuth_deg - 10.0) < 5.0,
              "object centroid (R=%.2f v=%+.2f az=%+.2f) vs truth (40,+6,+10)",
              objs[0].range_m, objs[0].velocity_mps, objs[0].azimuth_deg);
    }

    /* ---------- (A2) separated targets -> distinct objects ---------- */
    printf("\n(A2) three separated targets -> three objects:\n");
    target_t three[] = {
        { 25.0, +10.0,   0.0, 1.0 },
        { 55.0,  -8.0, +25.0, 1.0 },
        { 80.0,  +5.0, -20.0, 1.0 },
    };
    npts = chain_to_points(&b, three, 3, 0.5, 22);
    nobj = cluster_dbscan(b.pts, npts, objs, TRACK_MAX);
    CHECK(nobj == 3, "%d detections -> %d objects (expected 3)", npts, nobj);
    for (int o = 0; o < nobj; ++o)
        printf("       object %d: R=%.1f m  v=%+.1f m/s  az=%+.1f deg  (%d dets)\n",
               o, objs[o].range_m, objs[o].velocity_mps, objs[o].azimuth_deg, objs[o].npoints);

    /* ---------- (B) moving target -> one persistent track ---------- */
    printf("\n(B) target moving radially across frames -> one persistent track:\n");
    const double R0 = 50.0, vtrue = -12.0, az = +10.0, dt = RC_FRAME_INTERVAL;
    const int FRAMES = 12;
    tracker_t trk; tracker_init(&trk);
    int first_id = -1, id_stable = 1, always_one = 1;

    printf("    frame |  true R   track R |  track v (Doppler / dR-dt) | id\n");
    for (int f = 0; f < FRAMES; ++f) {
        double R = R0 + vtrue * dt * f;          /* radial motion           */
        target_t mv[] = { { R, vtrue, az, 1.0 } };
        npts = chain_to_points(&b, mv, 1, 0.5, 100 + f);
        nobj = cluster_dbscan(b.pts, npts, objs, TRACK_MAX);
        tracker_update(&trk, objs, nobj, dt);

        if (tracker_active_count(&trk) != 1) always_one = 0;
        /* the single active track */
        track_t *t = NULL;
        for (int i = 0; i < trk.ntracks; ++i) if (trk.tracks[i].active) { t = &trk.tracks[i]; break; }
        if (!t) { always_one = 0; continue; }
        if (first_id < 0) first_id = t->id;
        if (t->id != first_id) id_stable = 0;

        printf("    %3d   | %6.2f   %6.2f  | %+7.2f / %+7.2f          | %d\n",
               f, R, t->range_m, t->velocity_mps, t->range_rate, t->id);

        /* track position must follow the moving target every frame (after lock) */
        if (f >= 1)
            CHECK(fabs(t->range_m - R) < 1.5,
                  "frame %2d track range %.2f follows truth %.2f", f, t->range_m, R);
    }

    CHECK(always_one, "exactly ONE active track maintained every frame");
    CHECK(id_stable, "track id stayed constant (id=%d) -> persistent, not re-created", first_id);

    /* final velocity correct, from both the Doppler measurement and cross-frame dR/dt */
    track_t *tf = NULL;
    for (int i = 0; i < trk.ntracks; ++i) if (trk.tracks[i].active) { tf = &trk.tracks[i]; break; }
    if (tf) {
        CHECK(fabs(tf->velocity_mps - vtrue) < 1.5,
              "tracked velocity (Doppler) %+.2f vs truth %+.1f m/s", tf->velocity_mps, vtrue);
        CHECK(fabs(tf->range_rate - vtrue) < 2.0,
              "tracked velocity (cross-frame dR/dt) %+.2f vs truth %+.1f m/s", tf->range_rate, vtrue);
    }

    free(b.frame); free(b.rc); free(b.rd); free(b.map); free(b.det); free(b.pts);
    printf("\n=== PHASE 5 GATE: %s (%d failures) ===\n",
           g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
