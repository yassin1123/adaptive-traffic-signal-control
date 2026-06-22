/*
 * test_phase6.c — Phase 6 host verification: zones, counting, classification, output.
 *
 * Inject a realistic scene (vehicles on several approaches, a stopped vehicle, a
 * slow pedestrian), run the full chain + tracker, then build the per-approach
 * report and assert the counts, speeds, and pedestrian classification match the
 * injected truth. Emits the node output (human + JSON) — the sensing->brain
 * contract — and writes node_output.json for the integration check.
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
#include "../zones.h"
#include "../node_output.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { \
    int ok_ = (cond); printf("  [%s] ", ok_ ? "PASS" : "FAIL"); \
    printf(__VA_ARGS__); printf("\n"); if (!ok_) g_fail++; \
} while (0)

#define MAXPTS 30000

int main(void) {
    printf("=== PHASE 6 - zones / counting / classification / output ===\n\n");

    cplx *frame = malloc(sizeof(cplx) * SYNTH_FRAME_LEN);
    cplx *rc = malloc(sizeof(cplx) * RANGE_CUBE_LEN);
    cplx *rd = malloc(sizeof(cplx) * RD_CUBE_LEN);
    float *map = malloc(sizeof(float) * RD_MAP_LEN);
    detection_t *det = malloc(sizeof(detection_t) * MAXPTS);
    radar_point_t *pts = malloc(sizeof(radar_point_t) * MAXPTS);
    object_t objs[TRACK_MAX];

    /* realistic scene (range, radial velocity, azimuth, rcs/amplitude) */
    target_t scene[] = {
        { 35.0, +12.0, +15.0, 1.0 },   /* vehicle  -> approach_E            */
        { 50.0, -10.0, +30.0, 1.0 },   /* vehicle  -> approach_E            */
        { 45.0,  +9.0, -60.0, 1.0 },   /* vehicle  -> approach_W            */
        { 20.0,  +0.4,  +5.0, 1.2 },   /* STOPPED vehicle (slow, big) -> E  */
        { 14.0,  +1.2, -10.0, 0.4 },   /* PEDESTRIAN (slow, small) -> S     */
    };
    int nt = 5;

    /* run a few frames of the static scene so tracks lock in */
    tracker_t trk; tracker_init(&trk);
    for (int f = 0; f < 3; ++f) {
        adc_synth_ctx ctx = { scene, nt, 0.5, 700 + f, 0 };
        adc_source src = adc_source_synth(&ctx);
        src.fill(&src, frame);
        range_proc_run(frame, rc);
        doppler_proc_run(rc, rd);
        doppler_proc_power_map(rd, map);
        int nd = cfar_detect(map, CFAR_CA, det, MAXPTS);
        for (int i = 0; i < nd; ++i) pts[i] = aoa_make_point(rd, &det[i]);
        int nobj = cluster_dbscan(pts, nd, objs, TRACK_MAX);
        tracker_update(&trk, objs, nobj, RC_FRAME_INTERVAL);
        (void)nobj;
    }

    /* per-track diagnostic: class + size, to show classification at work */
    printf("  tracks (id, approach, class, speed, cluster-size):\n");
    for (int i = 0; i < trk.ntracks; ++i) {
        track_t *t = &trk.tracks[i];
        if (!t->active) continue;
        int z = zones_assign(t->azimuth_deg, t->range_m);
        obj_class_t cl = zones_classify(t);
        printf("    id=%d  %-11s  %-10s  speed=%5.1f m/s  npts=%d  power=%.3g\n",
               t->id, z >= 0 ? "zone" : "outside",
               cl == CLASS_PEDESTRIAN ? "PEDESTRIAN" : "vehicle",
               fabs(t->velocity_mps), t->npoints, (double)t->power);
    }
    printf("\n");

    /* build + show the node output (the brain's input) */
    node_output_t out;
    zones_build_report(&trk, 42, 42 * RC_FRAME_INTERVAL, &out);
    node_output_print(&out, stdout);
    printf("\n");

    /* asserts vs injected truth */
    int zE = zones_assign(15.0, 35.0);   /* approach_E index */
    int zW = zones_assign(-60.0, 45.0);  /* approach_W index */
    int zS = zones_assign(-10.0, 14.0);  /* approach_S index */
    int zN = zones_assign(+70.0, 30.0);  /* approach_N index */
    CHECK(out.approaches[zE].vehicle_count == 3 && out.approaches[zE].pedestrian_count == 0,
          "approach_E: 3 vehicles, 0 peds (got %d veh, %d ped)",
          out.approaches[zE].vehicle_count, out.approaches[zE].pedestrian_count);
    CHECK(out.approaches[zW].vehicle_count == 1,
          "approach_W: 1 vehicle (got %d)", out.approaches[zW].vehicle_count);
    CHECK(out.approaches[zS].pedestrian_count == 1 && out.approaches[zS].vehicle_count == 0,
          "approach_S: 1 pedestrian, 0 vehicles (got %d ped, %d veh)",
          out.approaches[zS].pedestrian_count, out.approaches[zS].vehicle_count);
    CHECK(out.approaches[zN].vehicle_count == 0 && out.approaches[zN].pedestrian_count == 0,
          "approach_N: empty");
    CHECK(out.approaches[zW].avg_speed_mps > 7.5 && out.approaches[zW].avg_speed_mps < 10.5,
          "approach_W avg vehicle speed ~9 m/s (got %.1f)", out.approaches[zW].avg_speed_mps);

    /* emit the JSON wire format and save it for the brain-contract check */
    char json[2048];
    int jn = node_output_to_json(&out, json, sizeof(json));
    CHECK(jn > 0, "node output serialised to JSON (%d bytes)", jn);
    printf("\n  JSON (sensing->brain contract):\n  %s\n", json);
    FILE *jf = fopen("node_output.json", "w");
    if (jf) { fprintf(jf, "%s\n", json); fclose(jf); }

    free(frame); free(rc); free(rd); free(map); free(det); free(pts);
    printf("\n=== PHASE 6 GATE: %s (%d failures) ===\n",
           g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
