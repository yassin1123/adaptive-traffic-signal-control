/*
 * tracker.c — nearest-neighbour association + alpha-filter state update.
 */
#include <math.h>
#include "tracker.h"

void tracker_init(tracker_t *trk) {
    trk->ntracks = 0;
    trk->next_id = 1;
}

int tracker_active_count(const tracker_t *trk) {
    int c = 0;
    for (int i = 0; i < trk->ntracks; ++i) if (trk->tracks[i].active) c++;
    return c;
}

static void spawn(tracker_t *trk, const object_t *o) {
    if (trk->ntracks >= TRACK_MAX) return;
    track_t *t = &trk->tracks[trk->ntracks++];
    t->id = trk->next_id++;
    t->active = 1; t->age = 1; t->miss = 0;
    t->range_m = o->range_m; t->azimuth_deg = o->azimuth_deg;
    t->velocity_mps = o->velocity_mps; t->range_rate = o->velocity_mps;
    t->prev_range = o->range_m;
    t->x = o->x; t->y = o->y;
    t->npoints = o->npoints; t->power = o->power;
}

void tracker_update(tracker_t *trk, const object_t *objs, int nobj, double dt) {
    const double a = RC_TRACK_ALPHA;
    int matched_obj[64] = { 0 };

    /* 1) associate each active track to its nearest unmatched object within gate,
     *    comparing against the track's PREDICTED position (constant-velocity). */
    for (int i = 0; i < trk->ntracks; ++i) {
        track_t *t = &trk->tracks[i];
        if (!t->active) continue;
        double pred_r = t->range_m + t->velocity_mps * dt;     /* predict range  */
        double ar = t->azimuth_deg * RC_PI / 180.0;
        double px = pred_r * sin(ar), py = pred_r * cos(ar);

        int best = -1; double bestd = RC_TRACK_GATE_M;
        for (int j = 0; j < nobj && j < 64; ++j) {
            if (matched_obj[j]) continue;
            double d = hypot(objs[j].x - px, objs[j].y - py);
            if (d < bestd) { bestd = d; best = j; }
        }
        if (best >= 0) {
            const object_t *o = &objs[best];
            matched_obj[best] = 1;
            /* cross-frame range rate, independent of the Doppler measurement */
            double rr = (o->range_m - t->prev_range) / dt;
            t->prev_range = o->range_m;
            t->range_m     = a * o->range_m     + (1 - a) * pred_r;
            t->azimuth_deg = a * o->azimuth_deg + (1 - a) * t->azimuth_deg;
            t->velocity_mps = a * o->velocity_mps + (1 - a) * t->velocity_mps;
            t->range_rate  = a * rr             + (1 - a) * t->range_rate;
            double arn = t->azimuth_deg * RC_PI / 180.0;
            t->x = t->range_m * sin(arn);
            t->y = t->range_m * cos(arn);
            t->npoints = o->npoints; t->power = o->power;
            t->age++; t->miss = 0;
        } else {
            t->miss++;
            if (t->miss > TRACK_MAX_MISS) t->active = 0;
        }
    }

    /* 2) unmatched objects spawn new tracks */
    for (int j = 0; j < nobj && j < 64; ++j)
        if (!matched_obj[j]) spawn(trk, &objs[j]);
}
