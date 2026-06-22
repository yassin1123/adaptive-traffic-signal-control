/*
 * tracker.h — frame-to-frame tracking of clustered objects.
 *
 * Clustering gives one object per target per frame, but those measurements are
 * noisy and momentary. The tracker associates each frame's objects to existing
 * tracks (nearest-neighbour within a Cartesian gate), then updates a smoothed
 * state (alpha filter) so each real target becomes ONE stable, persistent track
 * with a steady position and velocity. New objects spawn tracks; tracks that go
 * unseen for a few frames are dropped.
 *
 * Config: RC_TRACK_GATE_M (association radius), RC_TRACK_ALPHA (smoothing).
 * This is MSS control-layer processing on-target.
 */
#ifndef TRACKER_H
#define TRACKER_H

#include "cluster.h"

#define TRACK_MAX       32
#define TRACK_MAX_MISS  3      /* drop a track after this many unseen frames */

typedef struct {
    int    id;
    int    active;
    int    age;                /* frames matched                            */
    int    miss;               /* consecutive frames unmatched              */
    double x, y;               /* smoothed Cartesian position               */
    double range_m;            /* smoothed range                            */
    double azimuth_deg;        /* smoothed azimuth                          */
    double velocity_mps;       /* smoothed radial velocity (from Doppler)   */
    double range_rate;         /* cross-frame dR/dt estimate (independent)  */
    double prev_range;
    int    npoints;            /* latest cluster size (detections) -> extent */
    float  power;              /* latest cluster power -> RCS proxy          */
} track_t;

typedef struct {
    track_t tracks[TRACK_MAX];
    int     ntracks;
    int     next_id;
} tracker_t;

void tracker_init(tracker_t *trk);

/* Update with this frame's `objs`; `dt` is the time since the last frame. */
void tracker_update(tracker_t *trk, const object_t *objs, int nobj, double dt);

/* Number of currently-active tracks. */
int tracker_active_count(const tracker_t *trk);

#endif /* TRACKER_H */
