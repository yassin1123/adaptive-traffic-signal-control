/*
 * zones.c — approach assignment, vehicle/pedestrian classification, counting.
 */
#include <math.h>
#include "zones.h"

static const double zedges[RC_NUM_ZONES + 1] = RC_ZONE_AZ_EDGES;
static const char  *znames[RC_NUM_ZONES]     = RC_ZONE_NAMES;

obj_class_t zones_classify(const track_t *t) {
    double speed = fabs(t->velocity_mps);
    int weak = (t->power < RC_VEHICLE_RCS_MIN);    /* small RCS */
    /* pedestrian = slow AND weak. A queued/stopped vehicle is slow but strong, so
     * it stays a vehicle; a moving object is a vehicle regardless of RCS. */
    if (speed < RC_PED_SPEED_MAX && weak)
        return CLASS_PEDESTRIAN;
    return CLASS_VEHICLE;
}

int zones_assign(double azimuth_deg, double range_m) {
    if (range_m <= 0.0 || range_m > RC_MAX_RANGE) return -1;
    for (int z = 0; z < RC_NUM_ZONES; ++z)
        if (azimuth_deg >= zedges[z] && azimuth_deg < zedges[z + 1])
            return z;
    return -1;
}

void zones_build_report(const tracker_t *trk, unsigned frame_id, double ts,
                        node_output_t *out) {
    out->frame_id = frame_id;
    out->timestamp_s = ts;
    out->num_approaches = RC_NUM_ZONES;
    double speed_sum[RC_NUM_ZONES] = { 0 };
    for (int z = 0; z < RC_NUM_ZONES; ++z) {
        out->approaches[z].zone_id = z;
        out->approaches[z].name = znames[z];
        out->approaches[z].vehicle_count = 0;
        out->approaches[z].pedestrian_count = 0;
        out->approaches[z].avg_speed_mps = 0.0;
    }

    for (int i = 0; i < trk->ntracks; ++i) {
        const track_t *t = &trk->tracks[i];
        if (!t->active) continue;
        int z = zones_assign(t->azimuth_deg, t->range_m);
        if (z < 0) continue;
        if (zones_classify(t) == CLASS_PEDESTRIAN) {
            out->approaches[z].pedestrian_count++;
        } else {
            out->approaches[z].vehicle_count++;
            speed_sum[z] += fabs(t->velocity_mps);
        }
    }
    for (int z = 0; z < RC_NUM_ZONES; ++z)
        if (out->approaches[z].vehicle_count > 0)
            out->approaches[z].avg_speed_mps =
                speed_sum[z] / out->approaches[z].vehicle_count;
}
