/*
 * cluster.h — DBSCAN-style grouping of detection points into objects.
 *
 * One physical target produces MANY detections (CFAR fires on a spread of
 * range-Doppler cells; an extended vehicle adds more). DBSCAN groups nearby
 * points in (range, velocity, azimuth) space into one object and, as a bonus,
 * drops isolated points (stray false alarms) as noise. Each axis is divided by
 * its own scale before measuring distance, so eps is a single unitless radius
 * (config: RC_CLUSTER_EPS, RC_CLUSTER_MIN_PTS, RC_CLUSTER_SCALE_*).
 *
 * This is MSS post-processing on-target.
 */
#ifndef CLUSTER_H
#define CLUSTER_H

#include "aoa.h"      /* radar_point_t */

typedef struct {
    double range_m;
    double velocity_mps;
    double azimuth_deg;
    double x, y;        /* Cartesian: x = R*sin(az), y = R*cos(az) (boresight +y) */
    float  power;       /* summed cluster power                                   */
    int    npoints;     /* detections collapsed into this object                  */
} object_t;

/* Cluster `n` points into objects; writes up to `max_objs`, returns count. */
int cluster_dbscan(const radar_point_t *pts, int n, object_t *objs, int max_objs);

#endif /* CLUSTER_H */
