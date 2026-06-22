/*
 * zones.h — map tracked objects to approaches, classify, and count.
 *
 * The last stage: each stable track is assigned to an intersection approach by
 * its azimuth sector, classified vehicle vs pedestrian (speed + cluster size),
 * and tallied into the per-approach report the brain consumes.
 */
#ifndef ZONES_H
#define ZONES_H

#include "tracker.h"
#include "node_output.h"

typedef enum { CLASS_VEHICLE = 0, CLASS_PEDESTRIAN = 1 } obj_class_t;

/* Classify a track. Pedestrian iff slow AND a small radar object (see config). */
obj_class_t zones_classify(const track_t *t);

/* Approach (zone) id for an azimuth/range, or -1 if outside all zones. */
int zones_assign(double azimuth_deg, double range_m);

/* Build the per-approach report from the current active tracks. */
void zones_build_report(const tracker_t *trk, unsigned frame_id, double ts,
                        node_output_t *out);

#endif /* ZONES_H */
