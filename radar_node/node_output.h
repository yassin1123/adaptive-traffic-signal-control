/*
 * node_output.h — the sensing -> brain CONTRACT.
 *
 * This is the node's only externally-visible output: per intersection approach,
 * how many vehicles and pedestrians are present and the vehicles' average speed.
 * That is exactly what the Python control "brain" ingests to fill its per-approach
 * queue counts and decide signal timing (the brain consumes per-approach demand;
 * vehicle_count -> queue length, pedestrian_count -> pedestrian demand,
 * avg_speed_mps -> approach speed). Transport (how the message reaches the brain)
 * is out of scope; here we define the message and serialise it as JSON, which the
 * Python side parses directly.
 */
#ifndef NODE_OUTPUT_H
#define NODE_OUTPUT_H

#include <stdio.h>
#include "radar_config.h"

typedef struct {
    int         zone_id;
    const char *name;            /* approach label                          */
    int         vehicle_count;
    int         pedestrian_count;
    double      avg_speed_mps;   /* mean |speed| of vehicles on this approach */
} approach_report_t;

typedef struct {
    unsigned          frame_id;
    double            timestamp_s;
    int               num_approaches;
    approach_report_t approaches[RC_NUM_ZONES];
} node_output_t;

/* Human-readable dump. */
void node_output_print(const node_output_t *o, FILE *f);

/* Serialise to the wire format the brain consumes (JSON). Returns bytes written
 * (excluding NUL), or -1 if the buffer is too small. */
int node_output_to_json(const node_output_t *o, char *buf, int buflen);

#endif /* NODE_OUTPUT_H */
