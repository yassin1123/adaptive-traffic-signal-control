/*
 * node_output.c — render the per-approach report (human + JSON wire format).
 */
#include <string.h>
#include "node_output.h"

void node_output_print(const node_output_t *o, FILE *f) {
    fprintf(f, "  node output  frame=%u  t=%.3f s\n", o->frame_id, o->timestamp_s);
    for (int i = 0; i < o->num_approaches; ++i) {
        const approach_report_t *a = &o->approaches[i];
        fprintf(f, "    %-12s vehicles=%d  pedestrians=%d  avg_speed=%.1f m/s\n",
                a->name, a->vehicle_count, a->pedestrian_count, a->avg_speed_mps);
    }
}

int node_output_to_json(const node_output_t *o, char *buf, int buflen) {
    int n = 0;
    #define EMIT(...) do { \
        int w = snprintf(buf + n, buflen - n, __VA_ARGS__); \
        if (w < 0 || w >= buflen - n) return -1; \
        n += w; \
    } while (0)

    EMIT("{\"frame_id\":%u,\"timestamp_s\":%.3f,\"approaches\":[",
         o->frame_id, o->timestamp_s);
    for (int i = 0; i < o->num_approaches; ++i) {
        const approach_report_t *a = &o->approaches[i];
        EMIT("%s{\"approach\":\"%s\",\"zone\":%d,\"vehicle_count\":%d,"
             "\"pedestrian_count\":%d,\"avg_speed_mps\":%.2f}",
             i ? "," : "", a->name, a->zone_id,
             a->vehicle_count, a->pedestrian_count, a->avg_speed_mps);
    }
    EMIT("]}");
    #undef EMIT
    return n;
}
