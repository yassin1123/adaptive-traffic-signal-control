/*
 * cluster.c — DBSCAN over the detection point cloud.
 *
 * Standard DBSCAN: a point is "core" if it has >= MIN_PTS neighbours within eps
 * (scaled distance); clusters grow from core points through density-reachable
 * neighbours; points reachable from no core are noise. Centroids are
 * power-weighted (stronger returns pull the object centre).
 */
#include <math.h>
#include <stdlib.h>
#include "cluster.h"

#define UNVISITED -2
#define NOISE     -1

static double pdist(const radar_point_t *a, const radar_point_t *b) {
    double dr = (a->range_m - b->range_m) / RC_CLUSTER_SCALE_RANGE;
    double dv = (a->velocity_mps - b->velocity_mps) / RC_CLUSTER_SCALE_VEL;
    double da = (a->azimuth_deg - b->azimuth_deg) / RC_CLUSTER_SCALE_ANGLE;
    return sqrt(dr * dr + dv * dv + da * da);
}

/* indices of points within eps of point i (including i) */
static int region(const radar_point_t *pts, int n, int i, int *out) {
    int c = 0;
    for (int j = 0; j < n; ++j)
        if (pdist(&pts[i], &pts[j]) <= RC_CLUSTER_EPS)
            out[c++] = j;
    return c;
}

int cluster_dbscan(const radar_point_t *pts, int n, object_t *objs, int max_objs) {
    if (n <= 0) return 0;
    int *label = malloc(sizeof(int) * n);
    int *neigh = malloc(sizeof(int) * n);
    int *seed = malloc(sizeof(int) * n);
    char *inseed = malloc(n);                            /* queued guard       */
    for (int i = 0; i < n; ++i) { label[i] = UNVISITED; inseed[i] = 0; }

    int cid = 0;
    for (int i = 0; i < n; ++i) {
        if (label[i] != UNVISITED) continue;
        int nc = region(pts, n, i, neigh);
        if (nc < RC_CLUSTER_MIN_PTS) { label[i] = NOISE; continue; }

        label[i] = cid;
        int ns = 0;
        /* enqueue each neighbour at most once -> ns can never exceed n */
        for (int q = 0; q < nc; ++q) {
            int nn = neigh[q];
            if (nn != i && !inseed[nn]) { seed[ns++] = nn; inseed[nn] = 1; }
        }
        for (int s = 0; s < ns; ++s) {
            int j = seed[s];
            if (label[j] == NOISE) label[j] = cid;        /* border point      */
            if (label[j] != UNVISITED) continue;
            label[j] = cid;
            int mj = region(pts, n, j, neigh);
            if (mj >= RC_CLUSTER_MIN_PTS)                 /* j is core -> expand */
                for (int q = 0; q < mj; ++q) {
                    int nn = neigh[q];
                    if (!inseed[nn]) { seed[ns++] = nn; inseed[nn] = 1; }
                }
        }
        cid++;
    }

    /* summarise clusters into power-weighted objects */
    int nobj = cid < max_objs ? cid : max_objs;
    for (int c = 0; c < nobj; ++c) {
        double sw = 0, sr = 0, sv = 0, sa = 0; int np = 0;
        for (int i = 0; i < n; ++i) {
            if (label[i] != c) continue;
            double w = pts[i].power;
            sw += w; sr += w * pts[i].range_m;
            sv += w * pts[i].velocity_mps; sa += w * pts[i].azimuth_deg; np++;
        }
        if (sw <= 0) sw = 1;
        object_t *o = &objs[c];
        o->range_m = sr / sw; o->velocity_mps = sv / sw; o->azimuth_deg = sa / sw;
        o->power = (float)sw; o->npoints = np;
        double ar = o->azimuth_deg * RC_PI / 180.0;
        o->x = o->range_m * sin(ar);
        o->y = o->range_m * cos(ar);
    }

    free(label); free(neigh); free(seed); free(inseed);
    return nobj;
}
