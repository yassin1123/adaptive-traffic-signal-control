/*
 * cfar.c — CA-CFAR and OS-CFAR over the 2-D range-Doppler power map.
 */
#include <math.h>
#include <stdlib.h>
#include "cfar.h"
#include "doppler_proc.h"   /* rd_map_idx, RD_MAP_LEN */

#define WIN  (RC_CFAR_GUARD + RC_CFAR_TRAIN)        /* window half-width per dim */
#define WMAX ((2 * WIN + 1) * (2 * WIN + 1))         /* max cells in the window  */

int cfar_num_training(void) {
    int outer = (2 * WIN + 1) * (2 * WIN + 1);
    int guard = (2 * RC_CFAR_GUARD + 1) * (2 * RC_CFAR_GUARD + 1);
    return outer - guard;                            /* excludes guard box + CUT */
}

/* CA-CFAR multiplier for a target Pfa with N training cells (square-law /
 * exponential-power noise): alpha = N * (Pfa^(-1/N) - 1). */
double cfar_alpha_ca(int n_train) {
    return n_train * (pow(RC_CFAR_PFA, -1.0 / n_train) - 1.0);
}

/* OS-CFAR multiplier: solve  prod_{i=0}^{k-1} (N-i)/(N-i+alpha) = Pfa  for alpha
 * (the product is monotonically decreasing in alpha) by bisection. No external
 * solver needed — a short, deterministic loop. */
double cfar_alpha_os(int n_train, int k_rank) {
    double lo = 0.0, hi = 1.0e7;
    for (int it = 0; it < 200; ++it) {
        double a = 0.5 * (lo + hi);
        double p = 1.0;
        for (int i = 0; i < k_rank; ++i)
            p *= (double)(n_train - i) / ((double)(n_train - i) + a);
        if (p > RC_CFAR_PFA) lo = a; else hi = a;
    }
    return 0.5 * (lo + hi);
}

static int cmp_float(const void *a, const void *b) {
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

int cfar_detect(const float *map, cfar_kind kind,
                detection_t *out, int max_out) {
    const int N = cfar_num_training();
    const int k = (int)(RC_CFAR_OS_FRAC * N);
    const double alpha = (kind == CFAR_CA) ? cfar_alpha_ca(N)
                                           : cfar_alpha_os(N, k);
    float train[WMAX];
    int count = 0;

    /* range does not wrap -> skip edges; Doppler wraps -> test all, modular idx */
    for (int r = WIN; r < RC_RANGE_FFT - WIN; ++r) {
        for (int d = 0; d < RC_DOPPLER_FFT; ++d) {
            float cut = map[rd_map_idx(r, d)];

            /* gather training cells (exclude guard box + CUT) */
            int nt = 0;
            double sum = 0.0;
            for (int dr = -WIN; dr <= WIN; ++dr) {
                for (int dd = -WIN; dd <= WIN; ++dd) {
                    if (abs(dr) <= RC_CFAR_GUARD && abs(dd) <= RC_CFAR_GUARD)
                        continue;                       /* guard band + CUT      */
                    int rr = r + dr;                    /* range: bounded        */
                    int ddi = (d + dd + RC_DOPPLER_FFT) % RC_DOPPLER_FFT; /* wrap */
                    float v = map[rd_map_idx(rr, ddi)];
                    train[nt++] = v;
                    sum += v;
                }
            }

            double noise;
            if (kind == CFAR_CA) {
                noise = sum / nt;                       /* cell average          */
            } else {
                qsort(train, nt, sizeof(float), cmp_float);
                int kk = k < nt ? k : nt - 1;
                noise = train[kk];                      /* k-th ordered statistic*/
            }

            if (cut > alpha * noise) {
                if (count < max_out) {
                    out[count].range_bin = r;
                    out[count].doppler_bin = d;
                    out[count].power = cut;
                }
                count++;
            }
        }
    }
    return count;
}
