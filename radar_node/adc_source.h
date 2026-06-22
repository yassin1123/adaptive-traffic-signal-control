/*
 * adc_source.h — the ONE hardware abstraction seam (spec §2/§3).
 *
 * Everything above this interface (range/Doppler/CFAR/AoA/cluster/track/zones)
 * is identical on host and target. Only the source of the per-frame IQ buffer
 * differs:
 *    - HOST  : adc_source_synth  -> synthetic IQ from known targets
 *    - TARGET: adc_source_target -> the real ADC/DMA buffer on the IWR6843
 *
 * A frame buffer is always SYNTH_FRAME_LEN complex samples laid out
 * [virtual element][chirp][ADC sample] (see synth_target.h).
 */
#ifndef ADC_SOURCE_H
#define ADC_SOURCE_H

#include <stdint.h>
#include "cplx.h"
#include "synth_target.h"

typedef struct adc_source adc_source;

/* fill() writes one frame into `frame` (length SYNTH_FRAME_LEN); returns 0 on
 * success, non-zero on error/end-of-stream. */
struct adc_source {
    int (*fill)(adc_source *self, cplx *frame);
    void *ctx;
};

/* ---- HOST source: synthetic IQ -------------------------------------------- */
typedef struct {
    const target_t *targets;
    int ntargets;
    double noise_sigma;
    uint32_t seed;          /* base seed; combined with frame index per frame   */
    uint32_t frame_count;   /* advanced each fill so successive frames differ   */
} adc_synth_ctx;

/* Build a synthetic ADC source bound to `ctx` (caller owns ctx + its targets). */
adc_source adc_source_synth(adc_synth_ctx *ctx);

/* ---- TARGET source: real ADC buffer (documented stub) --------------------- */
/* On the IWR6843 this is where the EDMA-delivered ADC buffer is handed in; it is
 * compiled but unused on host. See adc_source.c for the hookup notes. */
adc_source adc_source_target(void);

#endif /* ADC_SOURCE_H */
