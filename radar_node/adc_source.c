/*
 * adc_source.c — host (synthetic) and target (real ADC) implementations of the
 * frame source. The chain above this seam never knows which one it is using.
 */
#include "adc_source.h"

/* ---- HOST: synthetic IQ --------------------------------------------------- */
static int synth_fill(adc_source *self, cplx *frame) {
    adc_synth_ctx *c = (adc_synth_ctx *)self->ctx;
    /* combine base seed with the frame index so each frame's noise differs but
     * the whole run is reproducible for a given base seed. */
    uint32_t seed = c->seed * 2654435761u + c->frame_count + 1u;
    synth_generate_frame(frame, c->targets, c->ntargets, c->noise_sigma, seed);
    c->frame_count++;
    return 0;
}

adc_source adc_source_synth(adc_synth_ctx *ctx) {
    adc_source s;
    s.fill = synth_fill;
    s.ctx = ctx;
    return s;
}

/* ---- TARGET: real ADC buffer (documented stub) ---------------------------- *
 * On the IWR6843 the data path is:
 *   1. mmWave front-end transmits the configured chirps; RX mixers produce IF.
 *   2. The ADC samples IF; the on-chip EDMA copies each chirp's samples into an
 *      L3/ADC-buffer ping-pong region as they arrive.
 *   3. A frame-done interrupt signals a complete frame of
 *      [TX-chirp][RX][ADC sample]; TDM-MIMO demux reorders TX into the virtual
 *      array, giving the [virtual element][chirp][ADC sample] layout this code
 *      consumes — IDENTICAL shape to the synthetic frame.
 * To flash on hardware, this fill() copies/points at that EDMA buffer instead of
 * synthesising. Nothing else in the chain changes. */
static int target_fill(adc_source *self, cplx *frame) {
    (void)self;
    (void)frame;
    /* No RF front-end on the host — this path is never taken in the host build.
     * On-target: memcpy from the ADC ping-pong buffer (post TDM-demux) here. */
    return -1;
}

adc_source adc_source_target(void) {
    adc_source s;
    s.fill = target_fill;
    s.ctx = 0;
    return s;
}
