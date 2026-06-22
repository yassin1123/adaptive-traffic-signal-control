/*
 * radar_config.h — THE single source of truth for every radar parameter.
 * (spec §5: no radar literal appears anywhere else in the code.)
 *
 * Target: TI IWR6843, 60 GHz FMCW automotive/industrial mmWave SoC, configured
 * for a roadside intersection-sensing node. Values are chosen so the derived
 * quantities (range/velocity resolution, max range/velocity) are sane for a
 * road intersection — see the asserts in radar_config_validate().
 *
 * Also defines the bin<->physical-unit conversions used by EVERY stage and test.
 * Keeping them here (built from the same constants) is what lets the gates assert
 * recovered PHYSICAL values (metres, m/s, degrees) against the injected truth,
 * rather than bin indices verifying against themselves.
 */
#ifndef RADAR_CONFIG_H
#define RADAR_CONFIG_H

#include <assert.h>
#include <stdio.h>
#include "cplx.h"

/* ===================== physical constants ===================== */
#define RC_C            299792458.0          /* speed of light (m/s)            */

/* ===================== RF / chirp ============================= */
#define RC_F0           60.0e9               /* chirp start frequency (Hz)      */
#define RC_LAMBDA       (RC_C / RC_F0)        /* wavelength (m) ~5 mm at 60 GHz  */
#define RC_SLOPE        11.0e12              /* chirp slope (Hz/s) = 11 MHz/us  */
#define RC_FS           8.0e6                /* ADC sample rate (Hz)            */
#define RC_N_SAMPLES    256                  /* ADC samples per chirp           */
#define RC_TR           ((double)RC_N_SAMPLES / RC_FS)  /* active sampling time (s) */
#define RC_BANDWIDTH    (RC_SLOPE * RC_TR)    /* swept bandwidth used (Hz)       */
#define RC_CHIRP_PERIOD 50.0e-6              /* chirp repetition period Tc (s)  */
#define RC_IDLE_TIME    (RC_CHIRP_PERIOD - RC_TR)        /* idle/reset per chirp */
#define RC_ADC_START    0.0                  /* ADC start delay (s) — modelled 0 */

/* ===================== frame ================================== */
#define RC_N_CHIRPS     128                  /* chirps per frame (Doppler dim)  */
#define RC_FRAME_PERIOD (RC_N_CHIRPS * RC_CHIRP_PERIOD)  /* active capture time (s) */
#define RC_FRAME_INTERVAL 0.05               /* frame repetition period (s), 20 Hz;
                                              * the dt between frames the tracker
                                              * integrates over (capture + idle).  */

/* ===================== antenna / MIMO ========================= */
/* IWR6843 has 3 TX, 4 RX. We use 2 TX for the azimuth virtual array (the 3rd is
 * elevation, out of scope) giving a NUM_TX*NUM_RX = 8-element half-wavelength
 * azimuth array via TDM-MIMO. */
#define RC_NUM_TX       2
#define RC_NUM_RX       4
#define RC_NUM_VIRT     (RC_NUM_TX * RC_NUM_RX)   /* virtual array elements (8)  */
#define RC_ELEMENT_SPACING 0.5               /* element spacing in wavelengths  */

/* ===================== processing sizes ====================== */
#define RC_RANGE_FFT    256                  /* range-FFT length (>= N_SAMPLES) */
#define RC_DOPPLER_FFT  128                  /* Doppler-FFT length (>= N_CHIRPS)*/
#define RC_ANGLE_FFT    64                   /* angle-FFT length (zero-padded)  */

/* ===================== CFAR ================================== */
#define RC_CFAR_GUARD   2                    /* guard cells each side (2-D)      */
#define RC_CFAR_TRAIN   8                    /* training cells each side (2-D)   */
#define RC_CFAR_PFA     1.0e-3               /* target probability of false alarm */
#define RC_CFAR_OS_FRAC 0.75                 /* OS-CFAR rank as fraction of N_train
                                              * (k = 0.75*N: robust to a few
                                              *  interfering targets in the window) */

/* ===================== clustering / tracking ================= */
/* DBSCAN works in a scaled (range[m], velocity[m/s], angle[deg]) space; epsilon
 * is a unitless distance after each axis is divided by its scale below. */
#define RC_CLUSTER_EPS      1.5
#define RC_CLUSTER_MIN_PTS  2
#define RC_CLUSTER_SCALE_RANGE  2.0          /* m per unit                       */
#define RC_CLUSTER_SCALE_VEL    2.0          /* m/s per unit                     */
#define RC_CLUSTER_SCALE_ANGLE 10.0          /* deg per unit                     */
#define RC_TRACK_GATE_M     6.0              /* association gate (m)             */
#define RC_TRACK_ALPHA      0.6              /* position/velocity smoothing      */

/* ===================== zones / classification =============== */
/* A roadside node faces the intersection; approaches are azimuth sectors across
 * the field of view. Edges are NUM_ZONES+1 azimuths (deg); names label each
 * approach. (In a real deployment the sector geometry is surveyed per site; here
 * equal azimuth sectors are a representative mapping.) */
#define RC_NUM_ZONES        4
#define RC_ZONE_AZ_EDGES    { -90.0, -45.0, 0.0, 45.0, 90.0 }
#define RC_ZONE_NAMES       { "approach_W", "approach_S", "approach_E", "approach_N" }
/* Vehicle vs pedestrian: a pedestrian is BOTH slow AND a weak (small-RCS)
 * reflector. Speed alone is ambiguous (a queued/stopped vehicle is also slow),
 * so the reflected POWER (RCS proxy, ~ amplitude^2) disambiguates: a stopped
 * vehicle is slow but strong, a pedestrian is slow and weak. (Detection COUNT is
 * a poor proxy here — the FFT main-lobe width sets it, not target strength.)
 * RC_VEHICLE_RCS_MIN is a summed-cluster-power threshold calibrated to the
 * current processing gain; on hardware it is set against a known reference
 * target, exactly as real radar RCS calibration is done. */
#define RC_PED_SPEED_MAX    2.5              /* m/s (~9 km/h) ped speed ceiling   */
#define RC_VEHICLE_RCS_MIN  5.0e8            /* cluster power below this => weak   */

/* ===================== derived (computed) ==================== */
/* range resolution  = c / (2 * B)                                            */
#define RC_RANGE_RES    (RC_C / (2.0 * RC_BANDWIDTH))
/* max range          = c * Fs / (2 * slope)   (complex sampling, N_SAMPLES bins)*/
#define RC_MAX_RANGE    (RC_C * RC_FS / (2.0 * RC_SLOPE))
/* velocity resolution = lambda / (2 * N_chirps * Tc)                          */
#define RC_VEL_RES      (RC_LAMBDA / (2.0 * RC_N_CHIRPS * RC_CHIRP_PERIOD))
/* max unambiguous velocity = lambda / (4 * Tc)                                */
#define RC_MAX_VEL      (RC_LAMBDA / (4.0 * RC_CHIRP_PERIOD))

/* ===================== compile-time invariants =============== */
_Static_assert(RC_RANGE_FFT >= RC_N_SAMPLES, "range FFT must cover all ADC samples");
_Static_assert(RC_DOPPLER_FFT >= RC_N_CHIRPS, "Doppler FFT must cover all chirps");
_Static_assert((RC_RANGE_FFT & (RC_RANGE_FFT - 1)) == 0, "range FFT must be power of two");
_Static_assert((RC_DOPPLER_FFT & (RC_DOPPLER_FFT - 1)) == 0, "Doppler FFT must be power of two");
_Static_assert((RC_ANGLE_FFT & (RC_ANGLE_FFT - 1)) == 0, "angle FFT must be power of two");
_Static_assert(RC_NUM_VIRT == RC_NUM_TX * RC_NUM_RX, "virtual array size mismatch");

/* ============================================================= */
/* bin <-> physical conversions — the SINGLE mapping every stage/test uses.    */
/* ============================================================= */

/* range (m) of range-FFT bin k. Derivation: bin k -> beat freq k*Fs/RANGE_FFT;
 * range = f_beat * c / (2*slope). Reduces to k*RANGE_RES when RANGE_FFT==N_SAMPLES. */
static inline double rc_range_of_bin(int k) {
    return (double)k * RC_C * RC_FS / (2.0 * RC_SLOPE * RC_RANGE_FFT);
}

/* velocity (m/s) of Doppler-FFT bin d, with fftshift origin: bins in the upper
 * half are negative (closing/opening sign convention matches the synth model:
 * +v -> +Doppler -> low positive bin). */
static inline double rc_velocity_of_bin(int d) {
    int idx = (d < RC_DOPPLER_FFT / 2) ? d : d - RC_DOPPLER_FFT;
    return (double)idx * RC_VEL_RES;
}

/* azimuth (deg) of angle-FFT bin a. The angle-FFT over the virtual array maps a
 * bin to spatial frequency u = (a or a-N)/N in cycles/element; sin(theta) =
 * u / element_spacing(lambda). Returns NaN-safe clamp outside the unambiguous FoV. */
static inline double rc_azimuth_of_bin(int a) {
    int idx = (a < RC_ANGLE_FFT / 2) ? a : a - RC_ANGLE_FFT;
    double u = (double)idx / (double)RC_ANGLE_FFT;        /* cycles per element  */
    double sin_theta = u / RC_ELEMENT_SPACING;
    if (sin_theta > 1.0) sin_theta = 1.0;
    if (sin_theta < -1.0) sin_theta = -1.0;
    return asin(sin_theta) * 180.0 / RC_PI;
}

/* ===================== validation + report ================== */
static inline void radar_config_validate(void) {
    /* derived quantities must be sane for a road intersection */
    assert(RC_RANGE_RES > 0.05 && RC_RANGE_RES < 1.0);      /* sub-metre range bins */
    assert(RC_MAX_RANGE > 50.0 && RC_MAX_RANGE < 300.0);    /* covers an approach   */
    assert(RC_VEL_RES > 0.05 && RC_VEL_RES < 1.0);          /* fine velocity bins   */
    assert(RC_MAX_VEL > 15.0 && RC_MAX_VEL < 60.0);         /* ~55..210 km/h span   */
    assert(RC_BANDWIDTH > 0.0 && RC_BANDWIDTH < 5.0e9);     /* within 60GHz band    */
    assert(RC_IDLE_TIME > 0.0);                             /* chirp period > active */
}

static inline void radar_config_print(void) {
    printf("  RF:    f0=%.1f GHz  lambda=%.2f mm  slope=%.1f MHz/us  B=%.0f MHz\n",
           RC_F0 / 1e9, RC_LAMBDA * 1e3, RC_SLOPE / 1e12, RC_BANDWIDTH / 1e6);
    printf("  frame: %d samples @ %.1f MHz x %d chirps  (Tc=%.1f us, frame=%.2f ms)\n",
           RC_N_SAMPLES, RC_FS / 1e6, RC_N_CHIRPS, RC_CHIRP_PERIOD * 1e6,
           RC_FRAME_PERIOD * 1e3);
    printf("  array: %d TX x %d RX -> %d virtual elements @ %.1f lambda spacing\n",
           RC_NUM_TX, RC_NUM_RX, RC_NUM_VIRT, RC_ELEMENT_SPACING);
    printf("  DERIVED:\n");
    printf("    range resolution      = %.3f m\n", RC_RANGE_RES);
    printf("    max range             = %.1f m\n", RC_MAX_RANGE);
    printf("    velocity resolution   = %.3f m/s\n", RC_VEL_RES);
    printf("    max unambiguous vel   = %.2f m/s (%.0f km/h)\n",
           RC_MAX_VEL, RC_MAX_VEL * 3.6);
}

#endif /* RADAR_CONFIG_H */
