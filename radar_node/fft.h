/*
 * fft.h — portable complex FFT (the host backend / HWA stand-in).
 *
 * On the IWR6843 the range- and Doppler-FFTs run on the hardware accelerator
 * (HWA) / C674x DSP. On the host build this self-contained radix-2 FFT stands in
 * behind the same call sites, so the *chain* is identical and only the FFT
 * backend differs (spec §2/§3). Dependency-light: standard library only.
 */
#ifndef FFT_H
#define FFT_H

#include "cplx.h"

/* In-place radix-2 Cooley-Tukey FFT. n MUST be a power of two.
 * dir = -1 forward (exp(-j..)), dir = +1 inverse (exp(+j..), scaled by 1/n). */
void fft(cplx *x, int n, int dir);

/* Convenience wrappers. */
void fft_forward(cplx *x, int n);
void fft_inverse(cplx *x, int n);

#endif /* FFT_H */
