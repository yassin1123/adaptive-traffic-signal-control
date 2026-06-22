/*
 * fft.c — iterative in-place radix-2 Cooley-Tukey FFT.
 *
 * Decimation-in-time: first bit-reverse the input order, then combine in
 * log2(n) butterfly stages. This is the textbook O(n log n) transform; on the
 * IWR6843 the equivalent is offloaded to the HWA, but the math is the same.
 */
#include "fft.h"

void fft(cplx *x, int n, int dir) {
    /* --- bit-reversal permutation --- */
    int j = 0;
    for (int i = 1; i < n; ++i) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j) {
            cplx t = x[i];
            x[i] = x[j];
            x[j] = t;
        }
    }

    /* --- butterfly stages --- */
    for (int len = 2; len <= n; len <<= 1) {
        /* principal twiddle: forward (dir=-1) uses exp(-j*2*pi/len), the standard
         * DFT sign so a tone exp(+j*2*pi*k0*n/N) lands in bin k0; inverse (dir=+1)
         * uses exp(+j*2*pi/len) and is scaled by 1/n below. */
        double ang = (double)dir * 2.0 * RC_PI / (double)len;
        cplx wlen = cplx_make((float)cos(ang), (float)sin(ang));
        for (int i = 0; i < n; i += len) {
            cplx w = cplx_make(1.0f, 0.0f);
            for (int k = 0; k < len / 2; ++k) {
                cplx u = x[i + k];
                cplx v = cplx_mul(x[i + k + len / 2], w);
                x[i + k] = cplx_add(u, v);
                x[i + k + len / 2] = cplx_sub(u, v);
                w = cplx_mul(w, wlen);
            }
        }
    }

    /* inverse transform: scale by 1/n */
    if (dir > 0) {
        float inv = 1.0f / (float)n;
        for (int i = 0; i < n; ++i)
            x[i] = cplx_scale(x[i], inv);
    }
}

void fft_forward(cplx *x, int n) { fft(x, n, -1); }
void fft_inverse(cplx *x, int n) { fft(x, n, +1); }
