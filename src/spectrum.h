#pragma once

// Amplitude spectrum of one window of one sensor axis.
//
// What this is for: after an excitation -- a tap, a gust, a step on the
// throttle -- whatever the board is bolted to goes on ringing at its own
// resonances. Those modes are invisible in the dashboard's time-series plots,
// which are fed at 20 Hz and alias everything above 10 Hz, and invisible to
// the filter, which only ever sees their sum. A transform of the raw 200 Hz
// tick data puts them on a frequency axis, where a mode is one peak and its
// decay is a peak that shrinks.
//
// Pure math with no Arduino dependency, so the transform is tested on the host
// against synthetic tones -- see test/test_spectrum/. Same reason nav_filter.h
// and geo.h are headers. The ring buffer that feeds it, and the decision about
// when to run it, live in vibration.h.

#include <math.h>

namespace spectrum {

// Window length, in estimator ticks. A power of two, because the transform
// below is radix-2.
//
// 256 ticks is 1.28 s at the 200 Hz tick: 0.78 Hz bins across the 0-100 Hz
// band the sensor can actually deliver (IMU_ACCEL_GYRO_ODR_HZ band-limits to
// ~104 Hz). Both ends of that were chosen against the ringdown case rather
// than for resolution alone. A longer window separates two close modes better
// but dilutes a decay that is over in a few hundred milliseconds, because the
// transform averages the ring together with the silence after it. A shorter
// one follows the decay but smears the modes into each other.
constexpr int kSamples = 256;

// Bins 0..N/2 inclusive. A real signal's spectrum is conjugate-symmetric, so
// the upper half carries nothing the lower half does not.
constexpr int kBins = kSamples / 2 + 1;

// Scratch for one transform: 2 kB, which is a quarter of the Arduino loop
// task's stack, so the caller owns it as static storage rather than letting it
// land there.
struct Workspace {
    float re[kSamples];
    float im[kSamples];
};

namespace internal {

constexpr float kTwoPi = 6.283185307179586f;

// Periodic Hann window, w[n] = 0.5 * (1 - cos(2*pi*n/N)).
//
// Windowing is the difference between a usable spectrum and a useless one
// here. The window is an arbitrary 1.28 s slice of a signal that did not stop
// at its edges, and the transform treats that slice as one period of something
// periodic -- so the step between the last sample and the first is read as
// broadband content, and it buries every real peak under it. Hann tapers both
// ends to zero and costs only a widening of each peak to about two bins.
//
// Dividing by N rather than N-1 gives the periodic form, which is the one that
// is genuinely periodic in the transform's own sense. The symmetric form
// leaves a one-sample discontinuity -- exactly the thing being removed.
inline float HannWeight(int index) {
    return 0.5f * (1.0f - cosf(kTwoPi * static_cast<float>(index) / kSamples));
}

// Coherent gain of that window, sum(w)/N, which is exactly 0.5 for periodic
// Hann. A windowed tone comes out this much smaller than it went in, and the
// scaling in Compute() puts it back.
constexpr float kCoherentGain = 0.5f;

inline void Swap(float& a, float& b) {
    const float held = a;
    a = b;
    b = held;
}

// In-place iterative radix-2 Cooley-Tukey, decimation in time.
inline void Transform(Workspace& work) {
    // Decimation in time yields its outputs in natural order only if the
    // inputs start in bit-reversed order, so permute them first.
    for (int i = 1, j = 0; i < kSamples; i++) {
        int bit = kSamples >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            Swap(work.re[i], work.re[j]);
            Swap(work.im[i], work.im[j]);
        }
    }

    for (int len = 2; len <= kSamples; len <<= 1) {
        const float angle = -kTwoPi / static_cast<float>(len);
        const int half = len >> 1;
        for (int k = 0; k < half; k++) {
            // Trig called once per twiddle and reused across every butterfly
            // group at this stage: 255 sinf/cosf pairs per transform, three
            // transforms four times a second. That is a rounding error on
            // core 0, and it buys exactness over the usual angle-recurrence,
            // which drifts across eight stages for no gain anyone could see
            // on a plot.
            const float w_re = cosf(angle * static_cast<float>(k));
            const float w_im = sinf(angle * static_cast<float>(k));
            for (int start = 0; start < kSamples; start += len) {
                const int a = start + k;
                const int b = a + half;
                const float t_re = work.re[b] * w_re - work.im[b] * w_im;
                const float t_im = work.re[b] * w_im + work.im[b] * w_re;
                work.re[b] = work.re[a] - t_re;
                work.im[b] = work.im[a] - t_im;
                work.re[a] += t_re;
                work.im[a] += t_im;
            }
        }
    }
}

}  // namespace internal

// Single-sided amplitude spectrum of `samples`, in the samples' own units: a
// pure tone of amplitude A that lands on a bin reads as A in that bin.
//
// `samples` must be in chronological order -- see the de-interleave in
// vibration.cpp for why that is worth saying.
//
// The mean is removed first. A vibration signal rides on a DC offset (9.81 on
// whichever accelerometer axis is up) three or four orders of magnitude larger
// than the modes being looked for, and even through a Hann window its leakage
// swamps the bottom of the band. Removing it also makes bin 0 meaningless,
// which is why the dashboard never plots it.
inline void Compute(const float samples[kSamples], Workspace& work, float bins_out[kBins]) {
    float mean = 0.0f;
    for (int i = 0; i < kSamples; i++) {
        mean += samples[i];
    }
    mean /= static_cast<float>(kSamples);

    for (int i = 0; i < kSamples; i++) {
        work.re[i] = (samples[i] - mean) * internal::HannWeight(i);
        work.im[i] = 0.0f;
    }

    internal::Transform(work);

    // Two corrections: 1/N for the transform's own gain, and 1/coherent_gain
    // for the amplitude the window took out. Interior bins are doubled on top
    // of that because a real tone splits its energy between its bin and the
    // mirror bin in the half that is not reported; bin 0 and the Nyquist bin
    // have no mirror to fold back in.
    constexpr float kScale = 1.0f / (static_cast<float>(kSamples) * internal::kCoherentGain);
    for (int bin = 0; bin < kBins; bin++) {
        const float magnitude = sqrtf(work.re[bin] * work.re[bin] + work.im[bin] * work.im[bin]);
        const bool mirrored = bin > 0 && bin < kSamples / 2;
        bins_out[bin] = magnitude * kScale * (mirrored ? 2.0f : 1.0f);
    }
}

}  // namespace spectrum
