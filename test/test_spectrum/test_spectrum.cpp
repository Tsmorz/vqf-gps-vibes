// Host-side tests for the amplitude spectrum.
//
// The transform is checked against synthetic signals whose spectrum is known
// in closed form, because the failure mode being guarded against is not a
// crash: a scaling slip or an off-by-one in the bit-reversal produces a plot
// that looks entirely plausible and puts every resonance at the wrong
// frequency or the wrong size. Nothing on the dashboard could show that.
//
// The amplitude convention under test: a tone of amplitude A landing exactly
// on a bin reads A in that bin, in the sensor's own units. Its two neighbours
// read A/2, which is the Hann window's main lobe and not an error -- see
// test_a_tone_spreads_over_the_hann_main_lobe.

#include <math.h>
#include <unity.h>

#include "spectrum.h"

namespace {

constexpr float kTwoPi = 6.283185307179586f;

spectrum::Workspace work;
float samples[spectrum::kSamples];
float bins[spectrum::kBins];

// Fills `samples` with a cosine of `amplitude` at exactly `bin` cycles per
// window, so it lands on a bin centre and leaks only into the window's own
// main lobe.
void FillTone(int bin, float amplitude, float offset = 0.0f) {
    for (int i = 0; i < spectrum::kSamples; i++) {
        const float phase = kTwoPi * bin * i / spectrum::kSamples;
        samples[i] = offset + amplitude * cosf(phase);
    }
}

// Largest bin at or above `first`, which skips bin 0 where the mean removal
// leaves nothing meaningful.
int PeakBin(int first = 1) {
    int peak = first;
    for (int bin = first; bin < spectrum::kBins; bin++) {
        if (bins[bin] > bins[peak]) {
            peak = bin;
        }
    }
    return peak;
}

}  // namespace

void setUp() {
}
void tearDown() {
}

// The window length has to stay a power of two or the radix-2 transform walks
// off the end of its arrays, and kBins has to match it.
void test_window_length_is_a_power_of_two() {
    TEST_ASSERT_EQUAL_INT(0, spectrum::kSamples & (spectrum::kSamples - 1));
    TEST_ASSERT_EQUAL_INT(spectrum::kSamples / 2 + 1, spectrum::kBins);
}

void test_a_tone_lands_in_its_own_bin_at_its_own_amplitude() {
    FillTone(20, 2.5f);
    spectrum::Compute(samples, work, bins);

    TEST_ASSERT_EQUAL_INT(20, PeakBin());
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 2.5f, bins[20]);
}

// The scaling must not depend on which bin the tone is in: a mistake in the
// interior-bin doubling shows up as a frequency-dependent amplitude, which
// would read as a sensor with a rolloff it does not have.
void test_amplitude_is_reported_the_same_across_the_band() {
    const int probes[] = {3, 17, 64, 100, 127};
    for (const int bin : probes) {
        FillTone(bin, 1.0f);
        spectrum::Compute(samples, work, bins);
        TEST_ASSERT_EQUAL_INT(bin, PeakBin());
        TEST_ASSERT_FLOAT_WITHIN(0.002f, 1.0f, bins[bin]);
    }
}

// The Nyquist bin has no mirror in the half that is not reported, so it must
// not be doubled the way the interior bins are. Getting this wrong overstates
// the top of the band by exactly 2x.
//
// Its lower neighbour reads the same 1.5, which is not a bug and is why this
// does not assert a unique peak: bin 127 is an ordinary interior bin, so it is
// doubled, and doubling the window's half-amplitude skirt lands back on the
// full amplitude. Only a tone sitting exactly on Nyquist does this, and the
// IMU band-limits to ~104 Hz against a 100 Hz Nyquist, so nothing real gets
// close.
void test_the_nyquist_bin_is_not_doubled() {
    // cos(pi*n) -- the fastest thing the sample rate can represent.
    for (int i = 0; i < spectrum::kSamples; i++) {
        samples[i] = (i % 2 == 0) ? 1.5f : -1.5f;
    }
    spectrum::Compute(samples, work, bins);

    TEST_ASSERT_FLOAT_WITHIN(0.002f, 1.5f, bins[spectrum::kBins - 1]);
    // Nothing below the band edge may exceed it, and nothing away from the
    // edge may carry energy at all.
    for (int bin = 1; bin < spectrum::kBins - 2; bin++) {
        TEST_ASSERT_TRUE(bins[bin] < 0.005f);
    }
}

// A Hann-windowed tone occupies three bins with a fixed 1 : 0.5 shape. This
// pins the window down: a rectangular window (or a symmetric Hann, or one
// applied to a rotated buffer) gives a visibly different skirt.
void test_a_tone_spreads_over_the_hann_main_lobe() {
    FillTone(40, 1.0f);
    spectrum::Compute(samples, work, bins);

    TEST_ASSERT_FLOAT_WITHIN(0.002f, 1.0f, bins[40]);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.5f, bins[39]);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.5f, bins[41]);

    // And nothing worth looking at outside it.
    TEST_ASSERT_TRUE(bins[37] < 0.005f);
    TEST_ASSERT_TRUE(bins[43] < 0.005f);
}

// The whole reason the mean is removed: an accelerometer axis sits at 9.81 and
// the modes being looked for are three or four orders of magnitude smaller.
void test_gravity_does_not_drown_the_signal() {
    FillTone(30, 0.01f, /*offset=*/9.81f);
    spectrum::Compute(samples, work, bins);

    TEST_ASSERT_EQUAL_INT(30, PeakBin());
    TEST_ASSERT_FLOAT_WITHIN(0.0005f, 0.01f, bins[30]);
    // Bin 0 is where an unremoved offset would pile up.
    TEST_ASSERT_TRUE(bins[0] < 0.0005f);
}

// Two modes ringing together have to come back as two peaks at their own
// amplitudes, not as one peak or as a sum -- that is the case the panel exists
// for.
void test_two_modes_are_resolved_independently() {
    for (int i = 0; i < spectrum::kSamples; i++) {
        const float slow = kTwoPi * 12 * i / spectrum::kSamples;
        const float fast = kTwoPi * 53 * i / spectrum::kSamples;
        samples[i] = 1.0f * cosf(slow) + 0.25f * cosf(fast);
    }
    spectrum::Compute(samples, work, bins);

    TEST_ASSERT_FLOAT_WITHIN(0.002f, 1.0f, bins[12]);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.25f, bins[53]);
}

// A real resonance almost never sits on a bin centre. It must still be found
// within one bin, and the amplitude must not collapse -- worst case for Hann
// is a tone exactly between two bins, which reads about 15% low.
void test_a_tone_between_bins_is_still_found() {
    for (int i = 0; i < spectrum::kSamples; i++) {
        samples[i] = cosf(kTwoPi * 25.5f * i / spectrum::kSamples);
    }
    spectrum::Compute(samples, work, bins);

    const int peak = PeakBin();
    TEST_ASSERT_TRUE(peak == 25 || peak == 26);
    TEST_ASSERT_TRUE(bins[peak] > 0.8f);
    TEST_ASSERT_TRUE(bins[peak] < 1.0f);
}

// A resting sensor must read as a flat floor, not as a peak somewhere. This
// also covers the constant-input path, where mean removal leaves exact zeros.
void test_a_constant_signal_has_no_spectrum() {
    for (int i = 0; i < spectrum::kSamples; i++) {
        samples[i] = -9.81f;
    }
    spectrum::Compute(samples, work, bins);

    for (int bin = 0; bin < spectrum::kBins; bin++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, bins[bin]);
    }
}

// The workspace is reused for every axis of every window, so a transform must
// not be able to see what the last one left behind.
void test_the_workspace_carries_nothing_between_transforms() {
    FillTone(70, 3.0f);
    spectrum::Compute(samples, work, bins);
    TEST_ASSERT_EQUAL_INT(70, PeakBin());

    FillTone(11, 0.5f);
    spectrum::Compute(samples, work, bins);

    TEST_ASSERT_EQUAL_INT(11, PeakBin());
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.5f, bins[11]);
    // Nothing left of the previous tone.
    TEST_ASSERT_TRUE(bins[70] < 0.002f);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_window_length_is_a_power_of_two);
    RUN_TEST(test_a_tone_lands_in_its_own_bin_at_its_own_amplitude);
    RUN_TEST(test_amplitude_is_reported_the_same_across_the_band);
    RUN_TEST(test_the_nyquist_bin_is_not_doubled);
    RUN_TEST(test_a_tone_spreads_over_the_hann_main_lobe);
    RUN_TEST(test_gravity_does_not_drown_the_signal);
    RUN_TEST(test_two_modes_are_resolved_independently);
    RUN_TEST(test_a_tone_between_bins_is_still_found);
    RUN_TEST(test_a_constant_signal_has_no_spectrum);
    RUN_TEST(test_the_workspace_carries_nothing_between_transforms);
    return UNITY_END();
}
