#pragma once

#include <stddef.h>

#include "estimator.h"
#include "filter_params.h"
#include "record_format.h"
#include "vibration.h"

// Encodes one estimator snapshot as the JSON frame the dashboard consumes.
//
// The frame is hand-built with snprintf rather than a JSON library: the schema
// is fixed and small, it is emitted 20 times a second, and this keeps both the
// flash cost and the per-frame allocation at zero.

// Bytes to reserve for a frame. The largest realistic frame is around 1100
// bytes; this leaves comfortable headroom without being worth heap-allocating.
constexpr size_t kTelemetryBufferSize = 1800;

// Bytes to reserve for a spectrum frame, which is far larger and far rarer:
// three axes of kBins amplitudes at four significant figures. 129 bins at a
// worst case of 11 characters each ("1.234e-05" plus a comma) is 4257 bytes
// across three axes, so this is the same kind of headroom as above.
constexpr size_t kSpectrumBufferSize = 5000;

// Writes a complete frame into `buffer`. Returns the number of bytes written,
// or 0 if the buffer was too small (in which case nothing is sent).
size_t BuildTelemetryFrame(char* buffer, size_t buffer_size, const EstimatorSnapshot& snapshot,
                           const FilterParams& params, const char* status_name,
                           const SpectrumConfig& spectrum,
                           const RecorderStatus& recorder = RecorderStatus());

// Writes one computed spectrum as its own frame, sent only while the panel is
// on and only when a new transform has been produced.
//
// It is a separate message rather than a field in the telemetry frame for two
// reasons: it is four times the size of one, and it is produced at a quarter
// of the rate. The dashboard tells them apart by the "type" field, which only
// this one carries.
size_t BuildSpectrumFrame(char* buffer, size_t buffer_size, const SpectrumSnapshot& spectrum);
