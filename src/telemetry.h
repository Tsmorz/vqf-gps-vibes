#pragma once

#include <stddef.h>

#include "estimator.h"
#include "filter_params.h"

// Encodes one estimator snapshot as the JSON frame the dashboard consumes.
//
// The frame is hand-built with snprintf rather than a JSON library: the schema
// is fixed and small, it is emitted 20 times a second, and this keeps both the
// flash cost and the per-frame allocation at zero.

// Bytes to reserve for a frame. The largest realistic frame is around 1100
// bytes; this leaves comfortable headroom without being worth heap-allocating.
constexpr size_t kTelemetryBufferSize = 1800;

// Writes a complete frame into `buffer`. Returns the number of bytes written,
// or 0 if the buffer was too small (in which case nothing is sent).
size_t BuildTelemetryFrame(char* buffer, size_t buffer_size, const EstimatorSnapshot& snapshot,
                           const FilterParams& params, const char* status_name);
