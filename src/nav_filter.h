#pragma once

// Loosely-coupled inertial/GNSS navigation filter.
//
// VQF solves orientation; this solves where the board is. It is an extended
// Kalman filter over nine states, propagated by the accelerometer and
// corrected by GPS with an ordinary linear measurement update:
//
//     x = [ pE pN pU | vE vN vU | bx by bz ]
//
// Position and velocity live in the local ENU tangent plane anchored at the
// first GPS fix (see geo.h). The bias states are accelerometer bias in the
// *body* frame, which is where the error physically originates.
//
// Every measurement this filter takes -- GPS position, GPS velocity, the
// zero-velocity pseudo-measurement -- observes exactly one state directly, so
// updates are applied one scalar at a time. That keeps the Kalman gain a
// single column and needs no matrix inversion; it is also the numerically
// better-behaved way to apply a diagonal measurement covariance.
//
// Pure math, no Arduino dependency -- unit tested on the host in test/test_nav/.

#include <math.h>
#include <string.h>

class NavFilter {
   public:
    static constexpr int kNumStates = 9;
    static constexpr float kGravityMps2 = 9.80665f;

    // Bounds beyond which the state is not drifting, it is broken. A filter
    // fed garbage can run away to values that are still finite -- an
    // accelerometer that reads (0, 0, 0) because its cable came out looks
    // exactly like free fall, and reaches 100 km in under two minutes. These
    // are generous enough that no plausible use of this board reaches them.
    static constexpr float kMaxPlausiblePositionM = 1.0e6f;
    static constexpr float kMaxPlausibleSpeedMps = 1.0e3f;

    // Process-noise multiplier applied while coasting without an IMU. The
    // constant-velocity assumption is much weaker than a measured
    // acceleration, and the envelope should say so.
    static constexpr float kCoastingNoiseFactor = 10.0f;

    // Indices into the state vector, also used to address a scalar update.
    enum StateIndex {
        kPosEast = 0,
        kPosNorth = 1,
        kPosUp = 2,
        kVelEast = 3,
        kVelNorth = 4,
        kVelUp = 5,
        kBiasX = 6,
        kBiasY = 7,
        kBiasZ = 8,
    };

    // The runtime-tunable process noise terms, exposed as dashboard knobs.
    struct Params {
        float sigma_accel = 0.35f;        // m/s^2/sqrt(Hz), accel white noise
        float sigma_accel_bias = 0.008f;  // m/s^3/sqrt(Hz), bias random walk
    };

    NavFilter() {
        Reset();
    }

    // Returns the filter to its boot state: at the origin, stationary, with
    // large position/velocity uncertainty and a modest prior on accel bias.
    void Reset() {
        memset(x_, 0, sizeof(x_));
        memset(p_, 0, sizeof(p_));
        for (int i = kPosEast; i <= kPosUp; i++) {
            p_[i][i] = 100.0f;  // (10 m)^2 -- position essentially unknown
        }
        for (int i = kVelEast; i <= kVelUp; i++) {
            p_[i][i] = 1.0f;  // (1 m/s)^2
        }
        for (int i = kBiasX; i <= kBiasZ; i++) {
            p_[i][i] = 0.04f;  // (0.2 m/s^2)^2 -- typical LSM6DSOX offset
        }
    }

    // Propagates the state forward by `dt` seconds.
    //
    // `accel_body` is the raw specific force from the accelerometer (m/s^2)
    // and `r_body_to_enu` the row-major 3x3 rotation built from VQF's
    // quaternion. Subtracting the estimated bias, rotating into ENU and adding
    // gravity leaves the true kinematic acceleration.
    void Predict(const float accel_body[3], const float r_body_to_enu[9], float dt,
                 const Params& params) {
        if (dt <= 0.0f || dt > 0.5f) {
            return;  // a stalled or absurd tick would corrupt the covariance
        }
        float accel_enu[3];
        RotateBiasCorrectedAccel(accel_body, r_body_to_enu, accel_enu);

        // Constant-acceleration kinematics over the tick.
        for (int axis = 0; axis < 3; axis++) {
            x_[kPosEast + axis] += x_[kVelEast + axis] * dt + 0.5f * accel_enu[axis] * dt * dt;
            x_[kVelEast + axis] += accel_enu[axis] * dt;
        }
        PropagateCovariance(r_body_to_enu, dt, params);
    }

    // Propagates with no accelerometer at all: constant velocity, with the
    // process noise inflated to reflect that this is a guess rather than a
    // measurement. Used while the IMU is disconnected -- far better than
    // integrating whatever the driver hands back, and better than freezing,
    // which would claim the position is still known to its old accuracy.
    void PredictCoasting(float dt, const Params& params) {
        if (dt <= 0.0f || dt > 0.5f) {
            return;
        }
        for (int axis = 0; axis < 3; axis++) {
            x_[kPosEast + axis] += x_[kVelEast + axis] * dt;
        }

        Params coasting = params;
        coasting.sigma_accel *= kCoastingNoiseFactor;
        // With no attitude to resolve the bias states through, the identity
        // stands in for the rotation. The bias block is unobservable while
        // coasting either way, so the choice does not matter.
        static const float kIdentity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        PropagateCovariance(kIdentity, dt, coasting);
    }

    // Applies one scalar measurement `z` of the state at `index`, with
    // measurement variance `r`. This is the standard Kalman update specialised
    // to H = e_index^T, which reduces the gain to a single scaled column of P.
    void UpdateScalar(int index, float z, float r) {
        const float innovation = z - x_[index];
        const float innovation_cov = p_[index][index] + r;
        if (!(innovation_cov > 0.0f)) {
            return;  // guards against a NaN or non-positive covariance
        }

        // K = P * H^T / S, which for H = e_index^T is column `index` of P.
        float gain[kNumStates];
        for (int i = 0; i < kNumStates; i++) {
            gain[i] = p_[i][index] / innovation_cov;
            x_[i] += gain[i] * innovation;
        }

        // P = (I - K H) P, i.e. subtract the outer product of the gain with
        // row `index` of P. Row `index` must be copied first because the loop
        // below overwrites it while it is still being read.
        float row[kNumStates];
        memcpy(row, p_[index], sizeof(row));
        for (int i = 0; i < kNumStates; i++) {
            for (int j = 0; j < kNumStates; j++) {
                p_[i][j] -= gain[i] * row[j];
            }
        }
        Symmetrize();
    }

    // GPS position fix in local ENU metres. Horizontal and vertical accuracy
    // differ by roughly 2x on consumer GNSS, hence the separate sigmas.
    void UpdateGpsPosition(const float pos_enu[3], float sigma_h, float sigma_v) {
        UpdateScalar(kPosEast, pos_enu[0], sigma_h * sigma_h);
        UpdateScalar(kPosNorth, pos_enu[1], sigma_h * sigma_h);
        UpdateScalar(kPosUp, pos_enu[2], sigma_v * sigma_v);
    }

    // GPS ground-speed/course converted to horizontal velocity. The PA1010D
    // reports no vertical rate, so the up component is left to the IMU.
    void UpdateGpsVelocity(float east_mps, float north_mps, float sigma) {
        UpdateScalar(kVelEast, east_mps, sigma * sigma);
        UpdateScalar(kVelNorth, north_mps, sigma * sigma);
    }

    // Zero-velocity update: while VQF's rest detector reports the board is
    // still, all three velocity components are known to be zero. This is what
    // keeps the filter bounded indoors where there is no GPS fix -- without
    // it, accelerometer bias double-integrates into runaway position error.
    void UpdateZeroVelocity(float sigma) {
        const float measurement_variance = sigma * sigma;
        UpdateScalar(kVelEast, 0.0f, measurement_variance);
        UpdateScalar(kVelNorth, 0.0f, measurement_variance);
        UpdateScalar(kVelUp, 0.0f, measurement_variance);
    }

    // Snaps position to a fix and shrinks its covariance accordingly. Used for
    // the very first fix, where the ENU origin is defined to be that point and
    // an incremental update would leave a large spurious innovation behind.
    void SetPosition(const float pos_enu[3], float sigma) {
        for (int axis = 0; axis < 3; axis++) {
            x_[kPosEast + axis] = pos_enu[axis];
            p_[kPosEast + axis][kPosEast + axis] = sigma * sigma;
        }
    }

    float state(int index) const {
        return x_[index];
    }

    float variance(int index) const {
        return p_[index][index];
    }

    // The 3-sigma envelope the dashboard draws around each estimate.
    float ThreeSigma(int index) const {
        const float diagonal = p_[index][index];
        return diagonal > 0.0f ? 3.0f * sqrtf(diagonal) : 0.0f;
    }

    // True if the filter has stopped producing anything meaningful: a
    // non-finite value, a negative variance, or a state so far outside
    // physical plausibility that it can only be the result of bad input.
    // The caller resets rather than publishing it to the dashboard.
    bool IsDiverged() const {
        for (int i = 0; i < kNumStates; i++) {
            if (!isfinite(x_[i]) || !isfinite(p_[i][i]) || p_[i][i] < 0.0f) {
                return true;
            }
        }
        for (int axis = 0; axis < 3; axis++) {
            if (fabsf(x_[kPosEast + axis]) > kMaxPlausiblePositionM) {
                return true;
            }
            if (fabsf(x_[kVelEast + axis]) > kMaxPlausibleSpeedMps) {
                return true;
            }
        }
        return false;
    }

   private:
    // accel_enu = R * (accel_body - bias) + g, with g pointing down. At rest
    // the accelerometer reads +1 g along up, so the two cancel to zero.
    void RotateBiasCorrectedAccel(const float accel_body[3], const float r[9],
                                  float out_enu[3]) const {
        const float corrected[3] = {
            accel_body[0] - x_[kBiasX],
            accel_body[1] - x_[kBiasY],
            accel_body[2] - x_[kBiasZ],
        };
        for (int row = 0; row < 3; row++) {
            out_enu[row] = r[row * 3 + 0] * corrected[0] + r[row * 3 + 1] * corrected[1] +
                           r[row * 3 + 2] * corrected[2];
        }
        out_enu[2] -= kGravityMps2;
    }

    // P = F P F^T + Q.
    //
    // F is identity except for two off-diagonal blocks: velocity feeding
    // position (I*dt), and bias feeding both through the rotation. Q is the
    // discrete white-noise-acceleration model. Because the accelerometer noise
    // is isotropic, rotating it into ENU leaves it unchanged
    // (R sigma^2 I R^T = sigma^2 I), so Q needs no rotation.
    void PropagateCovariance(const float r[9], float dt, const Params& params) {
        float f[kNumStates][kNumStates] = {};
        for (int i = 0; i < kNumStates; i++) {
            f[i][i] = 1.0f;
        }
        for (int axis = 0; axis < 3; axis++) {
            f[kPosEast + axis][kVelEast + axis] = dt;
        }
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                const float rotation = r[row * 3 + col];
                f[kVelEast + row][kBiasX + col] = -rotation * dt;
                f[kPosEast + row][kBiasX + col] = -0.5f * rotation * dt * dt;
            }
        }

        float temp[kNumStates][kNumStates];
        MultiplyMatrices(f, p_, temp);     // temp = F P
        MultiplyByTranspose(temp, f, p_);  // P    = temp F^T

        const float accel_var = params.sigma_accel * params.sigma_accel;
        const float bias_var = params.sigma_accel_bias * params.sigma_accel_bias;
        for (int axis = 0; axis < 3; axis++) {
            p_[kPosEast + axis][kPosEast + axis] += accel_var * dt * dt * dt * dt / 4.0f;
            p_[kPosEast + axis][kVelEast + axis] += accel_var * dt * dt * dt / 2.0f;
            p_[kVelEast + axis][kPosEast + axis] += accel_var * dt * dt * dt / 2.0f;
            p_[kVelEast + axis][kVelEast + axis] += accel_var * dt * dt;
            p_[kBiasX + axis][kBiasX + axis] += bias_var * dt;
        }
    }

    static void MultiplyMatrices(const float a[kNumStates][kNumStates],
                                 const float b[kNumStates][kNumStates],
                                 float out[kNumStates][kNumStates]) {
        for (int i = 0; i < kNumStates; i++) {
            for (int j = 0; j < kNumStates; j++) {
                float sum = 0.0f;
                for (int k = 0; k < kNumStates; k++) {
                    sum += a[i][k] * b[k][j];
                }
                out[i][j] = sum;
            }
        }
    }

    static void MultiplyByTranspose(const float a[kNumStates][kNumStates],
                                    const float b[kNumStates][kNumStates],
                                    float out[kNumStates][kNumStates]) {
        for (int i = 0; i < kNumStates; i++) {
            for (int j = 0; j < kNumStates; j++) {
                float sum = 0.0f;
                for (int k = 0; k < kNumStates; k++) {
                    sum += a[i][k] * b[j][k];
                }
                out[i][j] = sum;
            }
        }
    }

    // Rounding in the covariance update slowly breaks the symmetry P must
    // have; averaging with the transpose costs little and keeps it valid.
    void Symmetrize() {
        for (int i = 0; i < kNumStates; i++) {
            for (int j = i + 1; j < kNumStates; j++) {
                const float mean = 0.5f * (p_[i][j] + p_[j][i]);
                p_[i][j] = mean;
                p_[j][i] = mean;
            }
        }
    }

    float x_[kNumStates];
    float p_[kNumStates][kNumStates];
};
