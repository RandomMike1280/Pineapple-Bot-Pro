#ifndef MOTION_QUEUE_HPP
#define MOTION_QUEUE_HPP

#include <Arduino.h>
#include "udp_protocol.hpp"

// ============================================================================
// Motion Queue — converts high-level move intents into timed motor segments
// ============================================================================
// Commands like "move right 500mm at fast speed" are enqueued and executed
// back-to-back with no stops between them.  Each segment knows its target
// position and the queue transitions seamlessly from one to the next.
//
// The queue runs on Core 1 at ~333 Hz.  Call tick() every control loop
// iteration to advance the queue and compute the current velocity target.
// ============================================================================

#define MQ_MAX_SEGMENTS 16

// ============================================================================
// S-Curve Trajectory Profile — jerk-limited velocity shaping
// ============================================================================
// Each non-duration segment gets a profiler that ramps acceleration via
// bounded jerk, producing smooth S-shaped velocity curves.
struct SCurveProfile {
    float maxAccel;      // mm/s² or deg/s² — peak acceleration (ramp-up)
    float maxDecel;      // mm/s² or deg/s² — peak deceleration (ramp-down)
    float maxJerk;       // mm/s³ or deg/s³ — rate of accel change (ramp-up)
    float maxDecelJerk;  // mm/s³ or deg/s³ — rate of accel change (ramp-down)
    float currentAccel;  // current instantaneous acceleration
    float profiledSpeed; // output speed after S-curve shaping
    float prevProfiledSpeed; // previous tick's profiled speed (for feedforward)

    SCurveProfile()
        : maxAccel(250.0f), maxDecel(500.0f), maxJerk(1200.0f), maxDecelJerk(3000.0f),
          currentAccel(0), profiledSpeed(0), prevProfiledSpeed(0) {}

    void reset(float ma, float mj) {
        maxAccel = ma; maxJerk = mj;
        // Decel is 2x accel, decel-jerk is 2.5x — allow aggressive braking
        maxDecel = ma * 2.0f; maxDecelJerk = mj * 2.5f;
        currentAccel = 0; profiledSpeed = 0; prevProfiledSpeed = 0;
    }

    /// Advance one tick: ramp profiledSpeed toward targetSpeed with jerk limit.
    /// Uses asymmetric limits — deceleration is faster than acceleration.
    float update(float targetSpeed, float dt_s) {
        if (dt_s < 0.0001f) return profiledSpeed;
        prevProfiledSpeed = profiledSpeed;
        float speedError = targetSpeed - profiledSpeed;
        bool decelerating = (targetSpeed < profiledSpeed);
        // Pick asymmetric limits
        float effAccel = decelerating ? maxDecel : maxAccel;
        float effJerk  = decelerating ? maxDecelJerk : maxJerk;
        // Desired acceleration to close the gap
        float desiredAccel = speedError / dt_s;
        if (desiredAccel >  effAccel) desiredAccel =  effAccel;
        if (desiredAccel < -effAccel) desiredAccel = -effAccel;
        // Jerk-limit: change currentAccel toward desiredAccel
        float accelError = desiredAccel - currentAccel;
        float maxJerkStep = effJerk * dt_s;
        if (accelError >  maxJerkStep) accelError =  maxJerkStep;
        if (accelError < -maxJerkStep) accelError = -maxJerkStep;
        currentAccel += accelError;
        profiledSpeed += currentAccel * dt_s;
        // Don't overshoot target
        if ((speedError > 0 && profiledSpeed > targetSpeed) ||
            (speedError < 0 && profiledSpeed < targetSpeed)) {
            profiledSpeed = targetSpeed;
            currentAccel = 0; // snap accel to zero when we hit target speed
        }
        if (profiledSpeed < 0) profiledSpeed = 0;
        return profiledSpeed;
    }
};

// ============================================================================
// 1D Kalman Filter for position+velocity estimation (per axis)
// ============================================================================
struct KalmanAxis {
    float x;    // estimated position
    float v;    // estimated velocity
    float p00, p01, p10, p11;  // 2x2 covariance matrix
    float qPos, qVel, rMeas;   // noise parameters

    KalmanAxis() : x(0), v(0), p00(1), p01(0), p10(0), p11(1),
                   qPos(0.5f), qVel(50.0f), rMeas(2.0f) {}

    void reset(float pos, float vel, float qp, float qv, float r) {
        x = pos; v = vel;
        p00 = 1; p01 = 0; p10 = 0; p11 = 1;
        qPos = qp; qVel = qv; rMeas = r;
    }

    void predict(float dt) {
        // State prediction: x += v*dt
        x += v * dt;
        // Covariance prediction
        float np00 = p00 + dt * (p10 + p01) + dt * dt * p11 + qPos;
        float np01 = p01 + dt * p11;
        float np10 = p10 + dt * p11;
        float np11 = p11 + qVel;
        p00 = np00; p01 = np01; p10 = np10; p11 = np11;
    }

    void correct(float measuredPos) {
        // Innovation
        float y = measuredPos - x;
        float s = p00 + rMeas;
        if (s < 1e-6f) s = 1e-6f;
        // Kalman gains
        float k0 = p00 / s;
        float k1 = p10 / s;
        // State update
        x += k0 * y;
        v += k1 * y;
        // Covariance update
        float np00 = (1.0f - k0) * p00;
        float np01 = (1.0f - k0) * p01;
        float np10 = p10 - k1 * p00;
        float np11 = p11 - k1 * p01;
        p00 = np00; p01 = np01; p10 = np10; p11 = np11;
    }
};

enum class SegmentState : uint8_t {
    PENDING,
    ACTIVE,
    HOLDING,
    COMPLETED
};

struct MotionSegment {
    MoveDirection    direction;
    uint16_t         distance_mm;
    SpeedLevel       speed;
    CorrectionPolicy correctionPolicy;
    ServoAction      servoAction;

    // Computed at enqueue time
    float target_x;      // absolute target position (mm)
    float target_y;
    float target_angle;  // absolute target angle (deg)
    float start_x;       // position at segment start
    float start_y;
    float start_angle;
    float vx_mm_s;       // velocity vector (mm/s)
    float vy_mm_s;
    float omega_deg_s;   // angular velocity (deg/s)
    float speed_mm_s;    // scalar speed (mm/s)
    float speed_deg_s;   // scalar rotation speed (deg/s) for rotation segments

    // Duration-based move fields
    bool           isDurationBased;  // true = complete after duration_ms, false = complete after distance_mm
    uint32_t       duration_ms;      // target duration (ms) for duration-based moves
    uint32_t       elapsed_ms;       // elapsed time (ms) for duration-based moves

    // Runtime state
    SegmentState state;
    float        traveled_mm;   // distance traveled in this segment so far
    float        deferred_correction_x;  // accumulated drift for deferred policy
    float        deferred_correction_y;
    float        deferred_correction_angle;

    MotionSegment() : direction(MoveDirection::INVALID), distance_mm(0),
                      speed(SpeedLevel::NORMAL), correctionPolicy(CorrectionPolicy::LIVE),
                      servoAction(ServoAction::NONE),
                      target_x(0), target_y(0), target_angle(0), start_x(0), start_y(0), start_angle(0),
                      vx_mm_s(0), vy_mm_s(0), omega_deg_s(0), speed_mm_s(0), speed_deg_s(0),
                      isDurationBased(false), duration_ms(0), elapsed_ms(0),
                      state(SegmentState::PENDING), traveled_mm(0),
                      deferred_correction_x(0), deferred_correction_y(0), deferred_correction_angle(0) {}
};

class MotionQueue {
public:
    MotionQueue();

    /// Set calibrated speed constants (call once at startup)
    void setSpeedCalibration(float slow_mm_s, float normal_mm_s, float fast_mm_s);

    /// Set calibrated rotation speed constants (call once at startup)
    void setRotationCalibration(float slow_deg_s, float normal_deg_s, float fast_deg_s);

    /// Set distance scaling factors for dead-reckoning mapping
    void setDistanceFactors(float factor_h, float factor_v);

    /// Set precision and deceleration parameters
    void setPrecisionParameters(float deccel_dist_mm, float rot_deccel_deg, 
                                float min_speed_mm_s, float min_rot_deg_s,
                                float precision_min_speed_mm_s, float precision_min_rot_deg_s,
                                float close_approach_dist_mm, float close_rot_approach_deg,
                                float waypoint_tol_mm, float rot_tol_deg,
                                float rot_stab_gain, float max_stab_omega);

    /// Set predictive steering parameters
    void setPredictiveParameters(float lookahead_time_s);

    /// Set S-curve trajectory profile parameters
    void setSCurveParameters(float max_accel_mm_s2, float max_jerk_mm_s3,
                             float max_rot_accel_deg_s2, float max_rot_jerk_deg_s3);

    /// Set feedforward compensation gains
    void setFeedforwardGains(float kv, float ka, float kv_rot, float ka_rot);

    /// Set adaptive lookahead parameters
    void setAdaptiveLookahead(float base_s, float gain, float min_s, float max_s);

    /// Set Kalman filter noise parameters
    void setKalmanParameters(float q_pos, float q_vel, float r_meas);

    /// Get the feedforward compensation values (for motor layer)
    void getFeedforward(float &ff_vx, float &ff_vy, float &ff_omega) const;

    /// Set predictive braking parameters (physics-based overshoot prevention)
    void setPredictiveBraking(float decel_mm_s2, float safety_factor);

    /// Set anti-slip / stall detection parameters
    void setSlipDetection(float cmd_thresh, float obs_thresh, int detect_ticks,
                          float boost_factor, int boost_max_ticks);

    /// Returns true if slip/stall was detected on the current tick
    bool isSlipDetected() const;

    /// Get the EMA-smoothed observed velocity (mm/s)
    void getEstimatedVelocity(float &vx, float &vy) const;

    /// Enqueue a new motion segment.
    /// @param currentX, currentY, currentAngle current estimated position
    /// @returns true if enqueued, false if queue is full
    bool enqueue(MoveDirection direction, uint16_t distance_mm,
                 SpeedLevel speed, CorrectionPolicy policy, ServoAction action,
                 float currentX, float currentY, float currentAngle);

    /// Enqueue a new waypoint motion segment (absolute coordinate).
    /// @param currentX, currentY, currentAngle current estimated position
    /// @returns true if enqueued, false if queue is full
    bool enqueueWaypoint(float target_x, float target_y, float targetAngle,
                         SpeedLevel speed, CorrectionPolicy policy, ServoAction action,
                         float currentX, float currentY, float currentAngle);

    /// Enqueue a duration-based move segment (move for a fixed time).
    /// @returns true if enqueued, false if queue is full
    bool enqueueDuration(MoveDirection direction, uint32_t duration_ms,
                         SpeedLevel speed, CorrectionPolicy policy,
                         float currentX, float currentY, float currentAngle);

    /// Enqueue a duration-based rotation segment (rotate for a fixed time).
    /// @returns true if enqueued, false if queue is full
    bool enqueueRotateDuration(RotationDirection direction, uint32_t duration_ms,
                               SpeedLevel speed, CorrectionPolicy policy,
                               float currentX, float currentY, float currentAngle);

    /// Enqueue a direct velocity segment (phone-driven PID output).
    /// The robot will move at the given world-frame velocities for up
    /// to timeout_ms, then auto-stop.  Aborts existing queue first.
    /// @returns true if enqueued, false if queue is full
    bool enqueueVelocity(float vx_mm_s, float vy_mm_s, float omega_deg_s,
                         uint32_t timeout_ms,
                         float currentX, float currentY, float currentAngle);

    /// Enqueue a new rotation motion segment (absolute orientation).
    /// @returns true if enqueued, false if queue is full
    bool enqueueRotate(float target_angle,
                       SpeedLevel speed, CorrectionPolicy policy,
                       float currentX, float currentY, float currentAngle);

    /// Called every control-loop tick.  Returns true if there is an active
    /// segment being executed.
    /// @param dt_ms elapsed time since last call
    bool tick(uint32_t dt_ms, float current_x, float current_y, float current_angle);

    /// Get the current motion velocity to apply to motors.
    /// Returns (0, 0, 0) if no segment is active.
    void getCurrentVelocity(float &vx_mm_s, float &vy_mm_s, float &omega_deg_s) const;

    /// Get the correction policy of the active segment
    CorrectionPolicy getActivePolicy() const;

    /// Store a deferred correction for the current segment
    void storeDeferredCorrection(float errX, float errY, float errAngle);

    /// Get deferred correction (when segment finishes)
    void getDeferredCorrection(float &errX, float &errY, float &errAngle) const;

    /// Get the servo action of the segment that just completed
    ServoAction getLastCompletedServoAction() const;

    /// Number of segments remaining (including active)
    int  remaining() const;

    /// Is the queue empty and no segment active?
    bool isEmpty() const;

    /// Emergency stop: abort current segment and clear queue
    void abort();

    /// Get the current active segment (or null)
    const MotionSegment* currentSegment() const;

    // Flag: set to true when a segment just completed (for deferred correction)
    bool segmentJustCompleted;

private:
    // === Queue State ===
    MotionSegment _segments[MQ_MAX_SEGMENTS];
    int           _head;                    // index of the current/next segment to execute
    int           _tail;                    // index of next free slot
    int           _count;                   // number of segments in queue (including active)

    // === Calibration ===
    float _speedSlow;                       // mm/s at SLOW speed level
    float _speedNormal;                      // mm/s at NORMAL speed level
    float _speedFast;                        // mm/s at FAST speed level
    float _rotSlow;                         // deg/s at SLOW speed level
    float _rotNormal;                       // deg/s at NORMAL speed level
    float _rotFast;                          // deg/s at FAST speed level
    float _distFactorH;                      // horizontal (strafe) distance factor
    float _distFactorV;                      // vertical (forward/back) distance factor

    // === Runtime State ===
    float _currentVx;                        // current world-frame velocity, x (mm/s)
    float _currentVy;                        // current world-frame velocity, y (mm/s)
    float _currentOmega;                     // current angular velocity (deg/s)

    // === Deceleration & Precision ===
    float _deccelDistMm;                     // start decelerating at this distance to target
    float _rotDeccelDeg;                    // start decelerating rotation at this angle
    float _minSpeedLimitMmS;                 // floor speed during deceleration
    float _minRotLimitDegS;                  // floor rotation speed during deceleration
    float _precisionMinSpeedLimitMmS;
    float _precisionMinRotLimitDegS;
    float _closeApproachDistMm;
    float _closeRotApproachDeg;
    float _waypointToleranceMm;
    float _rotToleranceDeg;
    float _rotStabilizationGain;
    float _maxStabilizationOmega;

    // === Velocity Estimation ===
    float _prevPoseX, _prevPoseY;
    bool  _hasPreviousPose;
    float _estVx, _estVy;                    // Kalman-estimated velocity (mm/s)
    float _lookaheadTimeS;                   // base lookahead (seconds)
    uint32_t _tickTimeMs;                   // monotonic tick timer

    // === S-Curve Profiler ===
    SCurveProfile _scurveTrans;
    SCurveProfile _scurveRot;
    float _scurveMaxAccelMmS2;
    float _scurveMaxJerkMmS3;
    float _scurveMaxRotAccelDegS2;
    float _scurveMaxRotJerkDegS3;

    // === Feedforward ===
    float _ffKv, _ffKa;                     // linear velocity/acceleration feedforward
    float _ffKvRot, _ffKaRot;              // rotational velocity/acceleration feedforward
    float _ffVx, _ffVy, _ffOmega;           // computed feedforward outputs

    // === Adaptive Lookahead ===
    float _adaptLookaheadBaseS;
    float _adaptLookaheadGain;
    float _adaptLookaheadMinS;
    float _adaptLookaheadMaxS;

    // === Kalman Filter ===
    KalmanAxis _kalmanX;
    KalmanAxis _kalmanY;
    bool _kalmanInitialized;

    // === Predictive Braking ===
    float _predictiveBrakeDecel;             // assumed deceleration capability (mm/s^2)
    float _predictiveBrakeSafety;            // safety margin factor

    // === Anti-Slip Detection ===
    float _slipCmdThresh;                   // min commanded speed to monitor slip
    float _slipObsThresh;                   // observed speed below this = stalled
    int   _slipDetectTicks;                  // consecutive ticks to confirm stall
    float _slipBoostFactor;                  // multiplicative boost on stall
    int   _slipBoostMaxTicks;                // max boost duration
    int   _slipCounter;                      // running count of suspect ticks
    int   _slipBoostRemaining;                // remaining boost ticks
    bool  _slipDetected;                     // flag for telemetry

    float _getSpeedMmS(SpeedLevel level) const;
    float _getSpeedDegS(SpeedLevel level) const;
    void  _advanceToNext();
    void  _updateVelocityEstimate(float x, float y, float dt_s);
    float _computeAdaptiveLookahead(float currentSpeed) const;
};

#endif // MOTION_QUEUE_HPP
