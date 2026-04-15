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
// ============================================================================

#define MQ_MAX_SEGMENTS 16

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
    MotionSegment _segments[MQ_MAX_SEGMENTS];
    int           _head;      // index of the current/next segment to execute
    int           _tail;      // index of next free slot
    int           _count;     // number of segments in queue (including active)

    // Speed calibration
    float _speedSlow;
    float _speedNormal;
    float _speedFast;

    // Rotation speed calibration
    float _rotSlow;
    float _rotNormal;
    float _rotFast;

    // Distance Calibration Scaling
    float _distFactorH;
    float _distFactorV;

    // Active runtime state (filtered/decelerated)
    float _currentVx;
    float _currentVy;
    float _currentOmega;

    // Tuning Parameters (Injected from main.h)
    float _deccelDistMm;
    float _rotDeccelDeg;
    float _minSpeedLimitMmS;
    float _minRotLimitDegS;
    float _precisionMinSpeedLimitMmS;
    float _precisionMinRotLimitDegS;
    float _closeApproachDistMm;
    float _closeRotApproachDeg;
    float _waypointToleranceMm;
    float _rotToleranceDeg;
    float _rotStabilizationGain;
    float _maxStabilizationOmega;

    // Velocity tracking for predictive steering
    float _prevPoseX, _prevPoseY;
    bool  _hasPreviousPose;
    float _estVx, _estVy;           // EMA-smoothed observed velocity (mm/s)
    float _lookaheadTimeS;           // how far ahead to predict (seconds)
    uint32_t _tickTimeMs;            // monotonic tick timer

    float _getSpeedMmS(SpeedLevel level) const;
    float _getSpeedDegS(SpeedLevel level) const;
    void  _advanceToNext();
    void  _updateVelocityEstimate(float x, float y, float dt_s);
};

#endif // MOTION_QUEUE_HPP
