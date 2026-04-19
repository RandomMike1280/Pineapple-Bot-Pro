#include "motion_queue.hpp"
#include <math.h>
#include "../Rotation/Rotation.hpp"
#include "../Common/opt_math.hpp"

// ============================================================================
// Agent Debug Logging — Serial NDJSON for long-run diagnostics
// H2: Kalman covariance trace (p00+p11 per axis)
// H4: profiledSpeed sanity (detect NaN or overflow)
// ============================================================================
static unsigned long _lastMqDiagLogMs = 0;
static const unsigned long MQ_DIAG_LOG_INTERVAL_MS = 5000;

static void _dbgLogMQDiag(
    float kalmanTraceX, float kalmanTraceY,
    float profiledTransSpeed, float profiledRotSpeed,
    uint32_t tickTimeMs, int count,
    const char* segState)
{
    unsigned long now = millis();
    if (now - _lastMqDiagLogMs < MQ_DIAG_LOG_INTERVAL_MS) return;
    _lastMqDiagLogMs = now;

    bool transNaN = isnan(profiledTransSpeed) || isinf(profiledTransSpeed);
    bool rotNaN = isnan(profiledRotSpeed) || isinf(profiledRotSpeed);

    Serial.printf(
        "{\"sessionId\":\"eb5734\",\"id\":\"mq_%lu\",\"timestamp\":%lu,"
        "\"location\":\"motion_queue.cpp:tick\",\"message\":\"H2_H4_diagnostics\",\"hypothesisId\":\"H2+H4\","
        "\"data\":{\"kalmanTraceX\":%.4f,\"kalmanTraceY\":%.4f,"
        "\"profiledTransSpd\":%.3f,\"profiledRotSpd\":%.3f,"
        "\"transNaN\":%d,\"rotNaN\":%d,"
        "\"tickTimeMs\":%lu,\"qCount\":%d,\"segState\":\"%s\"}}\n",
        now, now,
        (double)kalmanTraceX, (double)kalmanTraceY,
        (double)profiledTransSpeed, (double)profiledRotSpeed,
        transNaN, rotNaN,
        (unsigned long)tickTimeMs, count, segState);
}

// ============================================================================
// Construction

// ============================================================================

MotionQueue::MotionQueue()
    : _head(0), _tail(0), _count(0),
      _speedSlow(29.0f), _speedNormal(43.5f), _speedFast(72.5f),
      _rotSlow(34.56f), _rotNormal(51.84f), _rotFast(86.4f),
      _distFactorH(1.0f), _distFactorV(1.0f),
      _currentVx(0), _currentVy(0), _currentOmega(0),
      _deccelDistMm(150.0f), _rotDeccelDeg(25.0f),
      _minSpeedLimitMmS(25.0f), _minRotLimitDegS(25.0f),
      _precisionMinSpeedLimitMmS(10.0f), _precisionMinRotLimitDegS(4.0f),
      _closeApproachDistMm(80.0f), _closeRotApproachDeg(8.0f),
      _waypointToleranceMm(12.0f), _rotToleranceDeg(2.0f),
      _rotStabilizationGain(2.5f), _maxStabilizationOmega(35.0f),
      _prevPoseX(0), _prevPoseY(0), _hasPreviousPose(false),
      _estVx(0), _estVy(0), _emaVx(0), _emaVy(0),
      _lookaheadTimeS(0.15f), _tickTimeMs(0),
      _scurveMaxAccelMmS2(250.0f), _scurveMaxJerkMmS3(1200.0f),
      _scurveMaxRotAccelDegS2(180.0f), _scurveMaxRotJerkDegS3(900.0f),
      _ffKv(0.02f), _ffKa(0.04f), _ffKvRot(0.01f), _ffKaRot(0.02f),
      _ffVx(0), _ffVy(0), _ffOmega(0),
      _adaptLookaheadBaseS(0.06f), _adaptLookaheadGain(0.0022f),
      _adaptLookaheadMinS(0.04f), _adaptLookaheadMaxS(0.25f),
      _kalmanInitialized(false),
      _predictiveBrakeDecel(200.0f), _predictiveBrakeSafety(1.5f),
      _latencyAwareDecelMmS2(0.0f),
      _slipCmdThresh(15.0f), _slipObsThresh(5.0f), _slipDetectTicks(25),
      _slipBoostFactor(1.35f), _slipBoostMaxTicks(40),
      _slipCounter(0), _slipBoostRemaining(0), _slipDetected(false),
      segmentJustCompleted(false),
      _blendedAngle(0.0f), _angleBlendInitialized(false)
{
    _scurveTrans.reset(_scurveMaxAccelMmS2, _scurveMaxJerkMmS3);
    _scurveRot.reset(_scurveMaxRotAccelDegS2, _scurveMaxRotJerkDegS3);
}

void MotionQueue::setSpeedCalibration(float slow_mm_s, float normal_mm_s, float fast_mm_s) {
    _speedSlow   = slow_mm_s;
    _speedNormal = normal_mm_s;
    _speedFast   = fast_mm_s;
}

void MotionQueue::setRotationCalibration(float slow_deg_s, float normal_deg_s, float fast_deg_s) {
    _rotSlow   = slow_deg_s;
    _rotNormal = normal_deg_s;
    _rotFast   = fast_deg_s;
}

void MotionQueue::setDistanceFactors(float factor_h, float factor_v) {
    _distFactorH = factor_h;
    _distFactorV = factor_v;
}

void MotionQueue::setPrecisionParameters(float deccel_dist_mm, float rot_deccel_deg, 
                                          float min_speed_mm_s, float min_rot_deg_s,
                                          float precision_min_speed_mm_s, float precision_min_rot_deg_s,
                                          float close_approach_dist_mm, float close_rot_approach_deg,
                                          float waypoint_tol_mm, float rot_tol_deg,
                                          float rot_stab_gain, float max_stab_omega) {
    _deccelDistMm = deccel_dist_mm;
    _rotDeccelDeg = rot_deccel_deg;
    _minSpeedLimitMmS = min_speed_mm_s;
    _minRotLimitDegS = min_rot_deg_s;
    _precisionMinSpeedLimitMmS = precision_min_speed_mm_s;
    _precisionMinRotLimitDegS = precision_min_rot_deg_s;
    _closeApproachDistMm = close_approach_dist_mm;
    _closeRotApproachDeg = close_rot_approach_deg;
    _waypointToleranceMm = waypoint_tol_mm;
    _rotToleranceDeg = rot_tol_deg;
    _rotStabilizationGain = rot_stab_gain;
    _maxStabilizationOmega = max_stab_omega;
}

void MotionQueue::setPredictiveParameters(float lookahead_time_s) {
    // NOTE: parameter shadows member _lookaheadTimeS — assignment is intentional
    _lookaheadTimeS = lookahead_time_s;
}

void MotionQueue::setSCurveParameters(float max_accel_mm_s2, float max_jerk_mm_s3,
                                       float max_rot_accel_deg_s2, float max_rot_jerk_deg_s3) {
    _scurveMaxAccelMmS2 = max_accel_mm_s2;
    _scurveMaxJerkMmS3 = max_jerk_mm_s3;
    _scurveMaxRotAccelDegS2 = max_rot_accel_deg_s2;
    _scurveMaxRotJerkDegS3 = max_rot_jerk_deg_s3;
    _scurveTrans.reset(max_accel_mm_s2, max_jerk_mm_s3);
    _scurveRot.reset(max_rot_accel_deg_s2, max_rot_jerk_deg_s3);
}

void MotionQueue::setFeedforwardGains(float kv, float ka, float kv_rot, float ka_rot) {
    _ffKv = kv; _ffKa = ka; _ffKvRot = kv_rot; _ffKaRot = ka_rot;
}

void MotionQueue::setAdaptiveLookahead(float base_s, float gain, float min_s, float max_s) {
    _adaptLookaheadBaseS = base_s;
    _adaptLookaheadGain = gain;
    _adaptLookaheadMinS = min_s;
    _adaptLookaheadMaxS = max_s;
}

void MotionQueue::setKalmanParameters(float q_pos, float q_vel, float r_meas) {
    _kalmanX.qPos = q_pos; _kalmanX.qVel = q_vel; _kalmanX.rMeas = r_meas;
    _kalmanY.qPos = q_pos; _kalmanY.qVel = q_vel; _kalmanY.rMeas = r_meas;
}

void MotionQueue::regroundPosition(float cam_x, float cam_y) {
    _kalmanX.setPosition(cam_x);
    _kalmanY.setPosition(cam_y);
    _estVx = 0.0f;
    _estVy = 0.0f;
    _prevPoseX = cam_x;
    _prevPoseY = cam_y;
}

void MotionQueue::setPredictiveBraking(float decel_mm_s2, float safety_factor) {
    _predictiveBrakeDecel = decel_mm_s2;
    _predictiveBrakeSafety = safety_factor;
}

void MotionQueue::setLatencyAwareDecel(float decel_mm_s2) {
    _latencyAwareDecelMmS2 = decel_mm_s2;
}

void MotionQueue::getFeedforward(float &ff_vx, float &ff_vy, float &ff_omega) const {
    ff_vx = _ffVx; ff_vy = _ffVy; ff_omega = _ffOmega;
}

void MotionQueue::setSlipDetection(float cmd_thresh, float obs_thresh, int detect_ticks,
                                    float boost_factor, int boost_max_ticks) {
    _slipCmdThresh = cmd_thresh;
    _slipObsThresh = obs_thresh;
    _slipDetectTicks = detect_ticks;
    _slipBoostFactor = boost_factor;
    _slipBoostMaxTicks = boost_max_ticks;
}

bool MotionQueue::isSlipDetected() const {
    return _slipDetected;
}

float MotionQueue::_computeAdaptiveLookahead(float currentSpeed) const {
    float la = _adaptLookaheadBaseS + _adaptLookaheadGain * currentSpeed;
    if (la < _adaptLookaheadMinS) la = _adaptLookaheadMinS;
    if (la > _adaptLookaheadMaxS) la = _adaptLookaheadMaxS;
    return la;
}

void MotionQueue::getEstimatedVelocity(float &vx, float &vy) const {
    vx = _estVx;
    vy = _estVy;
}

bool MotionQueue::getBlendedAngle(float &out_angle) const {
    if (!_angleBlendInitialized) return false;
    out_angle = _blendedAngle;
    return true;
}

void MotionQueue::getEmaVelocity(float &vx, float &vy) const {
    vx = _emaVx;
    vy = _emaVy;
}

void MotionQueue::_updateVelocityEstimate(float x, float y, float dt_s) {
    if (dt_s < 0.001f) return;

    if (!_kalmanInitialized) {
        _kalmanX.reset(x, 0.0f, _kalmanX.qPos, _kalmanX.qVel, _kalmanX.rMeas);
        _kalmanY.reset(y, 0.0f, _kalmanY.qPos, _kalmanY.qVel, _kalmanY.rMeas);
        _kalmanInitialized = true;
        _prevPoseX = x;
        _prevPoseY = y;
        _hasPreviousPose = true;
        return;
    }

    // Reject outlier spikes from camera correction jumps
    float jumpDist = fastLength2(x - _prevPoseX, y - _prevPoseY);
    float maxJump = _speedFast * 3.0f * dt_s;
    _prevPoseX = x;
    _prevPoseY = y;
    _hasPreviousPose = true;

    if (jumpDist > maxJump) {
        // Outlier — re-anchor Kalman without updating velocity
        _kalmanX.x = x;
        _kalmanY.x = y;
        return;
    }

    // Kalman predict step (propagate state forward by dt)
    // Only called after initialization — predict on uninitialized Kalman
    // would propagate undefined covariance values from reset()
    _kalmanX.predict(dt_s);
    _kalmanY.predict(dt_s);

    // Kalman correct step (fuse position measurement)
    _kalmanX.correct(x);
    _kalmanY.correct(y);

    // Stationary deadzone: kill velocity estimate noise when nearly stopped
    float est_speed = fastLength2(_kalmanX.v, _kalmanY.v);
    const float deadzoneMmS = 5.0f;
    if (est_speed < deadzoneMmS) {
        _kalmanX.v = 0.0f;
        _kalmanY.v = 0.0f;
    }

    _estVx = _kalmanX.v;
    _estVy = _kalmanY.v;
}

float MotionQueue::_getSpeedMmS(SpeedLevel level) const {
    switch (level) {
        case SpeedLevel::SLOW:   return _speedSlow;
        case SpeedLevel::NORMAL: return _speedNormal;
        case SpeedLevel::FAST:   return _speedFast;
        default:                 return _speedNormal;
    }
}

float MotionQueue::_getSpeedDegS(SpeedLevel level) const {
    switch (level) {
        case SpeedLevel::SLOW:   return _rotSlow;
        case SpeedLevel::NORMAL: return _rotNormal;
        case SpeedLevel::FAST:   return _rotFast;
        default:                 return _rotNormal;
    }
}

// ============================================================================
// Enqueue a new motion segment
// ============================================================================

bool MotionQueue::enqueueWaypoint(float target_x, float target_y, float targetAngle,
                                  SpeedLevel speed, CorrectionPolicy policy, ServoAction action,
                                  float currentX, float currentY, float currentAngle) {
    if (_count >= MQ_MAX_SEGMENTS) return false;

    MotionSegment &seg = _segments[_tail % MQ_MAX_SEGMENTS];
    seg.direction       = MoveDirection::INVALID;  // Not a discrete direction
    seg.speed           = speed;
    seg.correctionPolicy = policy;
    seg.servoAction     = action;
    seg.state           = SegmentState::PENDING;
    seg.traveled_mm     = 0;
    seg.isDurationBased = false;
    seg.deferred_correction_x = 0;
    seg.deferred_correction_y = 0;
    seg.deferred_correction_angle = 0;

    // Determine start position for this segment
    if (_count == 0) {
        seg.start_x = currentX;
        seg.start_y = currentY;
        seg.start_angle = currentAngle;
    } else {
        const MotionSegment &prev = _segments[(_tail - 1 + MQ_MAX_SEGMENTS) % MQ_MAX_SEGMENTS];
        seg.start_x = prev.target_x;
        seg.start_y = prev.target_y;
        seg.start_angle = prev.target_angle;
    }

    seg.target_angle = targetAngle; // Set target angle from parameter
    seg.target_x = target_x;
    seg.target_y = target_y;

    // Calculate displacement vector
    float dx = target_x - seg.start_x;
    float dy = target_y - seg.start_y;
    float dist = fastLength2(dx, dy);
    seg.distance_mm = (uint16_t)dist;

    // Calculate normalized direction vector
    float dirX = 0, dirY = 0;
    if (dist > 0.001f) {
        dirX = dx / dist;
        dirY = dy / dist;
    }

    float spd = _getSpeedMmS(speed);
    seg.speed_mm_s = spd;
    seg.vx_mm_s = dirX * spd;
    seg.vy_mm_s = dirY * spd;
    seg.omega_deg_s = 0;

    _tail = (_tail + 1) % MQ_MAX_SEGMENTS;
    _count++;
    return true;
}

bool MotionQueue::enqueueRotate(float target_angle,
                                  SpeedLevel speed, CorrectionPolicy policy,
                                  float currentX, float currentY, float currentAngle) {
    if (_count >= MQ_MAX_SEGMENTS) return false;

    MotionSegment &seg = _segments[_tail % MQ_MAX_SEGMENTS];
    seg.direction       = MoveDirection::INVALID;
    seg.speed           = speed;
    seg.correctionPolicy = policy;
    seg.state           = SegmentState::PENDING;
    seg.traveled_mm     = 0;
    seg.isDurationBased = false;
    seg.deferred_correction_x = 0;
    seg.deferred_correction_y = 0;
    seg.deferred_correction_angle = 0;

    if (_count == 0) {
        seg.start_x = currentX;
        seg.start_y = currentY;
        seg.start_angle = currentAngle;
    } else {
        const MotionSegment &prev = _segments[(_tail - 1 + MQ_MAX_SEGMENTS) % MQ_MAX_SEGMENTS];
        seg.start_x = prev.target_x;
        seg.start_y = prev.target_y;
        seg.start_angle = prev.target_angle;
    }

    seg.target_x = seg.start_x;
    seg.target_y = seg.start_y;
    seg.target_angle = target_angle;

    float d_angle = (Rotation(target_angle) - Rotation(seg.start_angle));

    seg.distance_mm = (uint16_t)abs(d_angle);
    float rotSpeed = _getSpeedDegS(speed);
    float linSpeed = _getSpeedMmS(speed);
    seg.speed_mm_s = linSpeed;
    seg.speed_deg_s = rotSpeed;
    seg.vx_mm_s = 0;
    seg.vy_mm_s = 0;
    if (abs(d_angle) > 0.001f) {
        seg.omega_deg_s = (d_angle > 0) ? rotSpeed : -rotSpeed;
    } else {
        seg.omega_deg_s = 0;
    }

    _tail = (_tail + 1) % MQ_MAX_SEGMENTS;
    _count++;
    return true;
}
// ============================================================================

bool MotionQueue::enqueueDuration(MoveDirection direction, uint32_t duration_ms,
                                       SpeedLevel speed, CorrectionPolicy policy,
                                       float currentX, float currentY, float currentAngle) {
    if (_count >= MQ_MAX_SEGMENTS) return false;
    // Reject zero-duration moves — they complete immediately and waste a queue slot
    if (duration_ms == 0) return false;

    MotionSegment &seg = _segments[_tail % MQ_MAX_SEGMENTS];
    seg.direction       = direction;
    seg.distance_mm     = 0;  // not used for duration-based
    seg.speed           = speed;
    seg.correctionPolicy = policy;
    seg.state           = SegmentState::PENDING;
    seg.traveled_mm     = 0;
    seg.deferred_correction_x = 0;
    seg.deferred_correction_y = 0;
    seg.deferred_correction_angle = 0;

    // Duration-specific fields
    seg.isDurationBased = true;
    seg.duration_ms     = duration_ms;
    seg.elapsed_ms      = 0;

    // Compute velocity vector
    float dirX, dirY;
    directionToVector(direction, dirX, dirY);
    float spd = _getSpeedMmS(speed);
    seg.speed_mm_s = spd;
    seg.vx_mm_s = dirX * spd;
    seg.vy_mm_s = dirY * spd;

    // Compute start position
    if (_count == 0) {
        seg.start_x = currentX;
        seg.start_y = currentY;
        seg.start_angle = currentAngle;
    } else {
        const MotionSegment &prev = _segments[(_tail - 1 + MQ_MAX_SEGMENTS) % MQ_MAX_SEGMENTS];
        seg.start_x = prev.target_x;
        seg.start_y = prev.target_y;
        seg.start_angle = prev.target_angle;
    }

    // Estimate target position (speed * duration) for chaining
    float estDist = spd * (duration_ms / 1000.0f);
    seg.target_x = seg.start_x + dirX * estDist;
    seg.target_y = seg.start_y + dirY * estDist;
    seg.target_angle = seg.start_angle;
    seg.omega_deg_s = 0;

    _tail = (_tail + 1) % MQ_MAX_SEGMENTS;
    _count++;
    return true;
}

bool MotionQueue::enqueueRotateDuration(RotationDirection direction, uint32_t duration_ms,
                                        SpeedLevel speed, CorrectionPolicy policy,
                                        float currentX, float currentY, float currentAngle) {
    if (_count >= MQ_MAX_SEGMENTS) return false;
    // Reject zero-duration moves — they complete immediately and waste a queue slot
    if (duration_ms == 0) return false;

    MotionSegment &seg = _segments[_tail % MQ_MAX_SEGMENTS];
    seg.direction       = MoveDirection::INVALID;
    seg.distance_mm     = 0;
    seg.speed           = speed;
    seg.correctionPolicy = policy;
    seg.state           = SegmentState::PENDING;
    seg.traveled_mm     = 0;
    seg.deferred_correction_x = 0;
    seg.deferred_correction_y = 0;
    seg.deferred_correction_angle = 0;

    seg.isDurationBased = true;
    seg.duration_ms     = duration_ms;
    seg.elapsed_ms      = 0;

    if (_count == 0) {
        seg.start_x = currentX;
        seg.start_y = currentY;
        seg.start_angle = currentAngle;
    } else {
        const MotionSegment &prev = _segments[(_tail - 1 + MQ_MAX_SEGMENTS) % MQ_MAX_SEGMENTS];
        seg.start_x = prev.target_x;
        seg.start_y = prev.target_y;
        seg.start_angle = prev.target_angle;
    }

    float rotSpeed = _getSpeedDegS(speed);
    seg.speed_mm_s = 0;
    seg.speed_deg_s = rotSpeed;
    seg.vx_mm_s = 0;
    seg.vy_mm_s = 0;
    seg.omega_deg_s = (direction == RotationDirection::CCW) ? rotSpeed : -rotSpeed;

    float estAngleDelta = seg.omega_deg_s * (duration_ms / 1000.0f);
    seg.target_x = seg.start_x;
    seg.target_y = seg.start_y;
    // Normalize to [0, 360) using Rotation constructor
    seg.target_angle = Rotation(seg.start_angle + estAngleDelta);

    _tail = (_tail + 1) % MQ_MAX_SEGMENTS;
    _count++;
    return true;
}

bool MotionQueue::enqueueVelocity(float vx_mm_s, float vy_mm_s, float omega_deg_s,
                                   uint32_t timeout_ms,
                                   float currentX, float currentY, float currentAngle) {
    if (_count >= MQ_MAX_SEGMENTS) return false;

    // Safety: default timeout 200ms if 0
    if (timeout_ms == 0) timeout_ms = 200;

    MotionSegment &seg = _segments[_tail % MQ_MAX_SEGMENTS];
    seg.direction       = MoveDirection::INVALID;
    seg.distance_mm     = 0;
    seg.speed           = SpeedLevel::NORMAL;
    seg.correctionPolicy = CorrectionPolicy::LIVE;
    seg.state           = SegmentState::PENDING;
    seg.traveled_mm     = 0;
    seg.deferred_correction_x = 0;
    seg.deferred_correction_y = 0;
    seg.deferred_correction_angle = 0;

    seg.isDurationBased = true;
    seg.duration_ms     = timeout_ms;
    seg.elapsed_ms      = 0;

    seg.start_x = currentX;
    seg.start_y = currentY;
    seg.start_angle = currentAngle;

    seg.vx_mm_s    = vx_mm_s;
    seg.vy_mm_s    = vy_mm_s;
    seg.omega_deg_s = omega_deg_s;
    seg.speed_mm_s  = fastLength2(vx_mm_s, vy_mm_s);
    seg.speed_deg_s = fabsf(omega_deg_s);

    // Estimate target position for telemetry
    float dt_s = timeout_ms / 1000.0f;
    seg.target_x = currentX + vx_mm_s * dt_s;
    seg.target_y = currentY + vy_mm_s * dt_s;
    seg.target_angle = currentAngle + omega_deg_s * dt_s;

    _tail = (_tail + 1) % MQ_MAX_SEGMENTS;
    _count++;
    return true;
}

bool MotionQueue::enqueue(MoveDirection direction, uint16_t distance_mm,
                          SpeedLevel speed, CorrectionPolicy policy, ServoAction action,
                          float currentX, float currentY, float currentAngle) {
    if (_count >= MQ_MAX_SEGMENTS) return false;

    MotionSegment &seg = _segments[_tail % MQ_MAX_SEGMENTS];
    seg.direction       = direction;
    seg.distance_mm     = distance_mm;
    seg.speed           = speed;
    seg.correctionPolicy = policy;
    seg.servoAction     = action;
    seg.state           = SegmentState::PENDING;
    seg.traveled_mm     = 0;
    seg.isDurationBased = false;
    seg.deferred_correction_x = 0;
    seg.deferred_correction_y = 0;
    seg.deferred_correction_angle = 0;

    // Compute velocity vector
    float dirX, dirY;
    directionToVector(direction, dirX, dirY);
    float spd = _getSpeedMmS(speed);
    seg.speed_mm_s = spd;
    seg.vx_mm_s = dirX * spd;
    seg.vy_mm_s = dirY * spd;

    // Compute target position relative to where queue will be when this
    // segment starts.  For the first segment, that's currentX/Y/Angle.
    // For subsequent segments, chain from previous target.
    if (_count == 0) {
        seg.start_x = currentX;
        seg.start_y = currentY;
        seg.start_angle = currentAngle;
    } else {
        // Chain from the previous segment's target
        const MotionSegment &prev = _segments[(_tail - 1 + MQ_MAX_SEGMENTS) % MQ_MAX_SEGMENTS];
        seg.start_x = prev.target_x;
        seg.start_y = prev.target_y;
        seg.start_angle = prev.target_angle;
    }
    seg.target_x = seg.start_x + dirX * distance_mm;
    seg.target_y = seg.start_y + dirY * distance_mm;
    seg.target_angle = seg.start_angle;
    seg.omega_deg_s = 0;

    _tail = (_tail + 1) % MQ_MAX_SEGMENTS;
    _count++;
    return true;
}

// ============================================================================
// Tick — called every control-loop iteration
// ============================================================================

bool MotionQueue::tick(uint32_t dt_ms, float current_x, float current_y, float current_angle,
                       float groundTruthAngle, bool hasGroundTruth) {
    segmentJustCompleted = false;
    _tickTimeMs += dt_ms;
    float dt_s = dt_ms / 1000.0f;

    // Update velocity estimate from observed position every tick
    _updateVelocityEstimate(current_x, current_y, dt_s);

    // === Angle Blending: fuse camera GT with DR via EMA ===
    // Short-term: smooth DR angle (avoids camera quantization jumps)
    // Long-term: converges to camera angle (eliminates DR drift)
    // This breaks the feedback loop where stabilization targets a drifting DR angle.
    if (hasGroundTruth && !isnan(groundTruthAngle)) {
        if (!_angleBlendInitialized) {
            _blendedAngle = groundTruthAngle;
            _angleBlendInitialized = true;
        } else {
            // EMA blend: 85% old blended, 15% new camera reading
            _blendedAngle = _blendedAngle * 0.85f + groundTruthAngle * 0.15f;
        }
    }

    if (_count == 0) {
        _currentVx = _currentVy = _currentOmega = 0;
        return false;
    }

    MotionSegment &seg = _segments[_head % MQ_MAX_SEGMENTS];

    // Activate pending segment
    if (seg.state == SegmentState::PENDING) {
        seg.state = SegmentState::ACTIVE;
        seg.traveled_mm = 0;
        seg.elapsed_ms = 0;
        // Reset S-curve profilers for clean start on new segment
        _scurveTrans.reset(_scurveMaxAccelMmS2, _scurveMaxJerkMmS3);
        _scurveRot.reset(_scurveMaxRotAccelDegS2, _scurveMaxRotJerkDegS3);
        _ffVx = _ffVy = _ffOmega = 0;
    }

    if (seg.state != SegmentState::ACTIVE && seg.state != SegmentState::HOLDING) {
        _currentVx = _currentVy = _currentOmega = 0;
        return false;
    }

    // Integrate distance traveled
    // Use current velocity for theoretical distance integration
    float displacement = 0;
    if (seg.speed_deg_s > 0 && seg.vx_mm_s == 0 && seg.vy_mm_s == 0) {
        displacement = abs(_currentOmega) * dt_s;
    } else {
        float phys_vx = _currentVx * _distFactorH;
        float phys_vy = _currentVy * _distFactorV;
        displacement = fastLength2(phys_vx, phys_vy) * dt_s;
    }
    seg.traveled_mm += displacement;

    if (seg.isDurationBased) {
        seg.elapsed_ms += dt_ms;
        _currentVx = seg.vx_mm_s;
        _currentVy = seg.vy_mm_s;
        _currentOmega = seg.omega_deg_s;
    } else {
        // --- CLOSED LOOP DECELERATION & TARGETING ---
        bool isRotationOnly = (seg.vx_mm_s == 0 && seg.vy_mm_s == 0 && seg.omega_deg_s != 0);
        float speed_scale = 1.0f;

        if (isRotationOnly) {
            float angle_diff = (Rotation(seg.target_angle) - Rotation(current_angle));
            float abs_diff = fabsf(angle_diff);
            float rotSign = (angle_diff >= 0.0f) ? 1.0f : -1.0f;
            speed_scale = 1.0f;

            if (abs_diff <= _rotToleranceDeg) {
                speed_scale = 0.0f;
            } else {
                if (_rotDeccelDeg > 0.001f && abs_diff < _rotDeccelDeg) {
                    float normalized = abs_diff / _rotDeccelDeg;
                    speed_scale = fastCbrtf(normalized);
                }

                if (_closeRotApproachDeg > 0.001f && abs_diff < _closeRotApproachDeg) {
                    float close_norm = abs_diff / _closeRotApproachDeg;
                    float close_scale = close_norm * close_norm;
                    if (close_scale < speed_scale) speed_scale = close_scale;
                    if (seg.speed_deg_s > 0.001f) {
                        float precision_min_scale = _precisionMinRotLimitDegS / seg.speed_deg_s;
                        if (speed_scale < precision_min_scale) speed_scale = precision_min_scale;
                    }
                } else if (seg.speed_deg_s > 0.001f) {
                    float min_scale = _minRotLimitDegS / seg.speed_deg_s;
                    if (speed_scale < min_scale) speed_scale = min_scale;
                }

                if (_count > 1) speed_scale = 1.0f;
            }

            _currentVx = 0;
            _currentVy = 0;
            // S-curve: shape rotation speed through jerk-limited profiler
            float rawRotSpeed = (abs_diff <= _rotToleranceDeg) ? 0.0f
                : seg.speed_deg_s * speed_scale;
            float profiledRotSpeed;
            // Bypass S-curve in settling band to prevent ramp-down tail wiggle
            if (rawRotSpeed < 0.001f || abs_diff < _rotToleranceDeg * 2.5f) {
                profiledRotSpeed = rawRotSpeed;
                _scurveRot.profiledSpeed = rawRotSpeed;
                _scurveRot.prevProfiledSpeed = rawRotSpeed;
                _scurveRot.currentAccel = 0;
            } else {
                profiledRotSpeed = _scurveRot.update(rawRotSpeed, dt_s);
            }
            _currentOmega = (abs_diff <= _rotToleranceDeg) ? 0.0f
                : rotSign * profiledRotSpeed;
            // Feedforward from rotation S-curve acceleration (only during accel phase)
            float rotAccel = (dt_s > 0.0001f) ?
                (profiledRotSpeed - _scurveRot.prevProfiledSpeed) / dt_s : 0.0f;
            _ffOmega = (rotAccel > 0.0f)
                ? (_ffKvRot * profiledRotSpeed + _ffKaRot * rotAccel)
                : 0.0f;
            _ffVx = 0; _ffVy = 0;
        } else {
            float dx = seg.target_x - current_x;
            float dy = seg.target_y - current_y;
            float dist_remaining = fastLength2(dx, dy);
            // Use blended angle (DR + camera GT via EMA) for heading error.
            // Pure DR angle drifts over time; pure camera angle has 100-deg quantization
            // jumps. Blending gives smooth short-term tracking with long-term convergence.
            float stable_angle = _angleBlendInitialized ? _blendedAngle : current_angle;
            float heading_err = (Rotation(seg.target_angle) - Rotation(stable_angle));
            float abs_heading_err = fabsf(heading_err);
            float speed_scale = 1.0f;

            // Settling band threshold — within this, a gentle min-speed floor applies
            float settlingDist = _waypointToleranceMm * 4.0f;
            bool inSettlingBand = (dist_remaining < settlingDist);

            if (dist_remaining <= _waypointToleranceMm) {
                speed_scale = 0.0f;
            } else if (inSettlingBand) {
                // Inside settling band: linear ramp toward zero.
                // NO precision floor here — the linear ramp naturally drives speed to 0.
                // The floor was keeping speed at 18mm/s even 6mm from target,
                // causing overshoot and the settle-overshoot-adjust oscillation.
                float settleNorm = (dist_remaining - _waypointToleranceMm)
                                 / (settlingDist - _waypointToleranceMm);
                speed_scale = settleNorm * 0.5f;
                // Floor removed: settling band handles final approach without boost.
            } else {
                if (_deccelDistMm > 0.001f && dist_remaining < _deccelDistMm) {
                    float normalized = dist_remaining / _deccelDistMm;
                    speed_scale = fastCbrtf(normalized);
                }

                if (_closeApproachDistMm > 0.001f && dist_remaining < _closeApproachDistMm) {
                    float close_norm = dist_remaining / _closeApproachDistMm;
                    float close_scale = close_norm * close_norm;
                    if (close_scale < speed_scale) speed_scale = close_scale;

                    float heading_window = (_closeRotApproachDeg > _rotToleranceDeg)
                        ? (_closeRotApproachDeg * 2.0f)
                        : (_rotToleranceDeg * 4.0f);
                    if (heading_window > 0.001f) {
                        float heading_scale = 1.0f - (abs_heading_err / heading_window);
                        if (heading_scale < 0.35f) heading_scale = 0.35f;
                        if (heading_scale > 1.0f) heading_scale = 1.0f;
                        speed_scale *= heading_scale;
                    }

                    // Precision min floor only OUTSIDE settling band
                    if (seg.speed_mm_s > 0.001f) {
                        float precision_min_scale = _precisionMinSpeedLimitMmS / seg.speed_mm_s;
                        if (speed_scale < precision_min_scale) speed_scale = precision_min_scale;
                    }
                } else if (seg.speed_mm_s > 0.001f) {
                    float min_scale = _minSpeedLimitMmS / seg.speed_mm_s;
                    if (speed_scale < min_scale) speed_scale = min_scale;
                }

                if (dist_remaining <= (_waypointToleranceMm * 2.0f) &&
                    abs_heading_err > (_rotToleranceDeg * 1.25f)) {
                    speed_scale = 0.0f;
                }
            }

            // Don't decelerate if another translation segment is waiting (seamless chaining)
            if (_count > 1) {
                const MotionSegment &next = _segments[(_head + 1) % MQ_MAX_SEGMENTS];
                if (!next.isDurationBased && (next.vx_mm_s != 0 || next.vy_mm_s != 0)) {
                    speed_scale = 1.0f;
                }
            }

            // --- Predictive Braking ---
            // Use Kalman-estimated velocity to compute closing speed toward target.
            // If the robot is approaching faster than it can kinematically stop
            // in the remaining distance, override speed_scale downward.
            // v_safe = sqrt(2 * decel * dist) — max speed to stop in 'dist'.
            // Never reduce below the settling band floor — the robot needs
            // minimum torque to overcome static friction near the target.
            if (dist_remaining > _waypointToleranceMm && _predictiveBrakeDecel > 0.001f) {
                float inv_dist = 1.0f / dist_remaining;
                // Closing speed = dot(observed_velocity, unit_vector_toward_target)
                float closingSpeed = (_estVx * dx + _estVy * dy) * inv_dist;
                if (closingSpeed > 1.0f) {
                    // Kinematic braking limit (with safety margin)
                    float effectiveDist = dist_remaining / _predictiveBrakeSafety;
                    float vSafe = fastSqrt(2.0f * _predictiveBrakeDecel * effectiveDist);
                    if (closingSpeed > vSafe && seg.speed_mm_s > 0.001f) {
                        float brake_scale = vSafe / seg.speed_mm_s;
                        // Don't clamp below settling band floor
                        float minBrakeScale = _precisionMinSpeedLimitMmS / seg.speed_mm_s;
                        if (brake_scale < minBrakeScale) brake_scale = minBrakeScale;
                        if (brake_scale < speed_scale) {
                            speed_scale = brake_scale;
                        }
                    }
                }
            }

            // Latency-aware speed cap — hard upper bound on commanded speed
            // so the robot can always stop within the remaining distance.
            // This is independent of the predictive braking check above, which
            // uses observed closing speed. This uses the commanded speed and
            // applies regardless of Kalman estimates, making it more robust to
            // stale velocity signals near the target.
            if (dist_remaining > _waypointToleranceMm && _latencyAwareDecelMmS2 > 0.001f) {
                float cmdSpd = fastLength2(_currentVx, _currentVy);
                if (cmdSpd > 0.001f) {
                    float maxSafeSpeed = fastSqrt(2.0f * _latencyAwareDecelMmS2 * dist_remaining);
                    if (cmdSpd > maxSafeSpeed) {
                        float capScale = maxSafeSpeed / cmdSpd;
                        _currentVx *= capScale;
                        _currentVy *= capScale;
                    }
                }
            }

            // --- Adaptive Predictive Steering ---
            // Only use prediction OUTSIDE the close-approach zone.
            // Inside close-approach, prediction causes aim-vector flipping/wiggle.
            float aim_dx = dx;
            float aim_dy = dy;
            if (dist_remaining > _closeApproachDistMm) {
                float currentObsSpeed = fastLength2(_estVx, _estVy);
                float adaptiveLookahead = _computeAdaptiveLookahead(currentObsSpeed);
                if (adaptiveLookahead > 0.001f) {
                    float pred_x = current_x + _estVx * adaptiveLookahead;
                    float pred_y = current_y + _estVy * adaptiveLookahead;
                    aim_dx = seg.target_x - pred_x;
                    aim_dy = seg.target_y - pred_y;
                }
            }
            float aim_dist = fastLength2(aim_dx, aim_dy);
            float aim_inv = (aim_dist > 0.001f) ? (1.0f / aim_dist) : 0.0f;

            // --- S-Curve Profiling: jerk-limited speed ramp ---
            float rawTransSpeed = seg.speed_mm_s * speed_scale;
            float profiledTransSpeed;

            // In settling band (within 4× tolerance) or at zero target:
            // bypass S-curve and use raw speed directly — prevents ramp-down
            // tail from coasting through the target and causing wiggle.
            if (rawTransSpeed < 0.001f || dist_remaining < _waypointToleranceMm * 4.0f) {
                profiledTransSpeed = rawTransSpeed;
                _scurveTrans.profiledSpeed = rawTransSpeed;
                _scurveTrans.prevProfiledSpeed = rawTransSpeed;
                _scurveTrans.currentAccel = 0;
            } else {
                profiledTransSpeed = _scurveTrans.update(rawTransSpeed, dt_s);
            }

            _currentVx = aim_dx * aim_inv * profiledTransSpeed;
            _currentVy = aim_dy * aim_inv * profiledTransSpeed;

            // --- Feedforward from S-curve acceleration ---
            // Only apply feedforward during acceleration (positive accel).
            // During deceleration, FF fights the brake and causes overshoot/orbiting.
            float transAccel = (dt_s > 0.0001f) ?
                (profiledTransSpeed - _scurveTrans.prevProfiledSpeed) / dt_s : 0.0f;
            if (transAccel > 0.0f) {
                _ffVx = aim_dx * aim_inv * (_ffKv * profiledTransSpeed + _ffKa * transAccel);
                _ffVy = aim_dy * aim_inv * (_ffKv * profiledTransSpeed + _ffKa * transAccel);
            } else {
                _ffVx = 0; _ffVy = 0;
            }

            // --- Active Heading Stabilization ---
            // Calculate error between actual heading and segment's target angle.
            // Taper gain DOWN in close-approach to avoid rotational disturbance
            // that causes wiggle during combined translation+rotation maneuvers.
            float stabilization_gain = _rotStabilizationGain;
            if (_closeApproachDistMm > 0.001f && dist_remaining < _closeApproachDistMm) {
                float close_mix = dist_remaining / _closeApproachDistMm;
                if (close_mix < 0.4f) close_mix = 0.4f; // don't go below 40%
                stabilization_gain *= close_mix;
            }

            _currentOmega = heading_err * stabilization_gain;
            // Only apply rotation FF during accel; during decel it causes heading wobble
            _ffOmega = (transAccel > 0.0f) ? (_ffKvRot * fabsf(_currentOmega)) : 0.0f;

            // Clamp stabilization power — taper max omega in close approach
            float maxOmegaClamp = _maxStabilizationOmega;
            if (_closeApproachDistMm > 0.001f && dist_remaining < _closeApproachDistMm) {
                float omegaTaper = 0.4f + 0.6f * (dist_remaining / _closeApproachDistMm);
                maxOmegaClamp *= omegaTaper;
            }
            if (_currentOmega > maxOmegaClamp) _currentOmega = maxOmegaClamp;
            if (_currentOmega < -maxOmegaClamp) _currentOmega = -maxOmegaClamp;

            // Anti-stall: if translation speed is non-zero but below the precision
            // minimum, boost it so motors can overcome static friction.
            // DISABLED inside settling band and close-approach — near the target we WANT low speed.
            // Prevents the feedback loop: closeApproach makes speed_scale tiny, boost amplifies
            // it back up, robot overshoots, drops out of closeApproach, speed_scale drops again...
            if (!inSettlingBand && dist_remaining >= _closeApproachDistMm) {
                float cmdSpd = fastLength2(_currentVx, _currentVy);
                if (cmdSpd > 0.001f && cmdSpd < _precisionMinSpeedLimitMmS) {
                    float boost = _precisionMinSpeedLimitMmS / cmdSpd;
                    _currentVx *= boost;
                    _currentVy *= boost;
                }
            }
        }
    }

    // --- EMA-Smoothed Commanded Velocity for DR Integration ---
    // _currentVx/Vy drops to 0 instantly when speed_scale→0. If fed directly
    // to dead-reckoning, DR freezes while the robot coasts, creating a "coast gap"
    // that manifests as consistent overshoot past the target.
    //
    // Fix: EMA-smoothed velocity decays naturally from _currentVx/Vy toward 0.
    // alpha=0.9 gives tau=10ms (≈30 ticks to settle at 1kHz), matching the
    // physical coast time of the robot after power is cut. DR integrates
    // through this period, so its position estimate matches reality.
    // Kalman velocity is still used for predictive steering/braking.
    const float EMA_ALPHA = 0.9f;
    _emaVx = EMA_ALPHA * _emaVx + (1.0f - EMA_ALPHA) * _currentVx;
    _emaVy = EMA_ALPHA * _emaVy + (1.0f - EMA_ALPHA) * _currentVy;

    // --- SLIP / STALL DETECTION ---
    // Compare commanded speed vs Kalman-observed speed.
    // Only check when NOT in settling band — near the target, low speed is expected.
    bool isTranslation = !(seg.vx_mm_s == 0 && seg.vy_mm_s == 0 && seg.omega_deg_s != 0);
    // Use squared distance to avoid redundant sqrtf — compare vs squared threshold
    float slipDistSq = isTranslation
        ? (seg.target_x - current_x) * (seg.target_x - current_x)
          + (seg.target_y - current_y) * (seg.target_y - current_y)
        : 1e6f;
    float slipDistCheck = fastSqrt(slipDistSq);
    // Disable slip detection when very close to target (< 5× tolerance).
    // At close range, low commanded speed naturally produces low observed speed,
    // which triggers false-positive slip detection and corrupts the velocity estimate.
    bool nearTarget = slipDistCheck < _waypointToleranceMm * 5.0f;
    if (!nearTarget && !seg.isDurationBased && slipDistCheck > _waypointToleranceMm * 4.0f) {
        // Compare squared speeds against squared thresholds — avoids sqrt entirely
        float cmdSpeedSq = _currentVx * _currentVx + _currentVy * _currentVy;
        float obsSpeedSq = _estVx * _estVx + _estVy * _estVy;
        float obsSpeed = fastSqrt(obsSpeedSq);
        // cmdSpeed only used in debug logging — use fastSqrt there
        float cmdSpeed = fastSqrt(cmdSpeedSq);

        if (cmdSpeedSq > _slipCmdThresh * _slipCmdThresh && obsSpeedSq < _slipObsThresh * _slipObsThresh) {
            _slipCounter++;
        } else {
            _slipCounter = 0;
            if (_slipBoostRemaining > 0) _slipBoostRemaining--;
        }

        if (_slipCounter >= _slipDetectTicks && _slipBoostRemaining == 0) {
            // Stall confirmed — start boost phase
            _slipBoostRemaining = _slipBoostMaxTicks;
            _slipDetected = true;
#if ENABLE_DEBUG_LOGGING
            Serial.printf("[MQ] Slip detected! cmd=%.1f obs=%.1f — boosting\n", cmdSpeed, obsSpeed);
#endif
        }

        if (_slipBoostRemaining > 0) {
            _currentVx *= _slipBoostFactor;
            _currentVy *= _slipBoostFactor;
            _slipBoostRemaining--;
            if (_slipBoostRemaining == 0) {
                _slipDetected = false;
                _slipCounter = 0;
            }
        }
    }

    // --- CHECK COMPLETION ---
    bool done = false;
    if (seg.isDurationBased) {
        done = (seg.elapsed_ms >= seg.duration_ms);
    } else {
        if (seg.vx_mm_s == 0 && seg.vy_mm_s == 0 && seg.omega_deg_s != 0) {
            float angle_diff = (Rotation(current_angle) - Rotation(seg.target_angle));
            if (fabsf(angle_diff) <= _rotToleranceDeg) done = true;
        } else {
            float dx = seg.target_x - current_x;
            float dy = seg.target_y - current_y;
            float dist_remaining = fastLength2(dx, dy);

            // --- PREDICTIVE COMPLETION CHECK ---
            float heading_err = (Rotation(seg.target_angle) - Rotation(current_angle));

            float obsSpd = fastLength2(_estVx, _estVy);
            
            // Compute stopping distance: d = v^2/(2*a)
            float stoppingDist = 0.0f;
            if (obsSpd > 0.001f && _predictiveBrakeDecel > 0.001f) {
                stoppingDist = (obsSpd * obsSpd) / (2.0f * _predictiveBrakeDecel);
                // Apply safety margin to ensure we stop WITHIN tolerance
                stoppingDist *= _predictiveBrakeSafety;
            }
            
            // Calculate margin to tolerance boundary
            float marginToTolerance = dist_remaining - _waypointToleranceMm;
            
            // Three-way completion gate:
            // 1. Position within tolerance
            // 2. Heading within tolerance  
            // 3. Either nearly stopped OR stopping distance less than margin
            bool atPositionTolerance = (dist_remaining <= _waypointToleranceMm);
            bool atHeadingTolerance = (fabsf(heading_err) <= _rotToleranceDeg);
            bool willStopInTolerance = (obsSpd < _precisionMinSpeedLimitMmS * 1.5f) ||
                                       (stoppingDist <= marginToTolerance + _waypointToleranceMm);
            
            if (atPositionTolerance && atHeadingTolerance && willStopInTolerance) {
                done = true;
            }
        }
    }

    if (done) {
        if (seg.state == SegmentState::ACTIVE) {
#if ENABLE_DEBUG_LOGGING
            Serial.printf("[MQ Debug] Segment %d done. Final error: %.1f mm\n", _head % MQ_MAX_SEGMENTS, seg.traveled_mm);
#endif
            seg.state = SegmentState::COMPLETED;
            segmentJustCompleted = true;

            if (_count > 1) {
                // Have more segments? Immediate advance.
                _advanceToNext();
                return true;
            } else {
                // Last segment? Switch to HOLDING to maintain position
                seg.state = SegmentState::HOLDING;
            }
        }
    }

    // In HOLDING state, we already calculated _currentVx/Vy/Omega based on the error above.
    // If we are within tolerance in HOLDING state, just stop motors.
    if (seg.state == SegmentState::HOLDING) {
        bool withinTol = false;
        if (seg.vx_mm_s == 0 && seg.vy_mm_s == 0 && seg.omega_deg_s != 0) {
            float angle_diff = (Rotation(current_angle) - Rotation(seg.target_angle));
            withinTol = (fabsf(angle_diff) <= _rotToleranceDeg);
        } else {
            float dx = seg.target_x - current_x;
            float dy = seg.target_y - current_y;
            float heading_err = (Rotation(seg.target_angle) - Rotation(current_angle));
            withinTol = (fastLength2(dx, dy) <= _waypointToleranceMm &&
                         fabsf(heading_err) <= _rotToleranceDeg);
        }

        if (withinTol) {
            _currentVx = _currentVy = _currentOmega = 0;
        }
    }

    // #region agent_debug_log H2+H4: Kalman covariance trace and profiledSpeed sanity
    {
        float kalmanTraceX = _kalmanX.p00 + _kalmanX.p11;
        float kalmanTraceY = _kalmanY.p00 + _kalmanY.p11;
        const char* stateName = "IDLE";
        if (_count > 0) {
            const MotionSegment& s = _segments[_head % MQ_MAX_SEGMENTS];
            switch (s.state) {
                case SegmentState::PENDING:    stateName = "PENDING";   break;
                case SegmentState::ACTIVE:     stateName = "ACTIVE";    break;
                case SegmentState::HOLDING:    stateName = "HOLDING";   break;
                case SegmentState::COMPLETED:   stateName = "COMPLETED"; break;
            }
        }
        _dbgLogMQDiag(
            kalmanTraceX, kalmanTraceY,
            _scurveTrans.profiledSpeed, _scurveRot.profiledSpeed,
            _tickTimeMs, _count, stateName);
    }
    // #endregion

    return true;
}

// ============================================================================

void MotionQueue::_advanceToNext() {
    _head = (_head + 1) % MQ_MAX_SEGMENTS;
    _count--;

    if (_count > 0) {
        MotionSegment &next = _segments[_head % MQ_MAX_SEGMENTS];
        next.state = SegmentState::ACTIVE;
        next.traveled_mm = 0;
        next.elapsed_ms = 0;
        // Reset velocity history for this new segment
        _hasPreviousPose = false;
    }
}

// ============================================================================

void MotionQueue::getCurrentVelocity(float &vx_mm_s, float &vy_mm_s, float &omega_deg_s) const {
    vx_mm_s = _currentVx;
    vy_mm_s = _currentVy;
    omega_deg_s = _currentOmega;
}

CorrectionPolicy MotionQueue::getActivePolicy() const {
    if (_count == 0) return CorrectionPolicy::NONE;
    const MotionSegment &seg = _segments[_head % MQ_MAX_SEGMENTS];
    return seg.correctionPolicy;
}

void MotionQueue::storeDeferredCorrection(float errX, float errY, float errAngle) {
    if (_count == 0) return;
    MotionSegment &seg = _segments[_head % MQ_MAX_SEGMENTS];
    seg.deferred_correction_x += errX;
    seg.deferred_correction_y += errY;
    seg.deferred_correction_angle += errAngle;
}

void MotionQueue::getDeferredCorrection(float &errX, float &errY, float &errAngle) const {
    // Return correction from the segment that just completed
    // (which was at _head - 1 before advance)
    int prevIdx = (_head - 1 + MQ_MAX_SEGMENTS) % MQ_MAX_SEGMENTS;
    errX = _segments[prevIdx].deferred_correction_x;
    errY = _segments[prevIdx].deferred_correction_y;
    errAngle = _segments[prevIdx].deferred_correction_angle;
}

ServoAction MotionQueue::getLastCompletedServoAction() const {
    int prevIdx = (_head - 1 + MQ_MAX_SEGMENTS) % MQ_MAX_SEGMENTS;
    return _segments[prevIdx].servoAction;
}

int MotionQueue::remaining() const {
    return _count;
}

bool MotionQueue::isEmpty() const {
    return _count == 0;
}

void MotionQueue::abort() {
    _head  = 0;
    _tail  = 0;
    _count = 0;
    segmentJustCompleted = false;
    // Reset velocity tracking — stale estimates must not corrupt next segment
    _hasPreviousPose = false;
    _estVx = 0; _estVy = 0;
    _emaVx = 0; _emaVy = 0;
    // Reset Kalman filter so next segment starts fresh
    _kalmanInitialized = false;
    // Reset S-curve profilers
    _scurveTrans.reset(_scurveMaxAccelMmS2, _scurveMaxJerkMmS3);
    _scurveRot.reset(_scurveMaxRotAccelDegS2, _scurveMaxRotJerkDegS3);
    _ffVx = _ffVy = _ffOmega = 0;
    _slipCounter = 0;
    _slipBoostRemaining = 0;
    _slipDetected = false;
    // Reset angle blend — fresh camera observations needed after abort
    _angleBlendInitialized = false;
    _blendedAngle = 0.0f;
}

// ============================================================================
// DEBUG: Wiggle diagnostics — trace speed_scale components near target
// ============================================================================
#if 1  // Toggle to 0 to disable
void MotionQueue::debugLogWiggle(float current_x, float current_y, float current_angle) {
    if (_count == 0) return;
    static unsigned long _lastWiggleLogMs = 0;
    if (millis() - _lastWiggleLogMs < 100) return;
    _lastWiggleLogMs = millis();

    MotionSegment &seg = _segments[_head % MQ_MAX_SEGMENTS];
    if (seg.state != SegmentState::ACTIVE && seg.state != SegmentState::HOLDING) return;
    if (seg.vx_mm_s == 0 && seg.vy_mm_s == 0 && seg.omega_deg_s != 0) return;

    float dx = seg.target_x - current_x;
    float dy = seg.target_y - current_y;
    float dist_remaining = fastLength2(dx, dy);
    if (dist_remaining > 250.0f) return;  // only log near target

    float heading_err = (Rotation(seg.target_angle) - Rotation(current_angle));
    float speed_scale = 1.0f;
    float settlingDist = _waypointToleranceMm * 4.0f;
    bool inSettlingBand = (dist_remaining < settlingDist);

    if (dist_remaining <= _waypointToleranceMm) {
        speed_scale = 0.0f;
    } else if (inSettlingBand) {
        float settleNorm = (dist_remaining - _waypointToleranceMm) / (settlingDist - _waypointToleranceMm);
        speed_scale = settleNorm * 0.5f;
        if (seg.speed_mm_s > 0.001f) {
            float floor = _precisionMinSpeedLimitMmS / seg.speed_mm_s;
            if (speed_scale < floor) speed_scale = floor;
        }
    } else {
        if (_deccelDistMm > 0.001f && dist_remaining < _deccelDistMm) {
            speed_scale = fastCbrtf(dist_remaining / _deccelDistMm);
        }
        if (_closeApproachDistMm > 0.001f && dist_remaining < _closeApproachDistMm) {
            float close_norm = dist_remaining / _closeApproachDistMm;
            float close_scale = close_norm * close_norm;
            if (close_scale < speed_scale) speed_scale = close_scale;
            float heading_window = (_closeRotApproachDeg > _rotToleranceDeg)
                ? (_closeRotApproachDeg * 2.0f) : (_rotToleranceDeg * 4.0f);
            if (heading_window > 0.001f) {
                float absErr = fabsf(heading_err);
                float h_scale = 1.0f - (absErr / heading_window);
                if (h_scale < 0.35f) h_scale = 0.35f;
                if (h_scale > 1.0f) h_scale = 1.0f;
                speed_scale *= h_scale;
            }
            if (seg.speed_mm_s > 0.001f) {
                float pmin = _precisionMinSpeedLimitMmS / seg.speed_mm_s;
                if (speed_scale < pmin) speed_scale = pmin;
            }
        } else if (seg.speed_mm_s > 0.001f) {
            float min_s = _minSpeedLimitMmS / seg.speed_mm_s;
            if (speed_scale < min_s) speed_scale = min_s;
        }
    }

    float cmdSpd = fastLength2(_currentVx, _currentVy);
    Serial.printf("[WIG] dist=%.1f ss=%.3f cmdVx=%.2f cmdVy=%.2f cmdSpd=%.2f close=%.0f settle=%d antiStall=%d\n",
        dist_remaining, speed_scale, _currentVx, _currentVy, cmdSpd,
        _closeApproachDistMm, inSettlingBand,
        (!inSettlingBand && dist_remaining >= _closeApproachDistMm && cmdSpd > 0.001f && cmdSpd < _precisionMinSpeedLimitMmS) ? 1 : 0);
}
#endif

const MotionSegment* MotionQueue::currentSegment() const {
    if (_count == 0) return nullptr;
    return &_segments[_head % MQ_MAX_SEGMENTS];
}
