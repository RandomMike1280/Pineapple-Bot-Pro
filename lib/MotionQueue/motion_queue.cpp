#include "motion_queue.hpp"
#include <math.h>

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
      _rotStabilizationGain(1.5f), _maxStabilizationOmega(35.0f),
      _prevPoseX(0), _prevPoseY(0), _hasPreviousPose(false),
      _estVx(0), _estVy(0), _lookaheadTimeS(0.15f), _tickTimeMs(0),
      _scurveMaxAccelMmS2(250.0f), _scurveMaxJerkMmS3(1200.0f),
      _scurveMaxRotAccelDegS2(180.0f), _scurveMaxRotJerkDegS3(900.0f),
      _ffKv(0.02f), _ffKa(0.04f), _ffKvRot(0.01f), _ffKaRot(0.02f),
      _ffVx(0), _ffVy(0), _ffOmega(0),
      _adaptLookaheadBaseS(0.06f), _adaptLookaheadGain(0.0022f),
      _adaptLookaheadMinS(0.04f), _adaptLookaheadMaxS(0.25f),
      _kalmanInitialized(false),
      _predictiveBrakeDecel(200.0f), _predictiveBrakeSafety(1.5f),
      _slipCmdThresh(15.0f), _slipObsThresh(5.0f), _slipDetectTicks(25),
      _slipBoostFactor(1.35f), _slipBoostMaxTicks(40),
      _slipCounter(0), _slipBoostRemaining(0), _slipDetected(false),
      segmentJustCompleted(false)
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

void MotionQueue::setPredictiveBraking(float decel_mm_s2, float safety_factor) {
    _predictiveBrakeDecel = decel_mm_s2;
    _predictiveBrakeSafety = safety_factor;
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
    float jumpDist = sqrtf((x - _prevPoseX) * (x - _prevPoseX) +
                           (y - _prevPoseY) * (y - _prevPoseY));
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
    _kalmanX.predict(dt_s);
    _kalmanY.predict(dt_s);

    // Kalman correct step (fuse position measurement)
    _kalmanX.correct(x);
    _kalmanY.correct(y);

    // Stationary deadzone: kill velocity estimate noise when nearly stopped
    float est_speed = sqrtf(_kalmanX.v * _kalmanX.v + _kalmanY.v * _kalmanY.v);
    const float deadzoneMmS = 3.0f;
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
    float dist = sqrtf(dx * dx + dy * dy);
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

    float d_angle = target_angle - seg.start_angle;
    while (d_angle > 180.0f) d_angle -= 360.0f;
    while (d_angle < -180.0f) d_angle += 360.0f;

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
    seg.target_angle = seg.start_angle + estAngleDelta;
    while (seg.target_angle >= 360.0f) seg.target_angle -= 360.0f;
    while (seg.target_angle < 0.0f) seg.target_angle += 360.0f;

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
    seg.speed_mm_s  = sqrtf(vx_mm_s * vx_mm_s + vy_mm_s * vy_mm_s);
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

bool MotionQueue::tick(uint32_t dt_ms, float current_x, float current_y, float current_angle) {
    segmentJustCompleted = false;
    _tickTimeMs += dt_ms;
    float dt_s = dt_ms / 1000.0f;

    // Update velocity estimate from observed position every tick
    _updateVelocityEstimate(current_x, current_y, dt_s);

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
        displacement = sqrtf(phys_vx * phys_vx + phys_vy * phys_vy) * dt_s;
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
            float angle_diff = seg.target_angle - current_angle;
            while (angle_diff > 180.0f) angle_diff -= 360.0f;
            while (angle_diff < -180.0f) angle_diff += 360.0f;
            float abs_diff = fabsf(angle_diff);
            speed_scale = 1.0f;

            if (abs_diff <= _rotToleranceDeg) {
                speed_scale = 0.0f;
            } else {
                if (_rotDeccelDeg > 0.001f && abs_diff < _rotDeccelDeg) {
                    float normalized = abs_diff / _rotDeccelDeg;
                    speed_scale = cbrtf(normalized);
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
            float rotSign = (angle_diff >= 0.0f) ? 1.0f : -1.0f;
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
            float dist_remaining = sqrtf(dx * dx + dy * dy);
            float heading_err = seg.target_angle - current_angle;
            while (heading_err > 180.0f) heading_err -= 360.0f;
            while (heading_err < -180.0f) heading_err += 360.0f;
            float abs_heading_err = fabsf(heading_err);
            float speed_scale = 1.0f;

            // Settling band threshold — within this, a gentle min-speed floor applies
            float settlingDist = _waypointToleranceMm * 4.0f;
            bool inSettlingBand = (dist_remaining < settlingDist);

            if (dist_remaining <= _waypointToleranceMm) {
                speed_scale = 0.0f;
            } else if (inSettlingBand) {
                // Inside settling band: linear ramp toward zero with a minimum
                // floor so the motors always have enough torque to actually move.
                // Without the floor, the robot stalls a few mm from the target
                // and requires a manual nudge to reach tolerance.
                float settleNorm = (dist_remaining - _waypointToleranceMm)
                                 / (settlingDist - _waypointToleranceMm);
                speed_scale = settleNorm * 0.5f;
                // Floor: precision min speed keeps motors turning (especially for strafe)
                if (seg.speed_mm_s > 0.001f) {
                    float settleFloor = _precisionMinSpeedLimitMmS / seg.speed_mm_s;
                    if (speed_scale < settleFloor) speed_scale = settleFloor;
                }
            } else {
                if (_deccelDistMm > 0.001f && dist_remaining < _deccelDistMm) {
                    float normalized = dist_remaining / _deccelDistMm;
                    speed_scale = cbrtf(normalized);
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
                    float vSafe = sqrtf(2.0f * _predictiveBrakeDecel * effectiveDist);
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

            // --- Adaptive Predictive Steering ---
            // Only use prediction OUTSIDE the close-approach zone.
            // Inside close-approach, prediction causes aim-vector flipping/wiggle.
            float aim_dx = dx;
            float aim_dy = dy;
            if (dist_remaining > _closeApproachDistMm) {
                float currentObsSpeed = sqrtf(_estVx * _estVx + _estVy * _estVy);
                float adaptiveLookahead = _computeAdaptiveLookahead(currentObsSpeed);
                if (adaptiveLookahead > 0.001f) {
                    float pred_x = current_x + _estVx * adaptiveLookahead;
                    float pred_y = current_y + _estVy * adaptiveLookahead;
                    aim_dx = seg.target_x - pred_x;
                    aim_dy = seg.target_y - pred_y;
                }
            }
            float aim_dist = sqrtf(aim_dx * aim_dx + aim_dy * aim_dy);
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
            // DISABLED inside settling band — near the target we WANT low speed.
            if (!inSettlingBand) {
                float cmdSpd = sqrtf(_currentVx * _currentVx + _currentVy * _currentVy);
                if (cmdSpd > 0.001f && cmdSpd < _precisionMinSpeedLimitMmS) {
                    float boost = _precisionMinSpeedLimitMmS / cmdSpd;
                    _currentVx *= boost;
                    _currentVy *= boost;
                }
            }
        }
    }

    // --- SLIP / STALL DETECTION ---
    // Compare commanded speed vs Kalman-observed speed.
    // Only check when NOT in settling band — near the target, low speed is expected.
    bool isTranslation = !(seg.vx_mm_s == 0 && seg.vy_mm_s == 0 && seg.omega_deg_s != 0);
    float slipDistCheck = isTranslation ? sqrtf(
        (seg.target_x - current_x) * (seg.target_x - current_x) +
        (seg.target_y - current_y) * (seg.target_y - current_y)) : 999.0f;
    if (!seg.isDurationBased && slipDistCheck > _waypointToleranceMm * 4.0f) {
        float cmdSpeed = sqrtf(_currentVx * _currentVx + _currentVy * _currentVy);
        float obsSpeed = sqrtf(_estVx * _estVx + _estVy * _estVy);

        if (cmdSpeed > _slipCmdThresh && obsSpeed < _slipObsThresh) {
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
            float angle_diff = current_angle - seg.target_angle;
            while (angle_diff > 180.0f) angle_diff -= 360.0f;
            while (angle_diff < -180.0f) angle_diff += 360.0f;
            if (fabsf(angle_diff) <= _rotToleranceDeg) done = true;
        } else {
            float dx = seg.target_x - current_x;
            float dy = seg.target_y - current_y;
            float dist_remaining = sqrtf(dx * dx + dy * dy);
            
            // Simultaneous completion check: position + orientation + velocity gate.
            // The velocity gate prevents declaring "done" while the robot is still
            // moving fast through the tolerance zone (momentum overshoot).
            float heading_err = seg.target_angle - current_angle;
            while (heading_err > 180.0f) heading_err -= 360.0f;
            while (heading_err < -180.0f) heading_err += 360.0f;

            float obsSpd = sqrtf(_estVx * _estVx + _estVy * _estVy);
            if (dist_remaining <= _waypointToleranceMm &&
                fabsf(heading_err) <= _rotToleranceDeg &&
                obsSpd < _precisionMinSpeedLimitMmS * 1.5f) {
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
            float angle_diff = current_angle - seg.target_angle;
            while (angle_diff > 180.0f) angle_diff -= 360.0f;
            while (angle_diff < -180.0f) angle_diff += 360.0f;
            withinTol = (fabsf(angle_diff) <= _rotToleranceDeg);
        } else {
            float dx = seg.target_x - current_x;
            float dy = seg.target_y - current_y;
            float heading_err = seg.target_angle - current_angle;
            while (heading_err > 180.0f) heading_err -= 360.0f;
            while (heading_err < -180.0f) heading_err += 360.0f;
            withinTol = (sqrtf(dx * dx + dy * dy) <= _waypointToleranceMm && 
                         fabsf(heading_err) <= _rotToleranceDeg);
        }

        if (withinTol) {
            _currentVx = _currentVy = _currentOmega = 0;
        }
    }

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
    _estVx = 0;
    _estVy = 0;
    // Reset Kalman filter so next segment starts fresh
    _kalmanInitialized = false;
    // Reset S-curve profilers
    _scurveTrans.reset(_scurveMaxAccelMmS2, _scurveMaxJerkMmS3);
    _scurveRot.reset(_scurveMaxRotAccelDegS2, _scurveMaxRotJerkDegS3);
    _ffVx = _ffVy = _ffOmega = 0;
    _slipCounter = 0;
    _slipBoostRemaining = 0;
    _slipDetected = false;
}

const MotionSegment* MotionQueue::currentSegment() const {
    if (_count == 0) return nullptr;
    return &_segments[_head % MQ_MAX_SEGMENTS];
}
