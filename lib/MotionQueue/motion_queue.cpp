#include "motion_queue.hpp"

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
      _waypointToleranceMm(12.0f), _rotToleranceDeg(2.0f),
      _rotStabilizationGain(1.5f), _maxStabilizationOmega(35.0f),
      segmentJustCompleted(false)
{
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
                                          float waypoint_tol_mm, float rot_tol_deg,
                                          float rot_stab_gain, float max_stab_omega) {
    _deccelDistMm = deccel_dist_mm;
    _rotDeccelDeg = rot_deccel_deg;
    _minSpeedLimitMmS = min_speed_mm_s;
    _minRotLimitDegS = min_rot_deg_s;
    _waypointToleranceMm = waypoint_tol_mm;
    _rotToleranceDeg = rot_tol_deg;
    _rotStabilizationGain = rot_stab_gain;
    _maxStabilizationOmega = max_stab_omega;
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

bool MotionQueue::enqueueWaypoint(float target_x, float target_y,
                                  SpeedLevel speed, CorrectionPolicy policy,
                                  float currentX, float currentY, float currentAngle) {
    if (_count >= MQ_MAX_SEGMENTS) return false;

    MotionSegment &seg = _segments[_tail % MQ_MAX_SEGMENTS];
    seg.direction       = MoveDirection::INVALID;  // Not a discrete direction
    seg.speed           = speed;
    seg.correctionPolicy = policy;
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

    seg.target_angle = seg.start_angle;
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
// ============================================================================

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

bool MotionQueue::enqueue(MoveDirection direction, uint16_t distance_mm,
                          SpeedLevel speed, CorrectionPolicy policy,
                          float currentX, float currentY, float currentAngle) {
    if (_count >= MQ_MAX_SEGMENTS) return false;

    MotionSegment &seg = _segments[_tail % MQ_MAX_SEGMENTS];
    seg.direction       = direction;
    seg.distance_mm     = distance_mm;
    seg.speed           = speed;
    seg.correctionPolicy = policy;
    seg.state           = SegmentState::PENDING;
    seg.traveled_mm     = 0;
    seg.isDurationBased = false;
    seg.deferred_correction_x = 0;
    seg.deferred_correction_y = 0;

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
    }

    if (seg.state != SegmentState::ACTIVE && seg.state != SegmentState::HOLDING) {
        _currentVx = _currentVy = _currentOmega = 0;
        return false;
    }

    // Integrate distance traveled
    float dt_s = dt_ms / 1000.0f;
    
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
            float abs_diff = abs(angle_diff);

            // Calculate deceleration scale
            if (abs_diff < _rotDeccelDeg) {
                speed_scale = abs_diff / _rotDeccelDeg;
                // Clamp to minimum speed to avoid stall
                float min_scale = _minRotLimitDegS / seg.speed_deg_s;
                if (speed_scale < min_scale) speed_scale = min_scale;
            }

            // Only decelerate if this is the last rotation or next isn't also a rotation
            if (_count > 1) speed_scale = 1.0f; 

            _currentVx = 0;
            _currentVy = 0;
            _currentOmega = (angle_diff > 0 ? seg.speed_deg_s : -seg.speed_deg_s) * speed_scale;
        } else {
            float dx = seg.target_x - current_x;
            float dy = seg.target_y - current_y;
            float dist_remaining = sqrtf(dx * dx + dy * dy);

            // Calculate deceleration scale
            if (dist_remaining < _deccelDistMm) {
                speed_scale = dist_remaining / _deccelDistMm;
                float min_scale = _minSpeedLimitMmS / seg.speed_mm_s;
                if (speed_scale < min_scale) speed_scale = min_scale;
            }

            // Don't decelerate if another translation segment is waiting (seamless chaining)
            if (_count > 1) {
                const MotionSegment &next = _segments[(_head + 1) % MQ_MAX_SEGMENTS];
                if (!next.isDurationBased && (next.vx_mm_s != 0 || next.vy_mm_s != 0)) {
                    speed_scale = 1.0f;
                }
            }

            _currentVx = (dx / dist_remaining) * seg.speed_mm_s * speed_scale;
            _currentVy = (dy / dist_remaining) * seg.speed_mm_s * speed_scale;
            
            // --- Active Heading Stabilization ---
            // Calculate error between actual heading and segment's target angle
            float heading_err = seg.target_angle - current_angle;
            while (heading_err > 180.0f) heading_err -= 360.0f;
            while (heading_err < -180.0f) heading_err += 360.0f;

            // Apply proportional stabilization
            _currentOmega = heading_err * _rotStabilizationGain;

            // Clamp stabilization power to prevent violent shakes
            if (_currentOmega > _maxStabilizationOmega) _currentOmega = _maxStabilizationOmega;
            if (_currentOmega < -_maxStabilizationOmega) _currentOmega = -_maxStabilizationOmega;
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
            if (abs(angle_diff) <= _rotToleranceDeg) done = true;
        } else {
            float dx = seg.target_x - current_x;
            float dy = seg.target_y - current_y;
            float dist_remaining = sqrtf(dx * dx + dy * dy);
            
            // Reached boundary tolerance, or drifted past target
            float dot_product = (dx * seg.vx_mm_s) + (dy * seg.vy_mm_s);
            if (dist_remaining <= _waypointToleranceMm || dot_product < 0) done = true;
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
            withinTol = (abs(angle_diff) <= _rotToleranceDeg);
        } else {
            float dx = seg.target_x - current_x;
            float dy = seg.target_y - current_y;
            withinTol = (sqrtf(dx * dx + dy * dy) <= _waypointToleranceMm);
        }

        if (withinTol) {
            _currentVx = _currentVy = _currentOmega = 0;
        }
    }

    return true;
}

void MotionQueue::_advanceToNext() {
    _head = (_head + 1) % MQ_MAX_SEGMENTS;
    _count--;

    // Immediately activate the next segment if available (seamless transition)
    if (_count > 0) {
        MotionSegment &next = _segments[_head % MQ_MAX_SEGMENTS];
        next.state = SegmentState::ACTIVE;
        next.traveled_mm = 0;
    }
}

// ============================================================================
// Accessors
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
}

const MotionSegment* MotionQueue::currentSegment() const {
    if (_count == 0) return nullptr;
    return &_segments[_head % MQ_MAX_SEGMENTS];
}
