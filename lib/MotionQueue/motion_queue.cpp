#include "motion_queue.hpp"

// ============================================================================
// Construction
// ============================================================================

MotionQueue::MotionQueue()
    : _head(0), _tail(0), _count(0),
      _speedSlow(100.0f), _speedNormal(250.0f), _speedFast(450.0f),
      segmentJustCompleted(false)
{
}

void MotionQueue::setSpeedCalibration(float slow_mm_s, float normal_mm_s, float fast_mm_s) {
    _speedSlow   = slow_mm_s;
    _speedNormal = normal_mm_s;
    _speedFast   = fast_mm_s;
}

float MotionQueue::_getSpeedMmS(SpeedLevel level) const {
    switch (level) {
        case SpeedLevel::SLOW:   return _speedSlow;
        case SpeedLevel::NORMAL: return _speedNormal;
        case SpeedLevel::FAST:   return _speedFast;
        default:                 return _speedNormal;
    }
}

// ============================================================================
// Enqueue a new motion segment
// ============================================================================

bool MotionQueue::enqueue(MoveDirection direction, uint16_t distance_mm,
                          SpeedLevel speed, CorrectionPolicy policy,
                          float currentX, float currentY) {
    if (_count >= MQ_MAX_SEGMENTS) return false;

    MotionSegment &seg = _segments[_tail % MQ_MAX_SEGMENTS];
    seg.direction       = direction;
    seg.distance_mm     = distance_mm;
    seg.speed           = speed;
    seg.correctionPolicy = policy;
    seg.state           = SegmentState::PENDING;
    seg.traveled_mm     = 0;
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
    // segment starts.  For the first segment, that's currentX/Y.
    // For subsequent segments, chain from previous target.
    if (_count == 0) {
        seg.start_x = currentX;
        seg.start_y = currentY;
    } else {
        // Chain from the previous segment's target
        const MotionSegment &prev = _segments[(_tail - 1 + MQ_MAX_SEGMENTS) % MQ_MAX_SEGMENTS];
        seg.start_x = prev.target_x;
        seg.start_y = prev.target_y;
    }
    seg.target_x = seg.start_x + dirX * distance_mm;
    seg.target_y = seg.start_y + dirY * distance_mm;

    _tail = (_tail + 1) % MQ_MAX_SEGMENTS;
    _count++;
    return true;
}

// ============================================================================
// Tick — called every control-loop iteration
// ============================================================================

bool MotionQueue::tick(uint32_t dt_ms) {
    segmentJustCompleted = false;

    if (_count == 0) return false;

    MotionSegment &seg = _segments[_head % MQ_MAX_SEGMENTS];

    // Activate pending segment
    if (seg.state == SegmentState::PENDING) {
        seg.state = SegmentState::ACTIVE;
        seg.traveled_mm = 0;
    }

    if (seg.state != SegmentState::ACTIVE) return false;

    // Integrate distance traveled
    float dt_s = dt_ms / 1000.0f;
    float displacement = seg.speed_mm_s * dt_s;
    seg.traveled_mm += displacement;

    // Check if segment is complete
    if (seg.traveled_mm >= (float)seg.distance_mm) {
        seg.state = SegmentState::COMPLETED;
        segmentJustCompleted = true;
        _advanceToNext();
        return (_count > 0);  // true if there's a next segment
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

void MotionQueue::getCurrentVelocity(float &vx_mm_s, float &vy_mm_s) const {
    if (_count == 0) {
        vx_mm_s = 0;
        vy_mm_s = 0;
        return;
    }
    const MotionSegment &seg = _segments[_head % MQ_MAX_SEGMENTS];
    if (seg.state == SegmentState::ACTIVE) {
        vx_mm_s = seg.vx_mm_s;
        vy_mm_s = seg.vy_mm_s;
    } else {
        vx_mm_s = 0;
        vy_mm_s = 0;
    }
}

CorrectionPolicy MotionQueue::getActivePolicy() const {
    if (_count == 0) return CorrectionPolicy::NONE;
    const MotionSegment &seg = _segments[_head % MQ_MAX_SEGMENTS];
    return seg.correctionPolicy;
}

void MotionQueue::storeDeferredCorrection(float errX, float errY) {
    if (_count == 0) return;
    MotionSegment &seg = _segments[_head % MQ_MAX_SEGMENTS];
    seg.deferred_correction_x += errX;
    seg.deferred_correction_y += errY;
}

void MotionQueue::getDeferredCorrection(float &errX, float &errY) const {
    // Return correction from the segment that just completed
    // (which was at _head - 1 before advance)
    int prevIdx = (_head - 1 + MQ_MAX_SEGMENTS) % MQ_MAX_SEGMENTS;
    errX = _segments[prevIdx].deferred_correction_x;
    errY = _segments[prevIdx].deferred_correction_y;
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
