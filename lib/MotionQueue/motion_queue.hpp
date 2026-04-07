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
    COMPLETED
};

struct MotionSegment {
    MoveDirection    direction;
    uint16_t         distance_mm;
    SpeedLevel       speed;
    CorrectionPolicy correctionPolicy;

    // Computed at enqueue time
    float target_x;      // absolute target position (mm)
    float target_y;
    float start_x;       // position at segment start
    float start_y;
    float vx_mm_s;       // velocity vector (mm/s)
    float vy_mm_s;
    float speed_mm_s;    // scalar speed (mm/s)

    // Runtime state
    SegmentState state;
    float        traveled_mm;   // distance traveled in this segment so far
    float        deferred_correction_x;  // accumulated drift for deferred policy
    float        deferred_correction_y;

    MotionSegment() : direction(MoveDirection::INVALID), distance_mm(0),
                      speed(SpeedLevel::NORMAL), correctionPolicy(CorrectionPolicy::LIVE),
                      target_x(0), target_y(0), start_x(0), start_y(0),
                      vx_mm_s(0), vy_mm_s(0), speed_mm_s(0),
                      state(SegmentState::PENDING), traveled_mm(0),
                      deferred_correction_x(0), deferred_correction_y(0) {}
};

class MotionQueue {
public:
    MotionQueue();

    /// Set calibrated speed constants (call once at startup)
    void setSpeedCalibration(float slow_mm_s, float normal_mm_s, float fast_mm_s);

    /// Enqueue a new motion segment.
    /// @param currentX, currentY  current estimated position for computing targets
    /// @returns true if enqueued, false if queue is full
    bool enqueue(MoveDirection direction, uint16_t distance_mm,
                 SpeedLevel speed, CorrectionPolicy policy,
                 float currentX, float currentY);

    /// Called every control-loop tick.  Returns true if there is an active
    /// segment being executed.
    /// @param dt_ms elapsed time since last call
    bool tick(uint32_t dt_ms);

    /// Get the current motion velocity to apply to motors.
    /// Returns (0, 0) if no segment is active.
    void getCurrentVelocity(float &vx_mm_s, float &vy_mm_s) const;

    /// Get the correction policy of the active segment
    CorrectionPolicy getActivePolicy() const;

    /// Store a deferred correction for the current segment
    void storeDeferredCorrection(float errX, float errY);

    /// Get deferred correction (when segment finishes)
    void getDeferredCorrection(float &errX, float &errY) const;

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

    float _getSpeedMmS(SpeedLevel level) const;
    void  _advanceToNext();
};

#endif // MOTION_QUEUE_HPP
