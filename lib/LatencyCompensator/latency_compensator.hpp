#ifndef LATENCY_COMPENSATOR_HPP
#define LATENCY_COMPENSATOR_HPP

#include <Arduino.h>
#include "dead_reckoning.hpp"
#include "motion_queue.hpp"

// ============================================================================
// Latency Compensator
// ============================================================================
// Handles camera position updates that arrive with network latency.
// Uses the dead-reckoning history buffer to compare "where the robot thinks
// it was" at the camera capture time vs "where the camera actually saw it",
// then applies a smooth correction forward to the present.
// ============================================================================

class LatencyCompensator {
public:
    LatencyCompensator();

    /// Set references to the dead-reckoning engine and motion queue
    void init(DeadReckoning* dr, MotionQueue* mq);

    /// Called when a camera observation arrives from the phone.
    /// @param phoneTimestamp  the phone's timestamp when frame was captured (ms)
    /// @param observedX       observed X position in mm
    /// @param observedY       observed Y position in mm
    /// @param observedAngle   observed angle in degrees
    /// @param correctionBlendMs  time to blend correction (default 200ms)
    void onCameraUpdate(uint32_t phoneTimestamp, float observedX, float observedY, float observedAngle,
                        uint32_t correctionBlendMs = 200);

    /// Called when a PONG response arrives.
    /// @param originalSendTime  the timestamp from the original PING
    void onPong(uint32_t originalSendTime);

    /// Get current estimated RTT in milliseconds
    uint32_t getRttMs() const;

    /// Get the last computed drift error magnitude (mm)
    float getLastDriftMagnitude() const;

    /// Check if an emergency correction was triggered on the last update
    bool wasEmergencyTriggered() const;

    /// Set drift thresholds
    void setThresholds(float driftThresholdMm, float emergencyThresholdMm);

private:
    DeadReckoning* _dr;
    MotionQueue*   _mq;

    // RTT tracking (exponential moving average)
    uint32_t _rttMs;
    bool     _rttInitialized;
    static const int RTT_EMA_ALPHA_SHIFT = 3;  // 1/8 weight for new samples

    // Drift metrics
    float _lastDriftX;
    float _lastDriftY;
    float _lastDriftAngle;
    float _lastDriftMagnitude;
    bool  _emergencyTriggered;

    // Thresholds
    float _driftThresholdMm;
    float _emergencyThresholdMm;
};

#endif // LATENCY_COMPENSATOR_HPP
