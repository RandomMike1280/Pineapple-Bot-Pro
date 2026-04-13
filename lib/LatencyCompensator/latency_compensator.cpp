#include "latency_compensator.hpp"
#include <math.h>

// ============================================================================
// Construction / Init
// ============================================================================

LatencyCompensator::LatencyCompensator()
    : _dr(nullptr), _mq(nullptr),
      _rttMs(100), _rttInitialized(false),
      _lastDriftX(0), _lastDriftY(0), _lastDriftAngle(0),
      _lastDriftMagnitude(0), _emergencyTriggered(false),
      _driftThresholdMm(20.0f), _emergencyThresholdMm(120.0f),
      _cameraLatencyMs(0), _clockOffsetMs(0), _offsetInitialized(false)
{
}

void LatencyCompensator::init(DeadReckoning* dr, MotionQueue* mq) {
    _dr = dr;
    _mq = mq;
}

void LatencyCompensator::setThresholds(float driftThresholdMm, float emergencyThresholdMm) {
    _driftThresholdMm = driftThresholdMm;
    _emergencyThresholdMm = emergencyThresholdMm;
}

void LatencyCompensator::setCameraLatency(uint32_t cameraLatencyMs) {
    _cameraLatencyMs = cameraLatencyMs;
}

// ============================================================================

// Camera observation processing — the core latency compensation logic
// ============================================================================
//
// The phone sends: "at time T, the robot was at (x, y)"
// But this data arrives with network latency.  We compensate:
//
// 1. Estimate the actual capture time:  capture_time = phone_time - RTT/2
//    (the observation is offset by half the round-trip latency)
//
// 2. Look up where our dead-reckoning THOUGHT we were at capture_time
//    using the position history buffer.
//
// 3. Compute the error:  error = observed_position - estimated_position
//    This tells us how much our dead-reckoning has drifted.
//
// 4. Apply the correction smoothly (blended over ~200ms) so there's
//    no visible jump in the robot's motion.
//
// ============================================================================

void LatencyCompensator::onCameraUpdate(uint32_t phoneTimestamp,
                                         float observedX, float observedY, float observedAngle,
                                         uint32_t correctionBlendMs) {
    if (_dr == nullptr || _mq == nullptr) return;
    if (!_offsetInitialized) {
        Serial.println("[LC] Clock not synchronized yet — skipping camera update");
        return;
    }

    _emergencyTriggered = false;

    // --- Absolute Time Synchronization ---
    // The phone sent its absolute timestamp (phoneTimestamp) of the frame capture.
    // We translate this to our local timeline using the calculated offset.
    uint32_t captureTime = (uint32_t)((int64_t)phoneTimestamp + _clockOffsetMs);

    // Step 2: Look up our estimated position at that exact past moment
    float estX, estY, estAngle;
    bool found = _dr->getPositionAt(captureTime, estX, estY, estAngle);
    if (!found) {
        // Observation is too old — our history doesn't go back that far.
        // This can happen if RTT is very high.  Skip this observation.
        Serial.println("[LC] Camera observation too old, skipping");
        return;
    }

    // Step 3: Compute the drift error
    _lastDriftX = observedX - estX;
    _lastDriftY = observedY - estY;
    _lastDriftAngle = observedAngle - estAngle;
    
    // Normalize angle error to [-180, 180]
    while (_lastDriftAngle > 180.0f) _lastDriftAngle -= 360.0f;
    while (_lastDriftAngle < -180.0f) _lastDriftAngle += 360.0f;

    _lastDriftMagnitude = sqrtf(_lastDriftX * _lastDriftX + _lastDriftY * _lastDriftY);

    // Step 4: Apply correction based on the active segment's policy
    CorrectionPolicy policy = _mq->getActivePolicy();

    if (policy == CorrectionPolicy::NONE) {
        // Pure dead-reckoning — ignore camera data
        return;
    }

    if (policy == CorrectionPolicy::DEFERRED) {
        // Store the correction for application at segment boundary
        _mq->storeDeferredCorrection(_lastDriftX, _lastDriftY, _lastDriftAngle);
        return;
    }

    // policy == CorrectionPolicy::LIVE
    if (_lastDriftMagnitude < 2.0f) {
        // Drift is negligible — don't bother correcting
        return;
    }

    if (_lastDriftMagnitude > _emergencyThresholdMm) {
        // Emergency! Drift is dangerously large.
        // Signal that the motion queue should decelerate briefly.
        _emergencyTriggered = true;
        Serial.printf("[LC] EMERGENCY drift: %.1f mm — decelerating\n", _lastDriftMagnitude);
        // Apply immediate correction with shorter blend
        _dr->applyCorrection(_lastDriftX, _lastDriftY, _lastDriftAngle, correctionBlendMs / 2);
    } else if (_lastDriftMagnitude > _driftThresholdMm || abs(_lastDriftAngle) > 2.0f) {
        // Significant drift — apply full correction
        Serial.printf("[LC] Correcting drift: %.1f mm / %.1f deg (err: %.1f, %.1f)\n",
                      _lastDriftMagnitude, _lastDriftAngle, _lastDriftX, _lastDriftY);
        _dr->applyCorrection(_lastDriftX, _lastDriftY, _lastDriftAngle, correctionBlendMs);
    } else {
        // Minor drift — apply gentle partial correction (50%)
        _dr->applyCorrection(_lastDriftX * 0.5f, _lastDriftY * 0.5f, _lastDriftAngle * 0.5f, correctionBlendMs);
    }
}

// ============================================================================
// RTT tracking via PONG responses
// ============================================================================

void LatencyCompensator::onPong(uint32_t originalSendTime, uint32_t remoteTimestamp) {
    uint32_t now = millis();
    uint32_t measuredRtt = now - originalSendTime;

    if (!_rttInitialized) {
        _rttMs = measuredRtt;
        _rttInitialized = true;
    } else {
        // EMA: average the Rtt over time to smooth out jitter
        _rttMs = _rttMs - (_rttMs >> RTT_EMA_ALPHA_SHIFT) + (measuredRtt >> RTT_EMA_ALPHA_SHIFT);
    }

    // --- NTP-style Clock Synchronization ---
    if (remoteTimestamp != 0) {
        // Estimated robot-local time when the phone sent the pong:
        // Assume network delay is symmetric (RTT / 2)
        uint32_t estimatedLocalTimeAtPong = originalSendTime + (measuredRtt / 2);
        
        // Offset = RobotTime - PhoneTime
        int64_t sampleOffset = (int64_t)estimatedLocalTimeAtPong - (int64_t)remoteTimestamp;

        if (!_offsetInitialized) {
            _clockOffsetMs = sampleOffset;
            _offsetInitialized = true;
            Serial.printf("[LC] Clock Sync: Offset=%lld ms (Initial)\n", _clockOffsetMs);
        } else {
            // Apply gentle smoothing to the offset to prevent jumps
            // Use same EMA shift for simplicity
            _clockOffsetMs = _clockOffsetMs - (_clockOffsetMs >> RTT_EMA_ALPHA_SHIFT) + (sampleOffset >> RTT_EMA_ALPHA_SHIFT);
        }
    }
}

// ============================================================================
// Accessors
// ============================================================================

uint32_t LatencyCompensator::getRttMs() const {
    return _rttMs;
}

float LatencyCompensator::getLastDriftMagnitude() const {
    return _lastDriftMagnitude;
}

bool LatencyCompensator::wasEmergencyTriggered() const {
    return _emergencyTriggered;
}
