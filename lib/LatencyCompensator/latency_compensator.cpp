#include "latency_compensator.hpp"
#include <math.h>
#include "../Rotation/Rotation.hpp"

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

    // Step 3: Compute the drift error using modulo-360 Rotation logic
    _lastDriftX = observedX - estX;
    _lastDriftY = observedY - estY;
    
    // shortest path difference
    _lastDriftAngle = Rotation(observedAngle) - Rotation(estAngle);
    
    // Telemetry: Compare Ground Truth (Camera) vs Robot's Belief (DR history)
    Serial.printf("[SYNC] Observed (Cam): %.1f | Estimated (DR): %.1f | Error: %.1f\n",
                  Rotation::normalize(observedAngle), Rotation::normalize(estAngle), _lastDriftAngle);

    _lastDriftMagnitude = sqrtf(_lastDriftX * _lastDriftX + _lastDriftY * _lastDriftY);

    // Step 4: Apply instant correction (History Rewriting)
    // We trust the fixed camera as a 100% ground truth. By applying the drift
    // instantly (blend = 0), we satisfy: Current = GroundTruth_past + (Current - Past)
    CorrectionPolicy policy = _mq->getActivePolicy();

    if (policy == CorrectionPolicy::NONE) {
        return; // Pure dead-reckoning
    }

    if (policy == CorrectionPolicy::DEFERRED) {
        _mq->storeDeferredCorrection(_lastDriftX, _lastDriftY, _lastDriftAngle);
        return;
    }

    // policy == CorrectionPolicy::LIVE
    // Apply full correction instantly to "re-ground" the robot's belief
    _dr->applyCorrection(_lastDriftX, _lastDriftY, _lastDriftAngle, 0);
    
    if (_lastDriftMagnitude > _emergencyThresholdMm) {
        _emergencyTriggered = true;
        Serial.printf("[LC] RE-GROUNDED (EMERGENCY): %.1f mm / %.1f deg\n", _lastDriftMagnitude, _lastDriftAngle);
    } else {
        Serial.printf("[LC] RE-GROUNDED: %.1f mm / %.1f deg\n", _lastDriftMagnitude, _lastDriftAngle);
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
