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
      _cameraLatencyMs(0), _clockOffsetMs(0), _offsetInitialized(false),
      _lastObservedAngle(0), _hasObservedAngle(false)
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

    // --- Timestamp Validation ---
    // Guard against overflow/underflow in clock offset arithmetic.
    // If captureTime would wrap to > ~49.7 days in the future, skip this observation.
    uint32_t nowMs = millis();
    if ((int64_t)phoneTimestamp + _clockOffsetMs < 0 ||
        (int64_t)phoneTimestamp + _clockOffsetMs > (int64_t)nowMs + 60000) {
        Serial.printf("[LC] Invalid capture time (phone=%lu offset=%lld now=%lu) — skipping\n",
            (unsigned long)phoneTimestamp, (long long)_clockOffsetMs, (unsigned long)nowMs);
        return;
    }

    // --- Absolute Time Synchronization ---
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
    
    // shortest path difference (only if angle is available)
    if (!isnan(observedAngle)) {
        _lastDriftAngle = Rotation(observedAngle) - Rotation(estAngle);
        // Store ground-truth angle for App display (report what camera sees)
        _lastObservedAngle = Rotation::normalize(observedAngle);
        _hasObservedAngle = true;
        // Telemetry: Compare Ground Truth (Camera) vs Robot's Belief (DR history)
        Serial.printf("[SYNC] Observed (Cam): %.1f | Estimated (DR): %.1f | Error: %.1f\n",
                      _lastObservedAngle, Rotation::normalize(estAngle), _lastDriftAngle);
    } else {
        _lastDriftAngle = 0.0f;
        Serial.printf("[SYNC] Observed (Cam): NAN | Estimated (DR): %.1f | Error: 0.0\n",
                      Rotation::normalize(estAngle));
    }

    _lastDriftMagnitude = sqrtf(_lastDriftX * _lastDriftX + _lastDriftY * _lastDriftY);

    // Step 4: Re-ground Kalman state BEFORE setting the dead-reckoning anchor.
    // When camera position diverges from dead-reckoning by > EMERGENCY_THRESHOLD,
    // the position jump causes Kalman velocity to flip sign (robot appears to move
    // backward). By re-grounding Kalman first (zeroing velocity), the motion queue
    // computes the correct direction toward the target without sign inversions.
    if (_lastDriftMagnitude > _emergencyThresholdMm) {
        _mq->regroundPosition(observedX, observedY);
    }

    // Step 5: Apply anchor offset so dead-reckoning stays in sync with camera.
    //
    // Bug fix (2026-04-18): Previously, emergency corrections used setAnchorPositionOnly()
    // which updated x/y but NOT angle. When angle drifted 130°+, the stabilization
    // layer saw huge heading_err and commanded max omega, causing erratic behavior.
    // Now angle is ALWAYS corrected, but with a continuity guard:
    //
    //   - If |angle_jump| <= 90°: normal anchor correction (position + angle)
    //   - If |angle_jump| > 90°: position-only anchor (avoid angle discontinuity)
    //
    // The 90° threshold handles the case where the camera reports an extreme angle
    // that clearly represents a single-frame outlier or wrapping artifact (e.g. DR=0°,
    // Cam=229°, jump=-129° is actually only -129° mod 360 = see note below).
    //
    // NOTE on angle wrapping: Camera angle 229° in a [0,360) system with DR at 0°
    // should be interpreted as 229° = -131° (mod 360). The Rotation::operator-
    // already uses shortest-path subtraction, so _lastDriftAngle = Rotation(229) - 0
    // correctly gives -131°. But when this -131° is fed into stabilization
    // (heading_err * gain), it produces max omega in the wrong direction.
    // We now use the continuity guard to detect and suppress these pathological jumps.

    bool hasValidAngle = !isnan(observedAngle);
    if (hasValidAngle) {
        // Compute the shortest-path angle correction
        float angleCorrection = Rotation(observedAngle) - Rotation(estAngle);
        float absAngleCorrection = fabsf(angleCorrection);

        // If angle jump is pathological (> 90°), apply position-only anchor to
        // avoid a massive stabilization spike that would destabilize the robot.
        // Use position-only anchor: fixes x/y drift without disrupting heading.
        if (absAngleCorrection > 90.0f) {
            _dr->setAnchorPositionOnly(observedX, observedY, captureTime);
        } else {
            _dr->setAnchor(observedX, observedY, observedAngle, captureTime);
        }
    } else {
        // No angle available — correct position only
        _dr->setAnchorPositionOnly(observedX, observedY, captureTime);
    }

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

float LatencyCompensator::getLastObservedAngle() const {
    if (!_hasObservedAngle) return NAN;
    return _lastObservedAngle;
}
