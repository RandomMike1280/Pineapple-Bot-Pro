#ifndef DEAD_RECKONING_HPP
#define DEAD_RECKONING_HPP

#include <Arduino.h>

// ============================================================================
// Dead-Reckoning Engine
// ============================================================================
// Maintains an estimated (x, y) position in millimeters using time-based
// integration of velocity.  Stores a rolling history of past positions so the
// latency-compensator can look up "where did we think we were at time T?"
// and apply smooth corrections without jarring jumps.
// ============================================================================

// Ring-buffer depth — must be a power of 2 for fast modulo via bitmask
#define DR_HISTORY_SIZE  256
#define DR_HISTORY_MASK  (DR_HISTORY_SIZE - 1)

struct PositionSnapshot {
    uint32_t timestamp_ms;   // millis() at which this snapshot was taken
    float    x;              // estimated X in mm
    float    y;              // estimated Y in mm
    float    angle;          // estimated angle in degrees
};

class DeadReckoning {
public:
    DeadReckoning();

    /// Set distance scaling factors for dead-reckoning mapping
    void setDistanceFactors(float factor_h, float factor_v);

    /// Reset position to a known origin
    void reset(float x0 = 0.0f, float y0 = 0.0f, float angle0 = 0.0f);

    /// Call every control-loop tick.
    /// @param vx_mm_s      current velocity along X axis (mm/s)
    /// @param vy_mm_s      current velocity along Y axis (mm/s)
    /// @param omega_deg_s  current angular velocity (deg/s)
    /// @param dt_ms        elapsed time since last call (ms)
    void update(float vx_mm_s, float vy_mm_s, float omega_deg_s, uint32_t dt_ms);

    /// Look up the estimated position at a past timestamp.
    /// Uses binary search over the history ring buffer.
    /// Returns false if the timestamp is too old (not in buffer).
    bool getPositionAt(uint32_t timestamp_ms, float &out_x, float &out_y, float &out_angle) const;

    /// Apply a smooth correction that will blend in over @p blend_ms.
    /// The correction is the error vector (observed − estimated at the
    /// observation time), projected forward to the present.
    void applyCorrection(float error_x, float error_y, float error_angle, uint32_t blend_ms = 200);

    /// Get the current best-estimate position (with any blended correction).
    void getCurrentPosition(float &out_x, float &out_y, float &out_angle) const;

    /// Raw estimated position (without active correction blend)
    void getRawPosition(float &out_x, float &out_y, float &out_angle) const;

    /// Get the current correction magnitude (for telemetry)
    float getCorrectionMagnitude() const;

private:
    // --- Core position state ---
    float _x;           // mm
    float _y;           // mm
    float _angle;       // deg

    // --- Distance Calibration ---
    float _distFactorH;
    float _distFactorV;

    // --- History ring buffer ---
    PositionSnapshot _history[DR_HISTORY_SIZE];
    uint16_t         _historyHead;   // next write index
    uint16_t         _historyCount;  // entries written (saturates at DR_HISTORY_SIZE)

    void _recordSnapshot(uint32_t now);

    // --- Smooth correction state ---
    float    _corrX;             // remaining correction to blend (mm)
    float    _corrY;
    float    _corrAngle;         // remaining correction to blend (deg)
    float    _corrTotalX;        // original correction vector
    float    _corrTotalY;
    float    _corrTotalAngle;
    uint32_t _corrStartTime;    // millis() when correction started
    uint32_t _corrDuration;     // blend window (ms)
    bool     _corrActive;

    float _blendedCorrX() const;
    float _blendedCorrY() const;
    float _blendedCorrAngle() const;
};

#endif // DEAD_RECKONING_HPP
