#ifndef DEAD_RECKONING_HPP
#define DEAD_RECKONING_HPP

#include <Arduino.h>
#include "../Rotation/Rotation.hpp"

// ============================================================================
// Dead-Reckoning Engine
// ============================================================================
// Maintains an estimated (x, y) position in millimeters using time-based
// integration of velocity.  Stores a rolling history of past positions so the
// latency-compensator can look up "where did we think we were at time T?"
// and apply smooth corrections without jarring jumps.
//
// The engine uses a two-tier position model:
//   1. Internal odometer  — continuously integrated from velocity
//   2. Global anchor     — absolute correction from camera observations
//
// Current position = anchor + (odometer - odometer_at_anchor_time)
// ============================================================================

// Ring-buffer depth — must be a power of 2 for fast modulo via bitmask
#define DR_HISTORY_SIZE  256
#define DR_HISTORY_MASK  (DR_HISTORY_SIZE - 1)

struct PositionSnapshot {
    uint32_t timestamp_ms;   // millis() at which this snapshot was taken
    float    ix;             // internal odometer X in mm
    float    iy;             // internal odometer Y in mm
    Rotation ia;              // internal odometer angle
};

class DeadReckoning {
public:
    DeadReckoning();

    /// Set distance scaling factors for dead-reckoning mapping
    void setDistanceFactors(float factor_h, float factor_v);

    /// Reset position to a known origin
    void reset(float x0 = 0.0f, float y0 = 0.0f, float angle0 = 0.0f);

    /// Call every control-loop tick.
    void update(float vx_mm_s, float vy_mm_s, float omega_deg_s, uint32_t dt_ms);

    /// Look up the internal odometer state at a past timestamp.
    bool getPositionAt(uint32_t timestamp_ms, float &out_ix, float &out_iy, float &out_ia) const;

    /// Force the robot's belief to a fixed camera anchor.
    /// The robot's current position becomes: Anchor + (CurrentOdo - OdoAtCaptureTime)
    void setAnchor(float worldX, float worldY, float worldAngle, uint32_t captureTime);

    /// Get the current best-estimate position (Anchor + Delta)
    void getCurrentPosition(float &out_x, float &out_y, float &out_angle) const;

    /// Get the pure odometer position
    void getOdoPosition(float &out_x, float &out_y, float &out_angle) const;

private:
    // === Odometer (continuous integration) ===
    float    _ix;
    float    _iy;
    Rotation _ia;

    // === Global Anchor (absolute camera correction) ===
    float    _gx;
    float    _gy;
    Rotation _ga;

    // === Odometer value at the time of last anchor capture ===
    float    _ax;
    float    _ay;
    Rotation _aa;

    // === Distance Calibration ===
    float _distFactorH;
    float _distFactorV;

    // === Position History Ring Buffer (for latency compensation) ===
    PositionSnapshot _history[DR_HISTORY_SIZE];
    uint16_t         _historyHead;   // next write index
    uint16_t         _historyCount;  // entries written (saturates at DR_HISTORY_SIZE)

    void _recordSnapshot(uint32_t now);
};

#endif // DEAD_RECKONING_HPP
