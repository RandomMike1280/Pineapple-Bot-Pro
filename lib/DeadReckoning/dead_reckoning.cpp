#include "dead_reckoning.hpp"
#include <math.h>

// ============================================================================
// Agent Debug Logging — Serial NDJSON for long-run diagnostics
// H1: Float precision of _ix/_iy vs anchor _ax/_ay
// ============================================================================
static unsigned long _lastDrFloatLogMs = 0;
static const unsigned long DR_FLOAT_LOG_INTERVAL_MS = 5000;

static void _dbgLogDRFloat(float ix, float iy, float ax, float ay) {
    unsigned long now = millis();
    if (now - _lastDrFloatLogMs < DR_FLOAT_LOG_INTERVAL_MS) return;
    _lastDrFloatLogMs = now;
    Serial.printf(
        "{\"sessionId\":\"eb5734\",\"id\":\"dr_%lu\",\"timestamp\":%lu,"
        "\"location\":\"dead_reckoning.cpp:update\",\"message\":\"H1_float_precision\",\"hypothesisId\":\"H1\","
        "\"data\":{\"ix\":%.1f,\"iy\":%.1f,\"ax\":%.1f,\"ay\":%.1f,"
        "\"odo_mag\":%.1f,\"anchor_mag\":%.1f}}\n",
        now, now, ix, iy, ax, ay,
        (double)sqrtf(ix*ix + iy*iy),
        (double)sqrtf(ax*ax + ay*ay));
}

// ============================================================================
// Construction / Reset
// ============================================================================

DeadReckoning::DeadReckoning()
    : _ix(0), _iy(0), _ia(0),
      _gx(0), _gy(0), _ga(0),
      _ax(0), _ay(0), _aa(0),
      _distFactorH(1.0f), _distFactorV(1.0f),
      _historyHead(0), _historyCount(0),
      _historyMutex(xSemaphoreCreateMutex())
{
    memset(_history, 0, sizeof(_history));
}

void DeadReckoning::setDistanceFactors(float factor_h, float factor_v) {
    _distFactorH = factor_h;
    _distFactorV = factor_v;
}

void DeadReckoning::reset(float x0, float y0, float angle0) {
    _ix = 0; _iy = 0; _ia = Rotation(0);
    _ax = 0; _ay = 0; _aa = Rotation(0);
    _gx = x0; _gy = y0; _ga = Rotation(angle0);
    _historyHead  = 0;
    _historyCount = 0;
}

// ============================================================================
// Internal Odometer Update
// ============================================================================

void DeadReckoning::update(float vx_mm_s, float vy_mm_s, float omega_deg_s, uint32_t dt_ms) {
    float dt = (float)dt_ms / 1000.0f;
    
    // Pure integration of motor movement in world frame
    // (Note: Robot reports vx/vy in its own body frame, normally we'd rotate it, 
    // but here we assume the input is already field-aligned vx/vy from the kinematics)
    _ix += vx_mm_s * dt * _distFactorH;
    _iy += vy_mm_s * dt * _distFactorV;
    _ia += omega_deg_s * dt;

    // #region agent_debug_log H1: float precision decay in odometer vs anchor
    _dbgLogDRFloat(_ix, _iy, _ax, _ay);
    // #endregion

    _recordSnapshot(millis());
}

void DeadReckoning::_recordSnapshot(uint32_t now) {
    if (_historyMutex != nullptr) {
        xSemaphoreTake(_historyMutex, portMAX_DELAY);
    }
    PositionSnapshot &snap = _history[_historyHead];
    snap.timestamp_ms = now;
    snap.ix = _ix;
    snap.iy = _iy;
    snap.ia = _ia;

    _historyHead = (_historyHead + 1) & DR_HISTORY_MASK;
    if (_historyCount < DR_HISTORY_SIZE) _historyCount++;
    if (_historyMutex != nullptr) {
        xSemaphoreGive(_historyMutex);
    }
}

// ============================================================================
// Anchor-Based Estimation (The "Three Data Store" model)
// ============================================================================

void DeadReckoning::setAnchor(float worldX, float worldY, float worldAngle, uint32_t captureTime) {
    float odoX, odoY, odoA;
    if (getPositionAt(captureTime, odoX, odoY, odoA)) {
        // We found exactly what our odo was doing when the photo was taken.
        _gx = worldX;
        _gy = worldY;
        _ga = Rotation(worldAngle);
        _ax = odoX;
        _ay = odoY;
        _aa = Rotation(odoA);
    } else {
        // Capture time is outside our history window — use current odo as fallback.
        // This is slightly less accurate but ensures the correction is always applied.
        _gx = worldX;
        _gy = worldY;
        _ga = Rotation(worldAngle);
        _ax = _ix;
        _ay = _iy;
        _aa = Rotation(_ia);
    }
}

void DeadReckoning::setAnchorPositionOnly(float worldX, float worldY, uint32_t captureTime) {
    float odoX, odoY, odoA;
    if (getPositionAt(captureTime, odoX, odoY, odoA)) {
        // Only update position anchor; preserve angle anchor
        _gx = worldX;
        _gy = worldY;
        _ax = odoX;
        _ay = odoY;
        // _ga and _aa unchanged — angle stays the same
    } else {
        _gx = worldX;
        _gy = worldY;
        _ax = _ix;
        _ay = _iy;
    }
}

void DeadReckoning::getCurrentPosition(float &out_x, float &out_y, float &out_angle) const {
    // Current = Anchor + (CurrentOdo - OdoAtAnchor)
    out_x = _gx + (_ix - _ax);
    out_y = _gy + (_iy - _ay);
    
    // Shortest-path rotation difference ensure we don't glitch at 360/0
    out_angle = _ga + (_ia - _aa);
}

void DeadReckoning::getOdoPosition(float &out_x, float &out_y, float &out_angle) const {
    out_x = _ix;
    out_y = _iy;
    out_angle = _ia;
}

// ============================================================================
// History Lookup
// ============================================================================

bool DeadReckoning::getPositionAt(uint32_t timestamp_ms, float &out_ix, float &out_iy, float &out_ia) const {
    if (_historyCount == 0) return false;

    if (_historyMutex != nullptr) {
        xSemaphoreTake(_historyMutex, portMAX_DELAY);
    }
    uint16_t oldest = (_historyHead - _historyCount) & DR_HISTORY_MASK;
    uint32_t tOldest = _history[oldest].timestamp_ms;
    uint32_t tNewest = _history[(_historyHead - 1) & DR_HISTORY_MASK].timestamp_ms;

    if (timestamp_ms < tOldest || timestamp_ms > tNewest) {
        if (_historyMutex != nullptr) xSemaphoreGive(_historyMutex);
        return false;
    }

    // Binary search
    uint16_t lo = 0;
    uint16_t hi = _historyCount - 1;
    while (lo < hi) {
        uint16_t mid = (lo + hi) / 2;
        uint16_t idx = (oldest + mid) & DR_HISTORY_MASK;
        if (_history[idx].timestamp_ms < timestamp_ms) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }

    uint16_t idx = (oldest + lo) & DR_HISTORY_MASK;

    if (lo > 0) {
        uint16_t prevIdx = (oldest + lo - 1) & DR_HISTORY_MASK;
        const PositionSnapshot &a = _history[prevIdx];
        const PositionSnapshot &b = _history[idx];
        uint32_t span = b.timestamp_ms - a.timestamp_ms;
        if (span > 0) {
            float t = (float)(timestamp_ms - a.timestamp_ms) / (float)span;
            out_ix = a.ix + (b.ix - a.ix) * t;
            out_iy = a.iy + (b.iy - a.iy) * t;
            out_ia = Rotation::lerp(a.ia, b.ia, t);
            if (_historyMutex != nullptr) xSemaphoreGive(_historyMutex);
            return true;
        }
    }

    out_ix = _history[idx].ix;
    out_iy = _history[idx].iy;
    out_ia = _history[idx].ia;
    if (_historyMutex != nullptr) xSemaphoreGive(_historyMutex);
    return true;
}
