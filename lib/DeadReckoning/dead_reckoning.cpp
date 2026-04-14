#include "dead_reckoning.hpp"

// ============================================================================
// Construction / Reset
// ============================================================================

DeadReckoning::DeadReckoning()
    : _x(0), _y(0), _angle(0),
      _distFactorH(1.0f), _distFactorV(1.0f),
      _historyHead(0), _historyCount(0),
      _corrX(0), _corrY(0), _corrAngle(0),
      _corrTotalX(0), _corrTotalY(0), _corrTotalAngle(0),
      _corrStartTime(0), _corrDuration(0),
      _corrActive(false)
{
    memset(_history, 0, sizeof(_history));
}

void DeadReckoning::setDistanceFactors(float factor_h, float factor_v) {
    _distFactorH = factor_h;
    _distFactorV = factor_v;
}

void DeadReckoning::reset(float x0, float y0, float angle0) {
    _x = x0;
    _y = y0;
    _angle = Rotation(angle0);
    _historyHead  = 0;
    _historyCount = 0;
    _corrActive   = false;
    _corrX = _corrY = _corrAngle = 0;
    _corrTotalX = _corrTotalY = _corrTotalAngle = 0;
    memset(_history, 0, sizeof(_history));
}

// ============================================================================
// Core integration — called every control-loop tick (~5-10 ms)
// ============================================================================

void DeadReckoning::update(float vx_mm_s, float vy_mm_s, float omega_deg_s, uint32_t dt_ms) {
    if (dt_ms == 0) return;

    float dt_s = dt_ms / 1000.0f;
    _x += (vx_mm_s * _distFactorH) * dt_s;
    _y += (vy_mm_s * _distFactorV) * dt_s;
    _angle += omega_deg_s * dt_s;

    // Advance the smooth correction blend (decay remaining correction)
    if (_corrActive) {
        uint32_t now = millis();
        uint32_t elapsed = now - _corrStartTime;
        if (elapsed >= _corrDuration) {
            // Correction fully applied — absorb remainder into raw position
            _x += _corrX;
            _y += _corrY;
            _angle += _corrAngle;
            _corrX = _corrY = _corrAngle = 0;
            _corrActive = false;
        }
    }

    // Store snapshot
    _recordSnapshot(millis());
}

// ============================================================================
// History buffer management
// ============================================================================

void DeadReckoning::_recordSnapshot(uint32_t now) {
    PositionSnapshot &s = _history[_historyHead & DR_HISTORY_MASK];
    s.timestamp_ms = now;
    s.x = _x;
    s.y = _y;
    s.angle = _angle;
    _historyHead++;
    if (_historyCount < DR_HISTORY_SIZE) _historyCount++;
}

// ============================================================================
// Timestamp-based position lookup (binary search over ring buffer)
// ============================================================================

bool DeadReckoning::getPositionAt(uint32_t timestamp_ms, float &out_x, float &out_y, float &out_angle) const {
    if (_historyCount == 0) return false;

    // The buffer is a ring; entries are in chronological order from
    // (head - count) to (head - 1), wrapped via DR_HISTORY_MASK.
    uint16_t oldest = (_historyHead - _historyCount) & DR_HISTORY_MASK;
    uint16_t newest = (_historyHead - 1)             & DR_HISTORY_MASK;

    uint32_t oldestTs = _history[oldest].timestamp_ms;
    uint32_t newestTs = _history[newest].timestamp_ms;

    // Out of range checks
    if (timestamp_ms < oldestTs) return false;  // too old
    if (timestamp_ms >= newestTs) {
        // Return most recent
        out_x = _history[newest].x;
        out_y = _history[newest].y;
        out_angle = _history[newest].angle;
        return true;
    }

    // Binary search within the ring buffer
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

    // Interpolate between idx-1 and idx for better accuracy
    if (lo > 0) {
        uint16_t prevIdx = (oldest + lo - 1) & DR_HISTORY_MASK;
        const PositionSnapshot &a = _history[prevIdx];
        const PositionSnapshot &b = _history[idx];
        uint32_t span = b.timestamp_ms - a.timestamp_ms;
        if (span > 0) {
            float t = (float)(timestamp_ms - a.timestamp_ms) / (float)span;
            out_x = a.x + (b.x - a.x) * t;
            out_y = a.y + (b.y - a.y) * t;
            // Use shortest-path interpolation for angle
            out_angle = Rotation::lerp(a.angle, b.angle, t);
            return true;
        }
    }

    out_x = _history[idx].x;
    out_y = _history[idx].y;
    out_angle = _history[idx].angle;
    return true;
}

// ============================================================================
// Smooth correction — blends error over a time window
// ============================================================================

void DeadReckoning::applyCorrection(float error_x, float error_y, float error_angle, uint32_t blend_ms) {
    if (blend_ms == 0) {
        // Instant correction
        _x += error_x;
        _y += error_y;
        _angle += error_angle;
        return;
    }

    _corrTotalX   = error_x;
    _corrTotalY   = error_y;
    _corrTotalAngle = error_angle;
    _corrX        = error_x;
    _corrY        = error_y;
    _corrAngle    = error_angle;
    _corrStartTime = millis();
    _corrDuration  = blend_ms;
    _corrActive    = true;
}

float DeadReckoning::_blendedCorrX() const {
    if (!_corrActive) return 0.0f;
    uint32_t elapsed = millis() - _corrStartTime;
    if (elapsed >= _corrDuration) return _corrTotalX;  // fully blended
    float t = (float)elapsed / (float)_corrDuration;
    return _corrTotalX * t;
}

float DeadReckoning::_blendedCorrY() const {
    if (!_corrActive) return 0.0f;
    uint32_t elapsed = millis() - _corrStartTime;
    if (elapsed >= _corrDuration) return _corrTotalY;
    float t = (float)elapsed / (float)_corrDuration;
    return _corrTotalY * t;
}

float DeadReckoning::_blendedCorrAngle() const {
    if (!_corrActive) return 0.0f;
    uint32_t elapsed = millis() - _corrStartTime;
    if (elapsed >= _corrDuration) return _corrTotalAngle;
    float t = (float)elapsed / (float)_corrDuration;
    return _corrTotalAngle * t;
}

// ============================================================================
// Position accessors
// ============================================================================

void DeadReckoning::getCurrentPosition(float &out_x, float &out_y, float &out_angle) const {
    out_x = _x + _blendedCorrX();
    out_y = _y + _blendedCorrY();
    out_angle = _angle + _blendedCorrAngle();
}

void DeadReckoning::getRawPosition(float &out_x, float &out_y, float &out_angle) const {
    out_x = _x;
    out_y = _y;
    out_angle = _angle;
}

float DeadReckoning::getCorrectionMagnitude() const {
    float cx = _blendedCorrX();
    float cy = _blendedCorrY();
    return sqrtf(cx * cx + cy * cy);
}
