#ifndef ROTATION_HPP
#define ROTATION_HPP

#include <math.h>

/**
 * Custom Rotation class to handle modulo-360 degree arithmetic automatically.
 * Ensures all angles live in the [0, 360) space and provides signed 
 * shortest-path difference calculations.
 */
struct Rotation {
    float angle; // Always in [0, 360)

    Rotation(float a = 0.0f) : angle(normalize(a)) {}

    static float normalize(float a) {
        float b = fmodf(a, 360.0f);
        if (b < 0) b += 360.0f;
        return b;
    }

    // Difference (this - other) returns shortest path in [-180, 180]
    float operator-(const Rotation& other) const {
        float diff = angle - other.angle;
        if (diff > 180.0f) diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;
        return diff;
    }

    Rotation operator+(float d) const { return Rotation(angle + d); }
    Rotation operator-(float d) const { return Rotation(angle - d); }

    void operator+=(float d) { angle = normalize(angle + d); }
    void operator-=(float d) { angle = normalize(angle - d); }

    // Linear interpolation using shortest-path logic
    static Rotation lerp(Rotation a, Rotation b, float t) {
        float diff = b - a; // uses the shortest-path logic operator-
        return Rotation(a.angle + diff * t);
    }

    // Implicit conversion to float for easier logging
    operator float() const { return angle; }
};

#endif // ROTATION_HPP
