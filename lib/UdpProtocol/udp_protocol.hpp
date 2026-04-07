#ifndef UDP_PROTOCOL_HPP
#define UDP_PROTOCOL_HPP

#include <Arduino.h>

// ============================================================================
// UDP Protocol — Structured message format for Robot ↔ Phone communication
// ============================================================================
// All messages are short ASCII strings for easy debugging on serial monitor.
// Format:  TYPE_CHAR:field1:field2:...
//
//   HELLO     H:<robot_id>
//   MOVE      M:<dir>:<dist_mm>:<speed>:<policy>
//   WAYPOINT  W:<target_x>:<target_y>:<speed>:<policy>
//   CAM       C:<timestamp_ms>:<x_mm>:<y_mm>
//   PING      P:<timestamp_ms>
//   PONG      Q:<orig_timestamp_ms>
//   STATUS    S:<x>:<y>:<queue_len>:<drift_mm>
//   ABORT     A
//   REGISTER  R:<robot_id>:<caps>
// ============================================================================

enum class MsgType : uint8_t {
    HELLO,
    MOVE,
    CAM,
    PING,
    PONG,
    STATUS,
    ABORT,
    REGISTER,
    WAYPOINT,
    UNKNOWN
};

enum class MoveDirection : uint8_t {
    UP = 0,
    DOWN,
    LEFT,
    RIGHT,
    INVALID
};

enum class SpeedLevel : uint8_t {
    SLOW = 0,
    NORMAL,
    FAST,
    INVALID
};

enum class CorrectionPolicy : uint8_t {
    LIVE = 0,
    DEFERRED,
    NONE,
    INVALID
};

// --- Parsed message container ---
struct UdpMessage {
    MsgType type;

    // MOVE fields
    MoveDirection direction;
    uint16_t      distance_mm;
    SpeedLevel    speed;
    CorrectionPolicy correctionPolicy;

    // WAYPOINT fields
    float         target_x;
    float         target_y;

    // CAM fields
    uint32_t cam_timestamp;
    float    cam_x;
    float    cam_y;

    // PING/PONG fields
    uint32_t ping_timestamp;

    // REGISTER fields
    char robot_id[8];
};

// --- Parsing ---

/// Parse a raw UDP buffer into a structured UdpMessage.
/// Returns true if parsing succeeded.
bool parseUdpMessage(const char* buffer, int len, UdpMessage &out);

// --- Building outgoing messages ---

/// Build a HELLO message: "H:<robot_id>"
int buildHelloMessage(char* buf, int maxLen, const char* robotId);

/// Build a PONG message: "Q:<orig_timestamp>"
int buildPongMessage(char* buf, int maxLen, uint32_t origTimestamp);

/// Build a STATUS message: "S:<x>:<y>:<queue_len>:<drift>"
int buildStatusMessage(char* buf, int maxLen,
                       float x, float y, int queueLen, float driftMm);

/// Build a REGISTER message: "R:<robot_id>:<caps>"
int buildRegisterMessage(char* buf, int maxLen,
                         const char* robotId, const char* capabilities);

// --- Helpers ---
MoveDirection   parseDirection(const char* str);
SpeedLevel      parseSpeed(const char* str);
CorrectionPolicy parsePolicy(const char* str);

/// Convert direction enum to velocity unit vector (vx, vy)
void directionToVector(MoveDirection dir, float &vx, float &vy);

#endif // UDP_PROTOCOL_HPP
