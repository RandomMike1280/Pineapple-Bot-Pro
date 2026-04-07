#include "udp_protocol.hpp"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// ============================================================================
// Helper: tokenize by ':'
// ============================================================================

static int tokenize(char* str, char** tokens, int maxTokens) {
    int count = 0;
    char* tok = strtok(str, ":");
    while (tok != NULL && count < maxTokens) {
        tokens[count++] = tok;
        tok = strtok(NULL, ":");
    }
    return count;
}

// ============================================================================
// Direction / Speed / Policy string parsing
// ============================================================================

MoveDirection parseDirection(const char* str) {
    if (strcmp(str, "up")    == 0 || strcmp(str, "U") == 0) return MoveDirection::UP;
    if (strcmp(str, "down")  == 0 || strcmp(str, "D") == 0) return MoveDirection::DOWN;
    if (strcmp(str, "left")  == 0 || strcmp(str, "L") == 0) return MoveDirection::LEFT;
    if (strcmp(str, "right") == 0 || strcmp(str, "R") == 0) return MoveDirection::RIGHT;
    return MoveDirection::INVALID;
}

SpeedLevel parseSpeed(const char* str) {
    if (strcmp(str, "slow")   == 0 || strcmp(str, "S") == 0) return SpeedLevel::SLOW;
    if (strcmp(str, "normal") == 0 || strcmp(str, "N") == 0) return SpeedLevel::NORMAL;
    if (strcmp(str, "fast")   == 0 || strcmp(str, "F") == 0) return SpeedLevel::FAST;
    return SpeedLevel::INVALID;
}

CorrectionPolicy parsePolicy(const char* str) {
    if (strcmp(str, "live")     == 0 || strcmp(str, "L") == 0) return CorrectionPolicy::LIVE;
    if (strcmp(str, "deferred") == 0 || strcmp(str, "D") == 0) return CorrectionPolicy::DEFERRED;
    if (strcmp(str, "none")     == 0 || strcmp(str, "N") == 0) return CorrectionPolicy::NONE;
    return CorrectionPolicy::INVALID;
}

void directionToVector(MoveDirection dir, float &vx, float &vy) {
    vx = 0.0f;
    vy = 0.0f;
    switch (dir) {
        case MoveDirection::UP:    vy =  1.0f; break;
        case MoveDirection::DOWN:  vy = -1.0f; break;
        case MoveDirection::LEFT:  vx = -1.0f; break;
        case MoveDirection::RIGHT: vx =  1.0f; break;
        default: break;
    }
}

// ============================================================================
// Message parsing
// ============================================================================

bool parseUdpMessage(const char* buffer, int len, UdpMessage &out) {
    if (len <= 0) return false;

    // Work on a copy because strtok mutates
    char copy[256];
    int copyLen = (len < 255) ? len : 255;
    memcpy(copy, buffer, copyLen);
    copy[copyLen] = '\0';

    // Remove trailing whitespace/newlines
    while (copyLen > 0 && (copy[copyLen-1] == '\n' || copy[copyLen-1] == '\r' || copy[copyLen-1] == ' ')) {
        copy[--copyLen] = '\0';
    }

    char* tokens[8];
    int numTokens = tokenize(copy, tokens, 8);
    if (numTokens < 1) return false;

    memset(&out, 0, sizeof(out));
    out.type = MsgType::UNKNOWN;

    char typeChar = tokens[0][0];

    switch (typeChar) {
        case 'H': // HELLO — H:<robot_id>
            out.type = MsgType::HELLO;
            if (numTokens >= 2) {
                strncpy(out.robot_id, tokens[1], sizeof(out.robot_id) - 1);
            }
            return true;

        case 'M': // MOVE — M:<dir>:<dist>:<speed>:<policy>
            if (numTokens < 5) return false;
            out.type = MsgType::MOVE;
            out.direction       = parseDirection(tokens[1]);
            out.distance_mm     = (uint16_t)atoi(tokens[2]);
            out.speed           = parseSpeed(tokens[3]);
            out.correctionPolicy = parsePolicy(tokens[4]);
            return (out.direction != MoveDirection::INVALID &&
                    out.speed != SpeedLevel::INVALID &&
                    out.correctionPolicy != CorrectionPolicy::INVALID);

        case 'C': // CAM — C:<timestamp>:<x>:<y>
            if (numTokens < 4) return false;
            out.type = MsgType::CAM;
            out.cam_timestamp = (uint32_t)strtoul(tokens[1], NULL, 10);
            out.cam_x = atof(tokens[2]);
            out.cam_y = atof(tokens[3]);
            return true;

        case 'P': // PING — P:<timestamp>
            if (numTokens < 2) return false;
            out.type = MsgType::PING;
            out.ping_timestamp = (uint32_t)strtoul(tokens[1], NULL, 10);
            return true;

        case 'Q': // PONG — Q:<orig_timestamp>
            if (numTokens < 2) return false;
            out.type = MsgType::PONG;
            out.ping_timestamp = (uint32_t)strtoul(tokens[1], NULL, 10);
            return true;

        case 'A': // ABORT
            out.type = MsgType::ABORT;
            return true;

        case 'R': // REGISTER — R:<robot_id>:<caps>
            if (numTokens < 2) return false;
            out.type = MsgType::REGISTER;
            strncpy(out.robot_id, tokens[1], sizeof(out.robot_id) - 1);
            return true;

        default:
            return false;
    }
}

// ============================================================================
// Message building
// ============================================================================

int buildHelloMessage(char* buf, int maxLen, const char* robotId) {
    return snprintf(buf, maxLen, "H:%s", robotId);
}

int buildPongMessage(char* buf, int maxLen, uint32_t origTimestamp) {
    return snprintf(buf, maxLen, "Q:%lu", (unsigned long)origTimestamp);
}

int buildStatusMessage(char* buf, int maxLen,
                       float x, float y, int queueLen, float driftMm) {
    return snprintf(buf, maxLen, "S:%.1f:%.1f:%d:%.1f", x, y, queueLen, driftMm);
}

int buildRegisterMessage(char* buf, int maxLen,
                         const char* robotId, const char* capabilities) {
    return snprintf(buf, maxLen, "R:%s:%s", robotId, capabilities);
}
