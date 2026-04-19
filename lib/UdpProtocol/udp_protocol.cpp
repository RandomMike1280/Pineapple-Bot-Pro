#include "udp_protocol.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// ============================================================================
// Helper: tokenize by ':'
// ============================================================================

static int tokenize(char *str, char **tokens, int maxTokens) {
  int count = 0;
  char *tok = strtok(str, ":");
  while (tok != NULL && count < maxTokens) {
    tokens[count++] = tok;
    tok = strtok(NULL, ":");
  }
  return count;
}

// ============================================================================
// Direction / Speed / Policy string parsing
// ============================================================================

MoveDirection parseDirection(const char *str) {
  if (strcmp(str, "up") == 0 || strcmp(str, "U") == 0)
    return MoveDirection::UP;
  if (strcmp(str, "down") == 0 || strcmp(str, "D") == 0)
    return MoveDirection::DOWN;
  if (strcmp(str, "left") == 0 || strcmp(str, "L") == 0)
    return MoveDirection::LEFT;
  if (strcmp(str, "right") == 0 || strcmp(str, "R") == 0)
    return MoveDirection::RIGHT;
  return MoveDirection::INVALID;
}

RotationDirection parseRotationDirection(const char *str) {
  if (strcmp(str, "cw") == 0 || strcmp(str, "CW") == 0)
    return RotationDirection::CW;
  if (strcmp(str, "ccw") == 0 || strcmp(str, "CCW") == 0)
    return RotationDirection::CCW;
  return RotationDirection::INVALID;
}

SpeedLevel parseSpeed(const char *str) {
  if (strcmp(str, "slow") == 0 || strcmp(str, "S") == 0)
    return SpeedLevel::SLOW;
  if (strcmp(str, "normal") == 0 || strcmp(str, "N") == 0)
    return SpeedLevel::NORMAL;
  if (strcmp(str, "fast") == 0 || strcmp(str, "F") == 0)
    return SpeedLevel::FAST;
  return SpeedLevel::INVALID;
}

CorrectionPolicy parsePolicy(const char *str) {
  if (strcmp(str, "live") == 0 || strcmp(str, "L") == 0)
    return CorrectionPolicy::LIVE;
  if (strcmp(str, "deferred") == 0 || strcmp(str, "D") == 0)
    return CorrectionPolicy::DEFERRED;
  if (strcmp(str, "none") == 0 || strcmp(str, "N") == 0)
    return CorrectionPolicy::NONE;
  return CorrectionPolicy::INVALID;
}

ServoAction parseServoAction(const char *str) {
  if (strcmp(str, "none") == 0 || strcmp(str, "N") == 0)
    return ServoAction::NONE;
  if (strcmp(str, "lower_left") == 0 || strcmp(str, "LL") == 0)
    return ServoAction::LOWER_LEFT;
  if (strcmp(str, "lower_right") == 0 || strcmp(str, "LR") == 0)
    return ServoAction::LOWER_RIGHT;
  if (strcmp(str, "upper_left") == 0 || strcmp(str, "UL") == 0)
    return ServoAction::UPPER_LEFT;
  if (strcmp(str, "upper_right") == 0 || strcmp(str, "UR") == 0)
    return ServoAction::UPPER_RIGHT;
  if (strcmp(str, "grabber_left") == 0 || strcmp(str, "GL") == 0)
    return ServoAction::GRABBER_LEFT;
  if (strcmp(str, "grabber_right") == 0 || strcmp(str, "GR") == 0)
    return ServoAction::GRABBER_RIGHT;
  if (strcmp(str, "grabber_center") == 0 || strcmp(str, "GC") == 0)
    return ServoAction::GRABBER_CENTER;
  if (strcmp(str, "slider_up") == 0 || strcmp(str, "SU") == 0)
    return ServoAction::SLIDER_UP;
  if (strcmp(str, "slider_down") == 0 || strcmp(str, "SD") == 0)
    return ServoAction::SLIDER_DOWN;
  if (strcmp(str, "arm_down") == 0 || strcmp(str, "AD") == 0)
    return ServoAction::ARM_DOWN;
  if (strcmp(str, "arm_up") == 0 || strcmp(str, "AU") == 0)
    return ServoAction::ARM_UP;
  if (strcmp(str, "fruit") == 0 || strcmp(str, "FRUIT") == 0)
    return ServoAction::FRUIT;
  if (strcmp(str, "dropper_open") == 0 || strcmp(str, "DO") == 0)
    return ServoAction::DROPPER_OPEN;
  if (strcmp(str, "dropper_close") == 0 || strcmp(str, "DC") == 0)
    return ServoAction::DROPPER_CLOSE;
  if (strcmp(str, "ceil_open") == 0 || strcmp(str, "CO") == 0)
    return ServoAction::CEIL_OPEN;
  if (strcmp(str, "ceil_close") == 0 || strcmp(str, "CC") == 0)
    return ServoAction::CEIL_CLOSE;
  if (strcmp(str, "ceil_water") == 0 || strcmp(str, "CW") == 0)
    return ServoAction::CEIL_WATER;
  return ServoAction::INVALID;
}

void directionToVector(MoveDirection dir, float &vx, float &vy) {
  vx = 0.0f;
  vy = 0.0f;
  switch (dir) {
  case MoveDirection::UP:
    vy = 1.0f;
    break;
  case MoveDirection::DOWN:
    vy = -1.0f;
    break;
  case MoveDirection::LEFT:
    vx = -1.0f;
    break;
  case MoveDirection::RIGHT:
    vx = 1.0f;
    break;
  default:
    break;
  }
}

// ============================================================================
// Message parsing
// ============================================================================

bool parseUdpMessage(const char *buffer, int len, UdpMessage &out) {
  if (len <= 0)
    return false;

  // Work on a copy because strtok mutates
  char copy[256];
  int copyLen = (len < 255) ? len : 255;
  memcpy(copy, buffer, copyLen);
  copy[copyLen] = '\0';

  // Remove trailing whitespace/newlines
  while (copyLen > 0 &&
         (copy[copyLen - 1] == '\n' || copy[copyLen - 1] == '\r' ||
          copy[copyLen - 1] == ' ')) {
    copy[--copyLen] = '\0';
  }

  char *tokens[8];
  int numTokens = tokenize(copy, tokens, 8);
  if (numTokens < 1)
    return false;

  memset(&out, 0, sizeof(out));
  out.type = MsgType::UNKNOWN;
  out.servoAction = ServoAction::NONE;

  char typeChar = tokens[0][0];

  switch (typeChar) {
  case 'H': // HELLO — H:<robot_id>
    out.type = MsgType::HELLO;
    if (numTokens >= 2) {
      strncpy(out.robot_id, tokens[1], sizeof(out.robot_id) - 1);
    }
    return true;

  case 'M': // MOVE — M:<dir>:<dist>:<speed>:<policy>[:<action>]
    if (numTokens < 5)
      return false;
    out.type = MsgType::MOVE;
    out.direction = parseDirection(tokens[1]);
    out.distance_mm = (uint16_t)atoi(tokens[2]);
    out.speed = parseSpeed(tokens[3]);
    out.correctionPolicy = parsePolicy(tokens[4]);
    if (numTokens >= 6) {
      out.servoAction = parseServoAction(tokens[5]);
    }
    return (out.direction != MoveDirection::INVALID &&
            out.speed != SpeedLevel::INVALID &&
            out.correctionPolicy != CorrectionPolicy::INVALID &&
            out.servoAction != ServoAction::INVALID);

  case 'W': { // WAYPOINT — W:<x>:<y>[:<angle>]:<speed>:<policy>
    if (numTokens < 5) return false;
    out.type = MsgType::WAYPOINT;
    float px = atof(tokens[1]);
    float py = atof(tokens[2]);
    out.target_x = py; // Swapped for robot-frame
    out.target_y = px; // Swapped for robot-frame
    if (numTokens >= 6) {
      // Full form: W:<x>:<y>:<angle>:<speed>:<policy>[:<action>]
      out.target_angle = atof(tokens[3]);
      out.speed = parseSpeed(tokens[4]);
      out.correctionPolicy = parsePolicy(tokens[5]);
      if (numTokens >= 7) {
        out.servoAction = parseServoAction(tokens[6]);
      }
    } else {
      // Short form: W:<x>:<y>:<speed>:<policy>  (angle defaults to 0)
      out.target_angle = 0.0f;
      out.speed = parseSpeed(tokens[3]);
      out.correctionPolicy = parsePolicy(tokens[4]);
    }
    return (out.speed != SpeedLevel::INVALID &&
            out.correctionPolicy != CorrectionPolicy::INVALID &&
            out.servoAction != ServoAction::INVALID);
  }

  case 'T': // ROTATE — T:<target_angle>:<speed>:<policy>
    if (numTokens < 4)
      return false;
    out.type = MsgType::ROTATE;
    out.target_angle = atof(tokens[1]);
    out.speed = parseSpeed(tokens[2]);
    out.correctionPolicy = parsePolicy(tokens[3]);
    return (out.speed != SpeedLevel::INVALID &&
            out.correctionPolicy != CorrectionPolicy::INVALID);

  case 'C': { // CAM — C:<timestamp>:<x>:<y>[:<angle>]
    if (numTokens < 4)
      return false;
    out.type = MsgType::CAM;
    out.cam_timestamp = (uint32_t)strtoull(tokens[1], NULL, 10);
    float cx = atof(tokens[2]);
    float cy = atof(tokens[3]);
    out.cam_x = cy;
    out.cam_y = cx;
    out.cam_angle = (numTokens >= 5) ? atof(tokens[4]) : NAN;
    return true;
  }

  case 'P': // PING — P:<timestamp>
    if (numTokens < 2)
      return false;
    out.type = MsgType::PING;
    out.ping_timestamp = (uint32_t)strtoull(tokens[1], NULL, 10);
    return true;

  case 'Q': // PONG — Q:<orig_timestamp>[:<remote_timestamp>]
    if (numTokens < 2)
      return false;
    out.type = MsgType::PONG;
    out.ping_timestamp = (uint32_t)strtoull(tokens[1], NULL, 10);
    out.remote_timestamp = (numTokens >= 3) ? (uint32_t)strtoull(tokens[2], NULL, 10) : 0;
    return true;

  case 'D': // MOVE_DURATION — D:<dir>:<duration_ms>:<speed>:<policy>
    if (numTokens < 5)
      return false;
    out.type = MsgType::MOVE_DURATION;
    out.direction = parseDirection(tokens[1]);
    out.duration_ms = (uint32_t)strtoull(tokens[2], NULL, 10);
    out.speed = parseSpeed(tokens[3]);
    out.correctionPolicy = parsePolicy(tokens[4]);
    return (out.direction != MoveDirection::INVALID &&
            out.speed != SpeedLevel::INVALID &&
            out.correctionPolicy != CorrectionPolicy::INVALID);

  case 'O': // ROTATE_DURATION — O:<cw|ccw>:<duration_ms>:<speed>:<policy>
    if (numTokens < 5)
      return false;
    out.type = MsgType::ROTATE_DURATION;
    out.rotationDirection = parseRotationDirection(tokens[1]);
    out.duration_ms = (uint32_t)strtoull(tokens[2], NULL, 10);
    out.speed = parseSpeed(tokens[3]);
    out.correctionPolicy = parsePolicy(tokens[4]);
    return (out.rotationDirection != RotationDirection::INVALID &&
            out.speed != SpeedLevel::INVALID &&
            out.correctionPolicy != CorrectionPolicy::INVALID);

  case 'V': { // VELOCITY — V:<vx>:<vy>:<omega>:<timeout_ms>
    if (numTokens < 5) return false;
    out.type = MsgType::VELOCITY;
    float pvx = atof(tokens[1]);
    float pvy = atof(tokens[2]);
    out.vel_vx = pvy;   // phone Y → ESP32 X (coordinate swap)
    out.vel_vy = pvx;   // phone X → ESP32 Y
    out.vel_omega = atof(tokens[3]);
    out.duration_ms = (uint32_t)strtoull(tokens[4], NULL, 10);
    return true;
  }

  case 'A': // ABORT
    out.type = MsgType::ABORT;
    return true;

  case 'E': // SERVO EXEC — E:<action>
    if (numTokens < 2) return false;
    out.type = MsgType::SERVO_EXEC;
    out.servoAction = parseServoAction(tokens[1]);
    return (out.servoAction != ServoAction::INVALID);

  case 'R': // REGISTER — R:<robot_id>:<caps>
    if (numTokens < 2)
      return false;
    out.type = MsgType::REGISTER;
    strncpy(out.robot_id, tokens[1], sizeof(out.robot_id) - 1);
    return true;

  case 'X': // DONE — X[:<status>]
    out.type = MsgType::DONE;
    return true;
    
  case 'U': // SET_SERIAL_MONITOR — U:<0|1>
    if (numTokens < 2) return false;
    out.type = MsgType::SET_SERIAL_MONITOR;
    out.duration_ms = (uint32_t)atoi(tokens[1]); // Reusing duration_ms for 0/1 flag
    return true;

  default:
    return false;
  }
}

// ============================================================================
// Message building
// ============================================================================

int buildHelloMessage(char *buf, int maxLen, const char *robotId) {
  return snprintf(buf, maxLen, "H:%s", robotId);
}

int buildPongMessage(char *buf, int maxLen, uint32_t origTimestamp) {
  return snprintf(buf, maxLen, "Q:%lu", (unsigned long)origTimestamp);
}

int buildStatusMessage(char *buf, int maxLen, float x, float y, float angle, int queueLen,
                       float driftMm) {
    // Because the phone is rotated on it's side while in portrait mode,
    // The phone's XY coordinate system directly translates to YX on the robot's coordinate system.
    // We thereby swap and negate X and Y to resolve this conflict.
    return snprintf(buf, maxLen, "S:%.1f:%.1f:%.1f:%d:%.1f", y, x, angle, queueLen, driftMm);
}

int buildRegisterMessage(char *buf, int maxLen, const char *robotId,
                         const char *capabilities) {
  return snprintf(buf, maxLen, "R:%s:%s", robotId, capabilities);
}

int buildLogMessage(char *buf, int maxLen, char type, const char *logId,
                    const char *msg) {
  if (logId && logId[0] != '\0') {
    return snprintf(buf, maxLen, "L:%c:%s:%s", type, logId, msg);
  } else {
    return snprintf(buf, maxLen, "L:%c:%s", type, msg);
  }
}

void directionToVector(MoveDirection dir, float &vx, float &vy) {
  vx = 0.0f;
  vy = 0.0f;
  switch (dir) {
  case MoveDirection::UP:
    vy = 1.0f;
    break;
  case MoveDirection::DOWN:
    vy = -1.0f;
    break;
  case MoveDirection::LEFT:
    vx = -1.0f;
    break;
  case MoveDirection::RIGHT:
    vx = 1.0f;
    break;
  default:
    break;
  }
}
