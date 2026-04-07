// ============================================================================
// Dead-Reckoning Robot Control Firmware
// ============================================================================
// ESP32-S3 firmware for a mecanum-wheeled robot that:
//   1. Accepts move commands over UDP (direction, distance, speed, policy)
//   2. Executes them via a motion queue (back-to-back, no stops)
//   3. Maintains a dead-reckoning position estimate (time × speed)
//   4. Receives camera observations from the phone and applies
//      latency-compensated corrections (smooth, no jumps)
//
// The same firmware runs on both robots — only BOT_ID in main.h differs.
// ============================================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include "main.h"
#include <dead_reckoning.hpp>
#include <motion_queue.hpp>
#include <latency_compensator.hpp>
#include <udp_protocol.hpp>

// ============================================================================
// WiFi Configuration
// ============================================================================
const char* ssid     = "IPhone 19 Professional";
const char* password = "sixseven";

// ============================================================================
// Subsystem instances
// ============================================================================
WiFiUDP          udp;
const int        udpPort = 4210;
IPAddress        phoneIP;

DeadReckoning    deadReckoning;
MotionQueue      motionQueue;
LatencyCompensator latencyComp;

// ============================================================================
// Timing state
// ============================================================================
uint32_t lastTick       = 0;
uint32_t lastHelloTime  = 0;
uint32_t lastStatusTime = 0;
uint32_t lastPingTime   = 0;

// ============================================================================
// Current motor velocities (for dead-reckoning integration)
// ============================================================================
float currentVx = 0;  // mm/s
float currentVy = 0;  // mm/s

// Speed level → motor duty% mapping
int getMotorDuty(SpeedLevel level) {
    switch (level) {
        case SpeedLevel::SLOW:   return DUTY_SLOW;
        case SpeedLevel::NORMAL: return DUTY_NORMAL;
        case SpeedLevel::FAST:   return DUTY_FAST;
        default:                 return DUTY_NORMAL;
    }
}

// ============================================================================
// Mecanum kinematics implementation
// ============================================================================
MecanumSpeeds computeMecanumSpeeds(int V, int H, int A) {
    MecanumSpeeds s;
    s.m1 = V - H - A;  // Front Right
    s.m2 = V + H - A;  // Back Right
    s.m3 = V - H + A;  // Back Left
    s.m4 = V + H + A;  // Front Left

    // Normalize to [-100, 100] if any motor exceeds range
    int maxSpeed = max(max(abs(s.m1), abs(s.m2)), max(abs(s.m3), abs(s.m4)));
    if (maxSpeed > 100) {
        s.m1 = (s.m1 * 100) / maxSpeed;
        s.m2 = (s.m2 * 100) / maxSpeed;
        s.m3 = (s.m3 * 100) / maxSpeed;
        s.m4 = (s.m4 * 100) / maxSpeed;
    }
    return s;
}

// ============================================================================
// Apply motor speeds from current motion queue state
// ============================================================================
void applyMotors() {
    float vx, vy;
    motionQueue.getCurrentVelocity(vx, vy);
    currentVx = vx;
    currentVy = vy;

    if (vx == 0 && vy == 0) {
        // No active segment — stop motors
        Motor1.Stop();
        Motor2.Stop();
        Motor3.Stop();
        Motor4.Stop();
        return;
    }

    // Determine duty cycle from the active segment's speed level
    const MotionSegment* seg = motionQueue.currentSegment();
    int duty = seg ? getMotorDuty(seg->speed) : DUTY_NORMAL;

    // Convert velocity direction to mecanum V/H components
    // vx > 0 = right = strafe, vy > 0 = forward
    int V = 0, H = 0;
    if (vy > 0)       V =  duty;
    else if (vy < 0)  V = -duty;
    if (vx > 0)       H =  duty;
    else if (vx < 0)  H = -duty;

    MecanumSpeeds s = computeMecanumSpeeds(V, H, 0);
    Motor1.Run(s.m1);
    Motor2.Run(s.m2);
    Motor3.Run(s.m3);
    Motor4.Run(s.m4);
}

// ============================================================================
// Handle incoming UDP messages
// ============================================================================
void handleIncomingUdp() {
    int packetSize = udp.parsePacket();
    if (!packetSize) return;

    char buffer[256];
    int len = udp.read(buffer, sizeof(buffer) - 1);
    if (len <= 0) return;
    buffer[len] = '\0';

    UdpMessage msg;
    if (!parseUdpMessage(buffer, len, msg)) {
        Serial.printf("[UDP] Failed to parse: %s\n", buffer);
        return;
    }

    switch (msg.type) {
        case MsgType::MOVE: {
            // Get current estimated position for segment target computation
            float cx, cy;
            deadReckoning.getCurrentPosition(cx, cy);

            bool ok = motionQueue.enqueue(
                msg.direction, msg.distance_mm,
                msg.speed, msg.correctionPolicy,
                cx, cy
            );
            if (ok) {
                Serial.printf("[CMD] MOVE queued: dir=%d dist=%d spd=%d policy=%d (queue=%d)\n",
                    (int)msg.direction, msg.distance_mm, (int)msg.speed,
                    (int)msg.correctionPolicy, motionQueue.remaining());
            } else {
                Serial.println("[CMD] MOVE rejected — queue full!");
            }
            break;
        }

        case MsgType::CAM: {
            latencyComp.onCameraUpdate(
                msg.cam_timestamp, msg.cam_x, msg.cam_y,
                CORRECTION_BLEND_MS
            );
            break;
        }

        case MsgType::PING: {
            // Respond immediately with PONG
            char pongBuf[32];
            int pongLen = buildPongMessage(pongBuf, sizeof(pongBuf), msg.ping_timestamp);
            udp.beginPacket(udp.remoteIP(), udpPort);
            udp.write((uint8_t*)pongBuf, pongLen);
            udp.endPacket();
            break;
        }

        case MsgType::PONG: {
            latencyComp.onPong(msg.ping_timestamp);
            Serial.printf("[RTT] %lu ms\n", (unsigned long)latencyComp.getRttMs());
            break;
        }

        case MsgType::ABORT: {
            Serial.println("[CMD] ABORT — emergency stop!");
            motionQueue.abort();
            Motor1.Stop();
            Motor2.Stop();
            Motor3.Stop();
            Motor4.Stop();
            currentVx = currentVy = 0;
            break;
        }

        default:
            Serial.printf("[UDP] Unhandled message type: %d\n", (int)msg.type);
            break;
    }
}

// ============================================================================
// Send telemetry status to phone (every STATUS_INTERVAL_MS)
// ============================================================================
void sendStatus() {
    float x, y;
    deadReckoning.getCurrentPosition(x, y);

    char buf[64];
    int len = buildStatusMessage(buf, sizeof(buf),
        x, y,
        motionQueue.remaining(),
        latencyComp.getLastDriftMagnitude()
    );

    udp.beginPacket(phoneIP, udpPort);
    udp.write((uint8_t*)buf, len);
    udp.endPacket();
}

// ============================================================================
// Send HELLO beacon for auto-discovery (every HELLO_INTERVAL_MS)
// ============================================================================
void sendHello() {
    char buf[32];
    int len = buildHelloMessage(buf, sizeof(buf), BOT_ID);
    udp.beginPacket(phoneIP, udpPort);
    udp.write((uint8_t*)buf, len);
    udp.endPacket();
}

// ============================================================================
// Send PING for RTT measurement (every PING_INTERVAL_MS)
// ============================================================================
void sendPing() {
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "P:%lu", (unsigned long)millis());
    udp.beginPacket(phoneIP, udpPort);
    udp.write((uint8_t*)buf, len);
    udp.endPacket();
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.printf("  Dead-Reckoning Robot [%s]\n", BOT_ID);
    Serial.println("========================================\n");

    // --- WiFi ---
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to hotspot");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected!");
    Serial.printf("  Robot IP:  %s\n", WiFi.localIP().toString().c_str());
    phoneIP = WiFi.gatewayIP();
    Serial.printf("  Phone IP:  %s\n", phoneIP.toString().c_str());

    udp.begin(udpPort);

    // --- Initialize subsystems ---
    deadReckoning.reset(0, 0);

    motionQueue.setSpeedCalibration(SPEED_SLOW_MM_S, SPEED_NORMAL_MM_S, SPEED_FAST_MM_S);

    latencyComp.init(&deadReckoning, &motionQueue);
    latencyComp.setThresholds(DRIFT_THRESHOLD_MM, EMERGENCY_THRESHOLD_MM);

    // --- Send registration to phone ---
    char regBuf[64];
    int regLen = buildRegisterMessage(regBuf, sizeof(regBuf), BOT_ID, "mecanum4wd");
    udp.beginPacket(phoneIP, udpPort);
    udp.write((uint8_t*)regBuf, regLen);
    udp.endPacket();

    Serial.println("\nReady. Waiting for commands...\n");
    lastTick = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
    uint32_t now = millis();
    uint32_t dt  = now - lastTick;

    // Target ~10ms control loop period
    if (dt < CONTROL_LOOP_INTERVAL_MS) return;
    lastTick = now;

    // ---- 1. Handle incoming UDP messages ----
    // Process all available packets this tick
    for (int i = 0; i < 5; i++) {
        handleIncomingUdp();
    }

    // ---- 2. Tick the motion queue ----
    bool moving = motionQueue.tick(dt);

    // If a segment just completed with deferred correction, apply it now
    if (motionQueue.segmentJustCompleted) {
        CorrectionPolicy prevPolicy = motionQueue.getActivePolicy();
        // The just-completed segment's correction is stored
        float errX, errY;
        motionQueue.getDeferredCorrection(errX, errY);
        float errMag = sqrtf(errX * errX + errY * errY);
        if (errMag > 2.0f) {
            Serial.printf("[MQ] Deferred correction: %.1f mm\n", errMag);
            deadReckoning.applyCorrection(errX, errY, CORRECTION_BLEND_MS);
        }
    }

    // ---- 3. Handle emergency deceleration ----
    if (latencyComp.wasEmergencyTriggered() && moving) {
        // Briefly stop motors for correction to take effect
        Motor1.Stop();
        Motor2.Stop();
        Motor3.Stop();
        Motor4.Stop();
        delay(50);  // 50ms pause — short enough to be barely noticeable
    }

    // ---- 4. Apply motor speeds ----
    applyMotors();

    // ---- 5. Update dead-reckoning ----
    deadReckoning.update(currentVx, currentVy, dt);

    // ---- 6. Periodic broadcasts ----
    if (now - lastStatusTime >= STATUS_INTERVAL_MS) {
        sendStatus();
        lastStatusTime = now;
    }

    if (now - lastHelloTime >= HELLO_INTERVAL_MS) {
        sendHello();
        lastHelloTime = now;
    }

    if (now - lastPingTime >= PING_INTERVAL_MS) {
        sendPing();
        lastPingTime = now;
    }
}