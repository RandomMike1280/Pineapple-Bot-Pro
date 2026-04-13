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
#include "auto_discovery.h"
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
float currentOmega = 0; // deg/s

// ============================================================================
// Remote Control Definitions
// ============================================================================
JoyStickButton leftJoy;
JoyStickButton rightJoy;
DPadButton DPad;
DPadButton lastDPad;
geoPadButton GeoPad;
geoPadButton lastGeoPad;
shoulderButton Shoulder;
shoulderButton lastShoulder;

int mspeed[4]       = {0};
double mspeedf[4]   = {0.};
int mkickstart[4]   = {KICKSTART_FRAMES, KICKSTART_FRAMES, KICKSTART_FRAMES, KICKSTART_FRAMES};

PS2X remote;

// Servo thresholds
#define GRABBA_ANGLE    45
#define SLIDER_UP       40
#define SLIDER_DOWN     87
#define ARM_UP          0
#define ARM_DOWN        70

void getRemoteState(PS2X &remote) {
    remote.read_gamepad();
    leftJoy.pressed = remote.Button(PSB_L3);
    leftJoy.X = remote.Analog(PSS_LX) - 128;
    leftJoy.Y = 127 - remote.Analog(PSS_LY);
    rightJoy.pressed = remote.Button(PSB_R3);
    rightJoy.X = remote.Analog(PSS_RX) - 128;
    rightJoy.Y = 127 - remote.Analog(PSS_RY);
    
    if (leftJoy.X > 0) leftJoy.X = map(leftJoy.X, 0, 127, 0, 100);
    else leftJoy.X = map(leftJoy.X, -128, 0, -100, 0);
    
    if (leftJoy.Y > 0) leftJoy.Y = map(leftJoy.Y, 0, 127, 0, 100);
    else leftJoy.Y = map(leftJoy.Y, -128, 0, -100, 0);
    
    if (rightJoy.X > 0) rightJoy.X = map(rightJoy.X, 0, 127, 0, 100);
    else rightJoy.X = map(rightJoy.X, -128, 0, -100, 0);
    
    if (rightJoy.Y > 0) rightJoy.Y = map(rightJoy.Y, 0, 127, 0, 100);
    else rightJoy.Y = map(rightJoy.Y, -128, 0, -100, 0);
    
    if (abs(rightJoy.X) < joyThres) rightJoy.X = 0;
    if (abs(rightJoy.Y) < joyThres) rightJoy.Y = 0;
    if (abs(leftJoy.X) < joyThres) leftJoy.X = 0;
    if (abs(leftJoy.Y) < joyThres) leftJoy.Y = 0;
    
    DPad.up = remote.Button(PSB_PAD_UP);
    DPad.down = remote.Button(PSB_PAD_DOWN);
    DPad.left = remote.Button(PSB_PAD_LEFT);
    DPad.right = remote.Button(PSB_PAD_RIGHT);
    
    GeoPad.triangle = remote.Button(PSB_TRIANGLE);
    GeoPad.circle = remote.Button(PSB_CIRCLE);
    GeoPad.cross = remote.Button(PSB_CROSS);
    GeoPad.square = remote.Button(PSB_SQUARE);
    
    Shoulder.L1 = remote.Button(PSB_L1);
    Shoulder.L2 = remote.Button(PSB_L2);
    Shoulder.R1 = remote.Button(PSB_R1);
    Shoulder.R2 = remote.Button(PSB_R2);
}

int sign(double x) {
    return (x > 0) - (x < 0);
}

int speed_to_motor_duty(double speed) {
    if (speed > 0) {
        return 43.0174134490 * speed + 57.16498038405947;
    } else if (speed < 0) {
        return 43.0174134490 * speed - 57.16498038405947;
    }
    return 0;
}

// ============================================================================
// Hybrid applyMotors (Manual override & Physics modeling)
// ============================================================================
void applyMotors() {
    float V = 0, H = 0, A = 0;
    
    float joy_V = verticalVelocity * vRate;
    float joy_H = horizontalVelocity * hRate;
    float joy_A = angularVelocity * angularRate;
    
    // Check manual override
    if (abs(joy_V) > 0.01f || abs(joy_H) > 0.01f || abs(joy_A) > 0.01f) {
        motionQueue.abort(); // Cancel autonomous path explicitly
        V = joy_V;
        H = joy_H;
        A = joy_A;
    } else {
        // Autonomous execution
        float vx, vy, omega;
        motionQueue.getCurrentVelocity(vx, vy, omega);
        
        if (vx == 0 && vy == 0 && omega == 0) {
            V = H = A = 0;
        } else {
            // Apply speed scaling base on level profile (SLOW/NORMAL/FAST relative to FAST max)
            const MotionSegment* seg = motionQueue.currentSegment();
            float speed_scale = seg ? (seg->speed_mm_s / SPEED_FAST_MM_S) : 0.0f;
            
            V = (vy / SPEED_FAST_MM_S) * speed_scale;
            H = (vx / SPEED_FAST_MM_S) * speed_scale;
            
            // Normalize Rotation
            if (omega != 0) {
                // Invert Math CCW (+) to Mechanical CCW (-) for mecanums
                A = -(omega / 86.4f) * speed_scale;
            } else {
                // Passive translation drift correction
                float cx, cy, c_angle;
                deadReckoning.getCurrentPosition(cx, cy, c_angle);
                float target_angle = seg ? seg->target_angle : c_angle;
                float angle_err = target_angle - c_angle;
                while (angle_err > 180.0f) angle_err -= 360.0f;
                while (angle_err < -180.0f) angle_err += 360.0f;
                
                float Kp_angle = 1.0f / 86.4f; // Scale proportional gain back to normalized A
                A = -Kp_angle * angle_err;
                if (A > 0.3f) A = 0.3f;
                if (A < -0.3f) A = -0.3f; // Limit drift correction 
            }
        }
    }

    if (V == 0 && H == 0 && A == 0) {
        // Halt physical motion fully, allowing kickstarts to reset
        for (int i = 0; i < 4; i++) mspeed[i] = 0;
#ifndef TEST_MODE
        Motor1.Stop();
        Motor2.Stop();
        Motor3.Stop();
        Motor4.Stop();
#endif
        currentVx = currentVy = currentOmega = 0;
        return;
    }

    // Kinematics matching Motor bindings [0=FR, 1=BR, 2=FL, 3=BL]
    mspeedf[0] = static_cast<double>(V - H - A); // Motor 1 - Front Right
    mspeedf[1] = static_cast<double>(V + H - A); // Motor 2 - Back Right
    mspeedf[2] = static_cast<double>(V + H + A); // Motor 3 - Front Left
    mspeedf[3] = static_cast<double>(V - H + A); // Motor 4 - Back Left
    
    // Scale fractions through physics lookup table
    for (int i = 0; i < 4; i++) {
        if (abs(mspeedf[i]) > 0) { 
            mspeedf[i] = speed_to_motor_duty(mspeedf[i]);
        }
    }

    // Safely Cap to [-100, 100] without distorting vectors mathematically
    double maxSpeed = 0;
    for (int i = 0; i < 4; i++) {
        maxSpeed = max(maxSpeed, abs(mspeedf[i]));
    }
    if (maxSpeed > MAX_SPEED) {
        for (int i = 0; i < 4; i++) {
            mspeedf[i] = mspeedf[i] / maxSpeed * MAX_SPEED;
        }
    }

    // Frame-based 70% Kickstart
    for (int i = 0; i < 4; i++) {
        if (abs(mspeedf[i]) < MOTOR_STOP_THRESHOLD) {
            mkickstart[i] = KICKSTART_FRAMES;
        }
        if (mkickstart[i] > 0 && abs(mspeedf[i]) >= MOTOR_STOP_THRESHOLD) {
            mkickstart[i]--;
            mspeedf[i] = sign(mspeedf[i]) * max((double)KICKSTART_SPEED, abs(mspeedf[i]));
        }
    }

    for (int i = 0; i < 4; i++) {
        mspeed[i] = static_cast<int>(mspeedf[i]);
    }

    // Plumb physical approximation back to dead-reckoning engine:
    currentVx = H * SPEED_FAST_MM_S;
    currentVy = V * SPEED_FAST_MM_S;
    currentOmega = -A * 86.4f;

#ifdef TEST_MODE
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime >= 500) {
        Serial.printf("[TEST MODE] Virtual Motors: M1=%d, M2=%d, M3=%d, M4=%d\n", mspeed[0], mspeed[1], mspeed[2], mspeed[3]);
        lastPrintTime = millis();
    }
#else
    Motor1.Run(mspeed[0]);
    Motor2.Run(mspeed[1]);
    Motor3.Run(mspeed[2]);
    Motor4.Run(mspeed[3]);
#endif
}

// ============================================================================
// Handle incoming UDP messages
// ============================================================================
void handleIncomingUdp() {
    int packetSize = udp.parsePacket();
    if (!packetSize) return;

    // Automatically latch the phone's IP since the phone controls the robot.
    // This allows the robot to send PONGs and status back without hardcoding the IP.
    phoneIP = udp.remoteIP();

    char buffer[256];
    int len = udp.read(buffer, sizeof(buffer) - 1);
    if (len <= 0) return;
    buffer[len] = '\0';

#ifdef TEST_MODE
    Serial.printf("[TEST MODE] Received UDP Packet: %s\n", buffer);
#endif

    UdpMessage msg;
    if (!parseUdpMessage(buffer, len, msg)) {
        Serial.printf("[UDP] Failed to parse: %s\n", buffer);
        return;
    }

    switch (msg.type) {
        case MsgType::MOVE: {
            // Get current estimated position for segment target computation
            float cx, cy, c_angle;
            deadReckoning.getCurrentPosition(cx, cy, c_angle);

            bool ok = motionQueue.enqueue(
                msg.direction, msg.distance_mm,
                msg.speed, msg.correctionPolicy,
                cx, cy, c_angle
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

        case MsgType::WAYPOINT: {
            float cx, cy, c_angle;
            deadReckoning.getCurrentPosition(cx, cy, c_angle);

            bool ok = motionQueue.enqueueWaypoint(
                msg.target_x, msg.target_y,
                msg.speed, msg.correctionPolicy,
                cx, cy, c_angle
            );
            if (ok) {
                Serial.printf("[CMD] WAYPOINT queued: target=(%.1f, %.1f) spd=%d policy=%d (queue=%d)\n",
                    msg.target_x, msg.target_y, (int)msg.speed,
                    (int)msg.correctionPolicy, motionQueue.remaining());
            } else {
                Serial.println("[CMD] WAYPOINT rejected — queue full!");
            }
            break;
        }

        case MsgType::ROTATE: {
            float cx, cy, c_angle;
            deadReckoning.getCurrentPosition(cx, cy, c_angle);

            bool ok = motionQueue.enqueueRotate(
                msg.target_angle, msg.speed, msg.correctionPolicy,
                cx, cy, c_angle
            );
            if (ok) {
                Serial.printf("[CMD] ROTATE queued: target=%.1f spd=%d policy=%d (queue=%d)\n",
                    msg.target_angle, (int)msg.speed,
                    (int)msg.correctionPolicy, motionQueue.remaining());
            } else {
                Serial.println("[CMD] ROTATE rejected — queue full!");
            }
            break;
        }

        case MsgType::CAM: {
            latencyComp.onCameraUpdate(
                msg.cam_timestamp, msg.cam_x, msg.cam_y, msg.cam_angle,
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
#ifndef TEST_MODE
            Motor1.Stop();
            Motor2.Stop();
            Motor3.Stop();
            Motor4.Stop();
#endif
            currentVx = currentVy = currentOmega = 0;
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
    float x, y, angle;
    deadReckoning.getCurrentPosition(x, y, angle);

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

// Auto-discovery is now handled by auto_discovery.cpp

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
    Serial.begin(921600);
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
    
    // Ensure we actually got an IP assigned before continuing
    while (WiFi.localIP()[0] == 0) {
        delay(100);
    }
    
    Serial.printf("  Robot IP:  %s\n", WiFi.localIP().toString().c_str());
    phoneIP = WiFi.gatewayIP();
    
    // Ensure Gateway IP is valid
    while (phoneIP[0] == 0) {
        delay(100);
        phoneIP = WiFi.gatewayIP();
    }
    Serial.printf("  Phone IP:  %s\n", phoneIP.toString().c_str());

    udp.begin(udpPort);

    // --- Initialize subsystems ---
    deadReckoning.reset(0, 0, 0);

    Motor1.Reverse();
    Motor2.Reverse();
    
    // Initialize Manual control connection
    byte error = remote.config_gamepad(PS2_CLK, PS2_CMD, PS2_CS, PS2_DAT);
    if (error) {
        ledcSetup(7, 10, 12); // Channel 7, 10 kHz, 12-bit resolution
        ledcAttachPin(LED, 7);
        ledcWrite(7, 2048); // Turn on LED to indicate error
        
        unsigned long failInit = millis();
        while (error) {
            error = remote.config_gamepad(PS2_CLK, PS2_CMD, PS2_CS, PS2_DAT);
            if (millis() - failInit > 3000) break; // Bypass if 3 seconds past, allowing UDP to live
        }
        ledcWrite(7, 0); // Turn off LED
        ledcDetachPin(LED);
    }
    
    servo1.write(GRABBA_ANGLE);
    servo2.write(SLIDER_DOWN);
    servo3.write(ARM_UP);

    motionQueue.setSpeedCalibration(SPEED_SLOW_MM_S, SPEED_NORMAL_MM_S, SPEED_FAST_MM_S);

    latencyComp.init(&deadReckoning, &motionQueue);
    latencyComp.setThresholds(DRIFT_THRESHOLD_MM, EMERGENCY_THRESHOLD_MM);

    // --- Start auto-discovery broadcast ---
    setupAutoDiscovery(udp, udpPort, BOT_ID);

#ifdef TEST_MODE
    Serial.println("\n*** TEST MODE ENABLED ***");
    Serial.println("Motors are disabled. Commands are printed to Serial.");
    ledcSetup(7, 10, 12); // Channel 7, 10 kHz, 12-bit resolution
    ledcAttachPin(LED, 7);
#endif

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

    // ---- 2. Handle Joystick Servos ----
    getRemoteState(remote);
    
    if (pressedButton(DPad.left, lastDPad.left)) servo1.write(GRABBA_ANGLE * 2);
    if (pressedButton(DPad.right, lastDPad.right)) servo1.write(0);
    if (pressedButton(GeoPad.triangle, lastGeoPad.triangle)) servo1.write(GRABBA_ANGLE);
    
    if (pressedButton(Shoulder.L1, lastShoulder.L1)) servo2.write(SLIDER_UP);
    if (pressedButton(Shoulder.L2, lastShoulder.L2)) servo2.write(SLIDER_DOWN);
    if (pressedButton(Shoulder.R1, lastShoulder.R1)) servo3.write(ARM_DOWN);
    if (pressedButton(Shoulder.R2, lastShoulder.R2)) servo3.write(ARM_UP);
    
    // ---- 3. Tick the motion queue ----
    bool moving = motionQueue.tick(dt);

    if (motionQueue.segmentJustCompleted) {
        CorrectionPolicy prevPolicy = motionQueue.getActivePolicy();
        float errX, errY, errAngle;
        motionQueue.getDeferredCorrection(errX, errY, errAngle);
        float errMag = sqrtf(errX * errX + errY * errY);
        if (errMag > 2.0f || abs(errAngle) > 2.0f) {
            Serial.printf("[MQ] Deferred correction: %.1f mm / %.1f deg\n", errMag, errAngle);
            deadReckoning.applyCorrection(errX, errY, errAngle, CORRECTION_BLEND_MS);
        }
    }

    // ---- 4. Handle emergency deceleration ----
    if (latencyComp.wasEmergencyTriggered() && moving) {
#ifndef TEST_MODE
        Motor1.Stop();
        Motor2.Stop();
        Motor3.Stop();
        Motor4.Stop();
#else
        Serial.println("[TEST MODE] Virtual emergency stop pause");
#endif
        delay(50);
    }

    // ---- 5. Apply motor speeds (Hybrid loop) ----
    applyMotors();
    
    // Cache controller state for next frame
    leftJoy.lastPressed = leftJoy.pressed;
    rightJoy.lastPressed = rightJoy.pressed;
    lastDPad = DPad;
    lastGeoPad = GeoPad;
    lastShoulder = Shoulder;

    // ---- 6. Update dead-reckoning ----
    deadReckoning.update(currentVx, currentVy, currentOmega, dt);

    // ---- 6. Periodic broadcasts ----
    if (now - lastStatusTime >= STATUS_INTERVAL_MS) {
        sendStatus();
        lastStatusTime = now;
    }

    tickAutoDiscovery(udp, udpPort, BOT_ID);

    if (now - lastPingTime >= PING_INTERVAL_MS) {
        sendPing();
        lastPingTime = now;
    }

#ifdef TEST_MODE
    static uint32_t lastLedBlinkTime = 0;
    static bool ledState = false;
    // Blink LED every 500ms
    if (now - lastLedBlinkTime >= 500) {
        lastLedBlinkTime = now;
        ledState = !ledState;
        ledcWrite(7, ledState ? 2048 : 0);
    }
#endif
}