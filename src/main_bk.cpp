// #include <main.h>

// // #define BOT1
// #define BOT2

// // #ifdef BOT1
// // // Bot I
// // #define HOLD_THRESHOLD 155
// // #define RELEASE_THRESHOLD 50
// // #define PICKUP_THRESHOLD 60
// // #define DROP_THRESHOLD 30
// // #define ang0 10
// // #define ang1 130
// // #define ang2 160
// // #endif

// //#ifdef BOT2
// // Bot II
// //#define HOLD_THRESHOLD 145
// //#define RELEASE_THRESHOLD 50
// //#define PICKUP_THRESHOLD 50
// //#define DROP_THRESHOLD 25
// //#define ang0 180
// //#define ang1 97
// //#define ang2 55
// //#endif

// // #ifdef BOT2
// // // Bot II
// // #define HOLD_THRESHOLD 110
// // #define RELEASE_THRESHOLD 65
// // #define PICKUP_THRESHOLD 50
// // #define DROP_THRESHOLD 25
// // #define ang0 10
// // #define ang1 163
// // #define ang2 170
// // #endif

// #ifdef BOT1
// // Bot II
// #define HOLD_THRESHOLD 130
// #define RELEASE_THRESHOLD 55
// #define PICKUP_THRESHOLD 55
// #define DROP_THRESHOLD 30
// #define ang0 10
// #define ang1 153
// #define ang2 162
// #endif

// #ifdef BOT2
// // Bot II
// #define HOLD_THRESHOLD 147
// #define RELEASE_THRESHOLD 55
// #define PICKUP_THRESHOLD 55
// #define DROP_THRESHOLD 30
// #define ang0 10
// #define ang1 145
// #define ang2 156
// #endif
// PS2X remote;
// void setup() {
//     Serial.begin(921600);
//     byte error = remote.config_gamepad(PS2_CLK, PS2_CMD, PS2_CS, PS2_DAT);
//     if (error) {
//         ledcSetup(7, 10, 12); // Channel 7, 10 kHz, 12-bit resolution
//         ledcAttachPin(LED, 7);
//         ledcWrite(7, 2048); // Turn on LED to indicate error
//         while (error) error = remote.config_gamepad(PS2_CLK, PS2_CMD, PS2_CS, PS2_DAT);
//         ledcWrite(7, 0); // Turn off LED after successful connection
//         ledcDetachPin(LED);
//     }
//     Motor1.Reverse();
//     Motor2.Reverse();
//     servo1.write(RELEASE_THRESHOLD);
//     servo4.write(PICKUP_THRESHOLD);
// }

// uint8_t servo3angle = 10;
// void loop() {
//     getRemoteState(remote);
//     if (pressedButton(Shoulder.L1, lastShoulder.L1)) {
//         servo1.write(HOLD_THRESHOLD);
//     }
//     if (pressedButton(Shoulder.L2, lastShoulder.L2)) {
//         servo1.write(RELEASE_THRESHOLD);
//     }
//     if (pressedButton(Shoulder.R1, lastShoulder.R1)) {
//         servo4.write(PICKUP_THRESHOLD);
//     }
//     if (pressedButton(Shoulder.R2, lastShoulder.R2)) {
//         servo4.write(DROP_THRESHOLD);
//     }
//     if (pressedButton(DPad.right, lastDPad.right)) {
//         servo3angle = ang0;
//     }
//     if (pressedButton(DPad.up, lastDPad.up)) {
//         servo3angle = ang1;
//     }
//     if (pressedButton(DPad.left, lastDPad.left)) {
//         servo3angle = ang2;
//     }

//     // SETUP
//     if (pressedButton(GeoPad.cross, lastGeoPad.cross)) {
//         servo1.write(0);
//     }

//     servo3.write(servo3angle);
//     calculateMotorSpeeds();
//     Motor1.Run(m1speed);
//     Motor2.Run(m2speed);
//     Motor3.Run(m3speed);
//     Motor4.Run(m4speed);
//     leftJoy.lastPressed = leftJoy.pressed;
//     rightJoy.lastPressed = rightJoy.pressed;
//     lastDPad = DPad;
//     lastGeoPad = GeoPad;
//     lastShoulder = Shoulder;
//     // Serial.print(V);
// // Serial.print("\t");
//     // Serial.print(H);
//     // Serial.print("\t");
//     // Serial.println(A);
//     delay(1);
// }