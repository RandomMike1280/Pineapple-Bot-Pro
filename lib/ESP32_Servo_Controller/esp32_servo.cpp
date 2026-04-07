#include <esp32_servo.hpp>

ESP32Servo::ESP32Servo(int pin, int channel) : _pin(pin), _channel(channel) {}
void ESP32Servo::init() {
    ledcSetup(_channel, 50, 12); // 50 Hz for servos, 12-bit resolution
    ledcAttachPin(_pin, _channel);
}

void ESP32Servo::write(int angle) {
    if (!_initialized) {
        init();
        _initialized = 1;
    }
    ledcWrite(_channel, map(constrain(angle, 0, 180), 0, 180, 102, 512));
}

ESP32Servo servo1(15, 4); // Pin 15, Channel 4
ESP32Servo servo2(14, 5);  // Pin 14, Channel 5
ESP32Servo servo3(13, 6);  // Pin 13, Channel 6
ESP32Servo servo4(12, 7);  // Pin 12, Channel 7