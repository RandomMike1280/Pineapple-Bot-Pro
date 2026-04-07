#ifndef ESP32_SERVO_HPP
#define ESP32_SERVO_HPP

#include <Arduino.h>

class ESP32Servo {
public:
    ESP32Servo(int pin, int channel);
    void init();
    void write(int angle);
private:
    uint8_t _initialized = 0;
    int _pin;
    int _channel;
};

extern ESP32Servo servo1;
extern ESP32Servo servo2;
extern ESP32Servo servo3;
extern ESP32Servo servo4;

#endif // ESP32_SERVO_HPP