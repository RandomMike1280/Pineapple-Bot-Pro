#include "servo_control.hpp"
#include <esp32_servo.hpp>
#include <Arduino.h>

#define VAL_GRABBER_LEFT     9 + (45 * 2)
#define VAL_GRABBER_RIGHT    9 + 0
#define VAL_GRABBER_CENTER   9 + 45
#define VAL_SLIDER_UP        40
#define VAL_SLIDER_DOWN      87
#define VAL_ARM_UP           0
#define VAL_ARM_DOWN         90

// Private helper functions to interact with the global servo objects from esp32_servo.hpp
void _servo_action_grabber_left() {
    servo1.write(VAL_GRABBER_LEFT);
}

void _servo_action_grabber_right() {
    servo1.write(VAL_GRABBER_RIGHT);
}

void _servo_action_grabber_center() {
    servo1.write(VAL_GRABBER_CENTER);
}

void _servo_action_slider_up() {
    servo2.write(VAL_SLIDER_UP);
}

void _servo_action_slider_down() {
    servo2.write(VAL_SLIDER_DOWN);
}

void _servo_action_arm_down() {
    servo3.write(VAL_ARM_DOWN);
}

void _servo_action_arm_up() {
    servo3.write(VAL_ARM_UP);
}

void initServoControl() {
    // Initialize the global servos (defined in esp32_servo.hpp/cpp)
    servo1.init();
    servo2.init();
    servo3.init();
    servo4.init();

    // Set initial states as requested by the user:
    // - Grabber: Middle
    // - Slider: Down
    // - Arm: Up
    _servo_action_grabber_center();
    _servo_action_slider_down();
    _servo_action_arm_up();
}

void executeServoAction(ServoAction action) {
    switch (action) {
        case ServoAction::LOWER_LEFT:
            _servo_action_grabber_left();
            delay(200);
            _servo_action_grabber_center();
            delay(50);
            break;

        case ServoAction::LOWER_RIGHT:
            _servo_action_grabber_right();
            delay(200);
            _servo_action_grabber_center();
            delay(50);
            break;

        case ServoAction::UPPER_LEFT:
            _servo_action_slider_down();
            delay(200);
            _servo_action_arm_down();
            delay(100); // Wait for arm/slider
            _servo_action_slider_up();
            _servo_action_arm_up();
            delay(50);      
            break;

        case ServoAction::UPPER_RIGHT:
            _servo_action_slider_up();
            delay(100);
            _servo_action_arm_down();
            delay(100); // Wait for arm/slider
            _servo_action_arm_up();
            delay(50); // Return
            break;

        case ServoAction::GRABBER_LEFT:
            _servo_action_grabber_left();
            break;
        case ServoAction::GRABBER_RIGHT:
            _servo_action_grabber_right();
            break;
        case ServoAction::GRABBER_CENTER:
            _servo_action_grabber_center();
            break;
        case ServoAction::SLIDER_UP:
            _servo_action_slider_up();
            break;
        case ServoAction::SLIDER_DOWN:
            _servo_action_slider_down();
            break;
        case ServoAction::ARM_DOWN:
            _servo_action_arm_down();
            break;
        case ServoAction::ARM_UP:
            _servo_action_arm_up();
            break;

        case ServoAction::NONE:
        default:
            break;
    }
}
