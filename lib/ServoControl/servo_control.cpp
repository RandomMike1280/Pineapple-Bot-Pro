#include "servo_control.hpp"
#include <esp32_servo.hpp>
#include <Arduino.h>

#define GRABBER_ANGLE    45
#define SLIDER_UP        40
#define SLIDER_DOWN      87
#define ARM_UP           0
#define ARM_DOWN         70

// Private helper functions to interact with the global servo objects from esp32_servo.hpp
void _servo_action_grabber_left() {
    servo1.write(GRABBER_ANGLE * 2);
}

void _servo_action_grabber_right() {
    servo1.write(0);
}

void _servo_action_grabber_center() {
    servo1.write(GRABBER_ANGLE);
}

void _servo_action_slider_up() {
    servo2.write(SLIDER_UP);
}

void _servo_action_slider_down() {
    servo2.write(SLIDER_DOWN);
}

void _servo_action_arm_down() {
    servo3.write(ARM_DOWN);
}

void _servo_action_arm_up() {
    servo3.write(ARM_UP);
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
            _servo_action_slider_up();
            _servo_action_arm_down();
            delay(400); // Wait for arm/slider
            _servo_action_grabber_left();
            delay(200);
            _servo_action_grabber_center();
            delay(50);
            _servo_action_arm_up();
            _servo_action_slider_down();
            delay(400); // Return
            break;

        case ServoAction::UPPER_RIGHT:
            _servo_action_slider_up();
            _servo_action_arm_down();
            delay(400); // Wait for arm/slider
            _servo_action_grabber_right();
            delay(200);
            _servo_action_grabber_center();
            delay(50);
            _servo_action_arm_up();
            _servo_action_slider_down();
            delay(400); // Return
            break;

        case ServoAction::NONE:
        default:
            break;
    }
}
