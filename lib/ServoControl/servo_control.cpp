#include "servo_control.hpp"
#include <esp32_servo.hpp>
#include <Arduino.h>

// ============================================================================
// Robot A Servo Actions
// ============================================================================
#ifdef IS_BOT_A

#define VAL_GRABBER_LEFT     9 + (45 * 2)
#define VAL_GRABBER_RIGHT    9 + 0
#define VAL_GRABBER_CENTER   9 + 45
#define VAL_SLIDER_UP        40
#define VAL_SLIDER_DOWN      87
#define VAL_ARM_UP           0
#define VAL_ARM_DOWN         90

void _servo_action_grabber_left() { servo1.write(VAL_GRABBER_LEFT); }
void _servo_action_grabber_right() { servo1.write(VAL_GRABBER_RIGHT); }
void _servo_action_grabber_center() { servo1.write(VAL_GRABBER_CENTER); }
void _servo_action_slider_up() { servo2.write(VAL_SLIDER_UP); }
void _servo_action_slider_down() { servo2.write(VAL_SLIDER_DOWN); }
void _servo_action_arm_down() { servo3.write(VAL_ARM_DOWN); }
void _servo_action_arm_up() { servo3.write(VAL_ARM_UP); }

#endif

// ============================================================================
// Robot B Servo Actions
// ============================================================================
#ifdef IS_BOT_B

#define DROPPER_OPEN    27
#define DROPPER_CLOSE   54
#define CEIL_OPEN       65
#define CEIL_CLOSE      85
#define CEIL_WATER      105

void _servo_action_dropper_open() { servo1.write(DROPPER_OPEN); }
void _servo_action_dropper_close() { servo1.write(DROPPER_CLOSE); }
void _servo_action_ceil_open() { servo3.write(CEIL_OPEN); }
void _servo_action_ceil_close() { servo3.write(CEIL_CLOSE); }
void _servo_action_ceil_water() { servo3.write(CEIL_WATER); }

#endif

void initServoControl() {
    servo1.init();
    servo2.init();
    servo3.init();
    servo4.init();

#ifdef IS_BOT_A
    _servo_action_grabber_center();
    _servo_action_slider_down();
    _servo_action_arm_up();
#endif
#ifdef IS_BOT_B
    _servo_action_dropper_close();
    _servo_action_ceil_close();
#endif
}

void executeServoAction(ServoAction action) {
    switch (action) {
#ifdef IS_BOT_A
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
            delay(100);
            _servo_action_slider_up();
            _servo_action_arm_up();
            delay(50);      
            break;

        case ServoAction::UPPER_RIGHT:
            _servo_action_slider_up();
            delay(100);
            _servo_action_arm_down();
            delay(100);
            _servo_action_arm_up();
            delay(50);
            break;

        case ServoAction::GRABBER_LEFT:   _servo_action_grabber_left();   break;
        case ServoAction::GRABBER_RIGHT:  _servo_action_grabber_right();  break;
        case ServoAction::GRABBER_CENTER: _servo_action_grabber_center(); break;
        case ServoAction::SLIDER_UP:      _servo_action_slider_up();      break;
        case ServoAction::SLIDER_DOWN:    _servo_action_slider_down();    break;
        case ServoAction::ARM_DOWN:       _servo_action_arm_down();       break;
        case ServoAction::ARM_UP:         _servo_action_arm_up();         break;
#endif

#ifdef IS_BOT_B
        case ServoAction::DROPPER_OPEN:   _servo_action_dropper_open();   break;
        case ServoAction::DROPPER_CLOSE:  _servo_action_dropper_close();  break;
        case ServoAction::CEIL_OPEN:      _servo_action_ceil_open();      break;
        case ServoAction::CEIL_CLOSE:     _servo_action_ceil_close();     break;
        case ServoAction::CEIL_WATER:     _servo_action_ceil_water();     break;
#endif

        case ServoAction::FRUIT:
        case ServoAction::NONE:
        default:
            break;
    }
}
