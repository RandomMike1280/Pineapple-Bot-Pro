#ifndef SERVO_CONTROL_HPP
#define SERVO_CONTROL_HPP

#include <udp_protocol.hpp>

/**
 * Initialize servos and set them to their default states:
 * - Grabber: Middle
 * - Slider: Down
 * - Arm: Up
 */
void initServoControl();

/**
 * Execute a specific servo action.
 * This function handles sequences and delays.
 */
void executeServoAction(ServoAction action);

#endif // SERVO_CONTROL_HPP
