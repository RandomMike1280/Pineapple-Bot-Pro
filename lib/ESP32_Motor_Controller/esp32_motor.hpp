#ifndef _ESP32_MOTOR_H_
#define _ESP32_MOTOR_H_

#include <Arduino.h>
#include "driver/mcpwm.h"

// ==== Motor Pin Mapping ====
#define MOTOR1_PIN_A 35
#define MOTOR1_PIN_B 36
#define MOTOR2_PIN_A 37
#define MOTOR2_PIN_B 38
#define MOTOR3_PIN_A 39
#define MOTOR3_PIN_B 40
#define MOTOR4_PIN_A 41
#define MOTOR4_PIN_B 42

// ==== MCPWM Mapping ====
#define MOTOR1_MCPWM_UNIT MCPWM_UNIT_0
#define MOTOR1_MCPWM_TIMER MCPWM_TIMER_0
#define MOTOR2_MCPWM_UNIT MCPWM_UNIT_0
#define MOTOR2_MCPWM_TIMER MCPWM_TIMER_1
#define MOTOR3_MCPWM_UNIT MCPWM_UNIT_0
#define MOTOR3_MCPWM_TIMER MCPWM_TIMER_2
#define MOTOR4_MCPWM_UNIT MCPWM_UNIT_1
#define MOTOR4_MCPWM_TIMER MCPWM_TIMER_0

// ==== Motor Class ====
class esp32_motor
{
private:
    gpio_num_t pinA, pinB;
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_io_signals_t sigA, sigB;
    uint32_t freq;
    bool reversed = false;
    bool initialized = false;
public:
    esp32_motor(int pinA, int pinB,
                mcpwm_unit_t unit = MCPWM_UNIT_0,
                mcpwm_timer_t timer = MCPWM_TIMER_0,
                uint32_t freq = 24000);

    void init();
    void Run(int duty);   // -100 ~ +100 (%)
    void Stop();          // duty = 0%
    void Brake();         // duty = 100% both sides (short brake)
    void Reverse();       // toggle direction logic
};

// ==== Export global motors ====
extern esp32_motor Motor1;
extern esp32_motor Motor2;
extern esp32_motor Motor3;
extern esp32_motor Motor4;

#endif
