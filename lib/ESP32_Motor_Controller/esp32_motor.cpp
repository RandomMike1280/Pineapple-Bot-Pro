#include <esp32_motor.hpp>

// ===== Class Implementation =====
esp32_motor::esp32_motor(int _pinA, int _pinB,
                         mcpwm_unit_t _unit, mcpwm_timer_t _timer,
                         uint32_t _freq)
{
    pinA = (gpio_num_t)_pinA;
    pinB = (gpio_num_t)_pinB;
    unit = _unit;
    timer = _timer;
    freq = _freq;

    sigA = (timer == MCPWM_TIMER_0) ? MCPWM0A :
           (timer == MCPWM_TIMER_1) ? MCPWM1A : MCPWM2A;
    sigB = (timer == MCPWM_TIMER_0) ? MCPWM0B :
           (timer == MCPWM_TIMER_1) ? MCPWM1B : MCPWM2B;
}

void esp32_motor::init()
{
    mcpwm_gpio_init(unit, sigA, pinA);
    mcpwm_gpio_init(unit, sigB, pinB);

    mcpwm_config_t config;
    config.frequency = freq;
    config.cmpr_a = 0;
    config.cmpr_b = 0;
    config.counter_mode = MCPWM_UP_COUNTER;
    config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(unit, timer, &config);
}

void esp32_motor::Run(int duty)
{
    if (!initialized) init();
    if (duty > 100) duty = 100;
    if (duty < -100) duty = -100;

    int dutyA = 0, dutyB = 0;

    if (duty > 0) {
        dutyA = duty;
        dutyB = 0;
    } else if (duty < 0) {
        dutyA = 0;
        dutyB = -duty;
    } else {
        Stop();
        return;
    }

    if (reversed) {
        mcpwm_set_duty(unit, timer, MCPWM_OPR_A, dutyB);
        mcpwm_set_duty(unit, timer, MCPWM_OPR_B, dutyA);
    } else {
        mcpwm_set_duty(unit, timer, MCPWM_OPR_A, dutyA);
        mcpwm_set_duty(unit, timer, MCPWM_OPR_B, dutyB);
    }
}

void esp32_motor::Stop()
{
    if (!initialized) init();
    mcpwm_set_duty(unit, timer, MCPWM_OPR_A, 0);
    mcpwm_set_duty(unit, timer, MCPWM_OPR_B, 0);
}

void esp32_motor::Brake()
{
    if (!initialized) init();
    mcpwm_set_duty(unit, timer, MCPWM_OPR_A, 100);
    mcpwm_set_duty(unit, timer, MCPWM_OPR_B, 100);
}

void esp32_motor::Reverse()
{
    reversed = !reversed;
}

// ===== Global Motor Objects =====
esp32_motor Motor1(MOTOR1_PIN_A, MOTOR1_PIN_B, MOTOR1_MCPWM_UNIT, MOTOR1_MCPWM_TIMER);
esp32_motor Motor2(MOTOR2_PIN_A, MOTOR2_PIN_B, MOTOR2_MCPWM_UNIT, MOTOR2_MCPWM_TIMER);
esp32_motor Motor3(MOTOR3_PIN_A, MOTOR3_PIN_B, MOTOR3_MCPWM_UNIT, MOTOR3_MCPWM_TIMER);
esp32_motor Motor4(MOTOR4_PIN_A, MOTOR4_PIN_B, MOTOR4_MCPWM_UNIT, MOTOR4_MCPWM_TIMER);
