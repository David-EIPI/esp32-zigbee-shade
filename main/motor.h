#pragma once

#include "esp_err.h"
#include "driver/ledc.h"

#define MOTOR_PWM_RESOLUTION LEDC_TIMER_8_BIT
#define MOTOR_PWM_DUTY_MAX ((1 << MOTOR_PWM_RESOLUTION)-1)

#define MOTOR_DIRECTION_UP 1

/**
 * @brief Initialize the PWM motor driver
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t pwm_motor_init(void);


/**
 * @brief Stop PWM output and wait for the motor to stop
 *
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t pwm_motor_stop(void);


/**
 * @brief Run the motor in specified direction and speed
 *
 * @param direction 1 = forward, -1 = reverse, 0 = stop
 * @param speed Duty cycle, 0-255 (0=stop, 255=full speed)
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t pwm_motor_run(int direction, unsigned int speed);


/**
 * @brief Report motor run status
 *
 * @return True if motor is currently running (PWM is generated on one of the control pins)
 */
bool pwm_motor_running(void);