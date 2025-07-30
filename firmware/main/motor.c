/*
 * Zigbee Roller Shade Example
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 * 2025/07
 * License: BSD-2
 */

#include "driver/uart.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "esp_system.h"
#include "motor.h"

#define MOTOR_PIN_A 0
#define MOTOR_PIN_B 1

/* Level to be used for stopping the motor: 1 = active braking, 0 = free coasting */
#define PIN_IDLE_LEVEL 1

#if PIN_IDLE_LEVEL
#define ZERO_SPEED_DUTY MOTOR_PWM_DUTY_MAX
#else
#define ZERO_SPEED_DUTY 0
#endif



/* Timer configuration */
static const
ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0,
    .duty_resolution = MOTOR_PWM_RESOLUTION,
    .freq_hz = 24000,
    .clk_cfg = LEDC_AUTO_CLK
};

/* LEDC channel configuration */
static const
ledc_channel_config_t channel_conf = {
    .gpio_num = MOTOR_PIN_A,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0,
    .flags.output_invert = PIN_IDLE_LEVEL != 0,
};


/* Current direction. */
static int currentDirection = MOTOR_DIRECTION_UP;

static int running = 0;

esp_err_t pwm_motor_init(void)
{
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_conf), "pwm_motor", "Timer config failed");

/* Configure PWM output channel (only one pin will use PWM at a time) */
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_conf), "pwm_motor", "Channel config failed");

/* Configure pin directions */
    ESP_RETURN_ON_ERROR(gpio_set_direction(MOTOR_PIN_A, GPIO_MODE_OUTPUT), "pwm_motor", "GPIO direction failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(MOTOR_PIN_A, PIN_IDLE_LEVEL), "pwm_motor", "GPIO set level failed");
    ESP_RETURN_ON_ERROR(gpio_set_direction(MOTOR_PIN_B, GPIO_MODE_OUTPUT), "pwm_motor", "GPIO direction failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(MOTOR_PIN_B, PIN_IDLE_LEVEL), "pwm_motor", "GPIO set level failed");
    return ESP_OK;
}


static esp_err_t pwm_motor_fade_to(uint32_t target_duty, int fade_time_ms)
{
    if (fade_time_ms < 150)
        fade_time_ms = 150;

    if (fade_time_ms > 1000)
        fade_time_ms = 1000;

    ESP_RETURN_ON_ERROR(ledc_fade_func_install(0), "pwm_motor", "Fade func install failed");

    ESP_RETURN_ON_ERROR(ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, target_duty, fade_time_ms), "pwm_motor", "Set fade with time failed");

    ESP_RETURN_ON_ERROR(ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_WAIT_DONE), "pwm_motor", "Set fade start failed");

    ledc_fade_func_uninstall();

    return ESP_OK;
}

esp_err_t pwm_motor_stop(void)
{
    esp_err_t res = ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, PIN_IDLE_LEVEL);

    uint32_t duty = ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    if (PIN_IDLE_LEVEL)
        duty = MOTOR_PWM_DUTY_MAX - duty;

    pwm_motor_fade_to(ZERO_SPEED_DUTY, duty * 4);

/* Delay to make sure the motor has stopped when this function returns */
    vTaskDelay((duty * 4 + 50) / portTICK_PERIOD_MS);

    running = 0;
    return res;
}

esp_err_t pwm_motor_run(int direction, unsigned int speed)
{
    if (0 == speed) {
        return pwm_motor_stop();
    }

    unsigned int duty = speed;
/* Invert PWM duty setting if idle level is 1. Future IDF versions may fix this. */
    if (PIN_IDLE_LEVEL)
        duty = MOTOR_PWM_DUTY_MAX - duty;

    if (duty > MOTOR_PWM_DUTY_MAX) duty = MOTOR_PWM_DUTY_MAX;

/* If changing direction, first stop the motor */
    if (currentDirection == !direction) {
        ESP_RETURN_ON_ERROR(pwm_motor_stop(), "pwm_motor", "Stopping failed");
    }

    int pwm_pin, high_pin;
    if (MOTOR_DIRECTION_UP == !!direction) { // Upwards: pinB = 1, pinA = PWM
        pwm_pin = MOTOR_PIN_A;
        high_pin = MOTOR_PIN_B;
    } else { // Reverse: pinA = 1, pinB = PWM
        pwm_pin = MOTOR_PIN_B;
        high_pin = MOTOR_PIN_A;
    }



/* Select the appropriate pin for PWM output */
    ESP_RETURN_ON_ERROR(ledc_set_pin(pwm_pin, LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0), "pwm_motor", "Set PWM pin failed");

/* Set the other pin to idle level */
    ESP_RETURN_ON_ERROR(gpio_set_direction(high_pin, GPIO_MODE_OUTPUT), "pwm_motor", "GPIO direction failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(high_pin, PIN_IDLE_LEVEL), "pwm_motor", "Set idle pin failed");

/* Activate the channel */
#if 0
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty), "pwm_motor", "Set duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0), "pwm_motor", "Update duty failed");
    vTaskDelay(100 / portTICK_PERIOD_MS);
#else
/* Set duty to zero speed to avoid jerking start */
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, ZERO_SPEED_DUTY), "pwm_motor", "Set duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0), "pwm_motor", "Update duty failed");
/* Fade into target duty with constant acceleration (4 duty steps/ms) */
    pwm_motor_fade_to(duty, speed * 4);
#endif
    ESP_LOGI("Motor", "Motor is running. Speed %u, direction: %d", speed, direction);

    currentDirection = !!direction;
    running = 1;
    return ESP_OK;
}

bool pwm_motor_running(void)
{
    return running != 0;
}
