#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "clock.h"
#include "encoder.h"

#define ENCODER_PIN_PWR 10
#define ENCODER_PIN_A   12
#define ENCODER_PIN_B   11

#define ENCODER_4COUNTS_PER_PULSE 0

#define POWER_ACTIVE_LEVEL 1
#define INVALID_WATCH_POINT ((int)-1)

static const char *TAG = "pcnt";

static const
pcnt_unit_config_t unitConfig = {
    .high_limit = 10000,
    .low_limit = -10000,
    .flags = {
        .accum_count = true,
    },
};

static const
pcnt_glitch_filter_config_t filterConfig = {
    .max_glitch_ns = 10000,
};

static const
pcnt_chan_config_t chanConfigA = {
    .edge_gpio_num = ENCODER_PIN_A,
    .level_gpio_num = ENCODER_PIN_B,
};

static const
pcnt_chan_config_t chanConfigB = {
    .edge_gpio_num = ENCODER_PIN_B,
    .level_gpio_num = ENCODER_PIN_A,
};


/****
*   Pulse counter can not be set to an arbitrary value at startup.
*   It can only be reset to 0.
*   Therefore, we need to save the current encoder position to the NVM.
*   After reboot this position becomes the offset to be added to the value of the counter.
*   This way we can keep track of the motor position between power cycles.
*   This module does not have access to the NVM.
*   Calling side must use set_encoder_position() to set the correct offset.
****/
static int32_t startupOffset = 0;

static int32_t accumulator = 0;
static int32_t previous = 0;

static pcnt_unit_handle_t pcntUnit;
static pcnt_channel_handle_t pcntChannelA;
static pcnt_channel_handle_t pcntChannelB;

esp_err_t rotary_encoder_init(void)
{
    ESP_LOGI(TAG, "install pcnt unit");
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unitConfig, &pcntUnit), TAG, "PCNT new unit failed");

    ESP_LOGI(TAG, "set glitch filter");
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(pcntUnit, &filterConfig), TAG, "PCNT glitch filter failed");

    ESP_LOGI(TAG, "install pcnt channels");
    ESP_RETURN_ON_ERROR(pcnt_new_channel(pcntUnit, &chanConfigA, &pcntChannelA), TAG, "PCNT channel A failed");
    ESP_RETURN_ON_ERROR(pcnt_new_channel(pcntUnit, &chanConfigB, &pcntChannelB), TAG, "PCNT channel B failed");

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels %u", sizeof(int));

#if ENCODER_4COUNTS_PER_PULSE
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(pcntChannelA, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE), TAG, "PCNT edge action failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(pcntChannelA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE), TAG, "PCNT level action failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(pcntChannelB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE), TAG, "PCNT edge action failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(pcntChannelB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE), TAG, "PCNT level action failed");
#else /* 1 count per pulse */
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(pcntChannelA, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD), TAG, "PCNT edge action failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(pcntChannelA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE), TAG, "PCNT level action failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(pcntChannelB, PCNT_CHANNEL_EDGE_ACTION_HOLD, PCNT_CHANNEL_EDGE_ACTION_HOLD), TAG, "PCNT edge action failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(pcntChannelB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE), TAG, "PCNT level action failed");
#endif

    ESP_RETURN_ON_ERROR(gpio_set_direction(ENCODER_PIN_PWR, GPIO_MODE_OUTPUT), TAG, "GPIO direction failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(ENCODER_PIN_PWR, !POWER_ACTIVE_LEVEL), TAG, "GPIO set level failed");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(pcntUnit), TAG, "PCNT unit enable failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(pcntUnit), TAG, "PCNT clear count failed");
    return ESP_OK;
}

esp_err_t get_encoder_position(int32_t *pos)
{
    int value = 0;
    ESP_RETURN_ON_ERROR(pcnt_unit_get_count(pcntUnit, &value), TAG, "Reading counter failed (get)");
    if ((value >= 0) && (previous - value > unitConfig.high_limit / 2)) {
        accumulator += unitConfig.high_limit;
    }

    if ((value <= 0) && (previous - value < unitConfig.low_limit / 2)) {
        accumulator += unitConfig.low_limit;
    }

    previous = value;
    *pos = accumulator + value - startupOffset;
    return ESP_OK;
}

esp_err_t set_encoder_position(int32_t pos)
{
    int value = 0;
    if (ESP_OK != pcnt_unit_get_count(pcntUnit, &value))
        value = 0;

    startupOffset = accumulator + value - pos;
    return ESP_OK;
}


esp_err_t encoder_start(void)
{
    ESP_RETURN_ON_ERROR(gpio_set_level(ENCODER_PIN_PWR, POWER_ACTIVE_LEVEL), TAG, "GPIO set level 1 failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(pcnt_unit_start(pcntUnit), TAG, "PCNT unit start failed");

    return ESP_OK;
}

esp_err_t encoder_stop(void)
{
    ESP_RETURN_ON_ERROR(pcnt_unit_stop(pcntUnit), TAG, "PCNT unit stop failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(gpio_set_level(ENCODER_PIN_PWR, !POWER_ACTIVE_LEVEL), TAG, "GPIO set level 0 failed");
    return ESP_OK;
}

esp_err_t encoder_reset(void)
{
    startupOffset = 0;
    return pcnt_unit_clear_count(pcntUnit);
}
