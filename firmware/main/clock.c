/*
 * General purpose clock functions (using gptimer module)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 * 2025/07
 * License: BSD-2
 */
#include "esp_check.h"
#include "esp_err.h"
#include "clock.h"

esp_err_t init_timer(gptimer_handle_t *timer_ptr, uint32_t timer_resolution)
{
    const gptimer_config_t config = {
        .resolution_hz = timer_resolution,
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
    };

    ESP_RETURN_ON_ERROR(gptimer_new_timer(&config, timer_ptr), "GPTIMER", "clock");
    ESP_ERROR_CHECK(gptimer_enable(*timer_ptr));
    ESP_ERROR_CHECK(gptimer_start(*timer_ptr));
    return ESP_OK;
}

uint64_t gpclock(gptimer_handle_t timer)
{
    uint64_t result = 0;

    if (ESP_OK != gptimer_get_raw_count(timer, &result))
        result = -1;

    return result;
}
