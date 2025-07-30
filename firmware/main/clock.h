/*
 * General purpose clock functions (using gptimer module)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 * 2025/07
 * License: BSD-2
 */

#pragma once

#include "driver/gptimer.h"

esp_err_t init_timer(gptimer_handle_t *timer_ptr, uint32_t timer_resolution);

uint64_t gpclock(gptimer_handle_t timer);

