/*
 * Quad encoder support functions (via Pulse Counter module)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 * 2025/07
 * License: BSD-2
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/pulse_cnt.h"
#include "hal/pcnt_ll.h"

#define ENCODER_PULSE_PER_REV (1*11*200)

#define ENCODER_MAX_LIMIT 100000
#define ENCODER_MIN_LIMIT -100000

esp_err_t rotary_encoder_init(void);

esp_err_t get_encoder_position(int32_t *pos);

esp_err_t set_encoder_position(int32_t pos);

esp_err_t encoder_start(void);

esp_err_t encoder_stop(void);

esp_err_t encoder_reset(void);
