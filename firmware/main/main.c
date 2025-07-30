/*
 * Zigbee Roller Shade Example
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 * 2025/07
 * License: BSD-2
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_zigbee_core.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "driver/gpio.h"
#include "driver/temperature_sensor.h"
#include "main.h"
#include "clock.h"
#include "encoder.h"
#include "motor.h"
#include "ota.h"


#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif


enum _queue_message_code {
    QM_UNKNOWN = 0,
    QM_FROM_ENCODER,
    QM_ZIGBEE_STOP,
    QM_ZIGBEE_UP,
    QM_ZIGBEE_DOWN,
    QM_ZIGBEE_TO_VALUE,
    QM_ZIGBEE_TO_PERCENTAGE,
    QM_ZIGBEE_SET_UPPER,
    QM_ZIGBEE_SET_LOWER,
    QM_ZIGBEE_RESET_LIMITS,
    QM_ZIGBEE_SET_VELOCITY,
    QM_ZIGBEE_SET_MODE,
};


static const char *ZB_TAG = "APP_ZIGBEE";
static const char *MAIN_TAG = "APP_MAIN";

static const unsigned indication_led_pin = 13;

static const uint32_t timer_resolution = 10 * 1000;
static gptimer_handle_t timer = 0;

TaskHandle_t motorTaskHandle = NULL;

static temperature_sensor_handle_t temp_handle = NULL;
static float temperature_sensor_value = 0;


/* The queue is to be created to hold a maximum of 16 uint32_t  variables. */
#define QUEUE_LENGTH      16
#define QUEUE_ITEM_SIZE   sizeof( uint32_t )

/* The variable used to hold the queue's data structure. */
static StaticQueue_t xStaticQueue;

/* The array to use as the queue's storage area. This must be at least uxQueueLength * uxItemSize bytes. */
static uint8_t ucQueueStorageArea[ QUEUE_LENGTH * QUEUE_ITEM_SIZE ];

/* Main task queue is used to send commands and data to the main task */
QueueHandle_t xMainQueue;

/* Position monitoring. current_pos = last reading from the encoder */
static int32_t currentPosition = 0;
/* Position monitoring. previous_pos = previous reading */
static int previousPosition = 0;
/* Position monitoring. target_pos = requested position.
The motor must spin towards target_pos, otherwise we should stop it. */
static int targetPosition = 0;

/*
    Save position to NVS when it changes by this amount.
    Position is also saved every time the motor stops.
 */
static const int32_t movingSaveDelta = ENCODER_PULSE_PER_REV;
/*
    NVS saving delta when motor is stationary accounts for backlash.
*/
static const int32_t staticSaveDelta = 20;

/* Approximate lift velocity at max motor speed, cm/s */
static const int maxLiftVelocity = 10;

/* Lift velocity requested via Zigbee */
static int32_t zbLiftVelocity = 5;

/* Upper and lower lift limits. Must be calibrated via Zigbee */
static int32_t upperLiftLimit = ENCODER_MAX_LIMIT;
static int32_t lowerLiftLimit = ENCODER_MIN_LIMIT;

/* Fixed upper and lower limits reported to clients */
static int32_t zb_UpperLiftLimit = 180;
static int32_t zb_LowerLiftLimit = 0;

/* User configured coverage limit, % */
static int32_t zb_userPercentageLimit = 100;

/* Inversion flag. Changes percentage assigment and up/down buttons' directions. */
static int zb_Inversion = 0;

/* NVS keys */
const char ZB_STORAGE_NAMESPACE[] = "zb_storage";

static char nvs_UpperLimit[] = "W_UL";
static char nvs_LowerLimit[] = "W_LL";
static char nvs_zbUpperLimit[] = "WZUL";
static char nvs_zbLowerLimit[] = "WZLL";
static char nvs_Velocity[] = "W_Ve";
static char nvs_ModeAttr[] = "W_MA";
static char nvs_CurrentPosition[] = "W_CP";

/********************* Utility functions **************************/

static int encoder_to_zigbee(int position, int zb_min, int zb_max)
{
    if (zb_min == zb_max ||
        upperLiftLimit == lowerLiftLimit) {
        return 0;
    }

    int encmin = lowerLiftLimit;
    int diff = upperLiftLimit - lowerLiftLimit;

/* Allow inversion */
    if (diff < 0) {
        diff = -diff;
        encmin = upperLiftLimit;
    }

    int zb_pos = zb_min + ((position - encmin) * (zb_max - zb_min)) / diff;
    if (zb_pos < zb_min) zb_pos = zb_min;
    else if (zb_pos > zb_max) zb_pos = zb_max;
    return zb_pos;
}

static int encoder_to_zigbee_units(int position)
{
    return encoder_to_zigbee(position, zb_LowerLiftLimit, zb_UpperLiftLimit);
}

static int encoder_to_zigbee_percentage(int position)
{
    uint8_t p = encoder_to_zigbee(position, 0, 100);
    if (!zb_Inversion)
        p = 100 - p;

    return p;
}

static int zigbee_to_encoder(int zb_position, int zb_min, int zb_max)
{
    if (zb_min == zb_max ||
        upperLiftLimit == lowerLiftLimit) {
        return 0;
    }

    int encmin = lowerLiftLimit;
    int diff = upperLiftLimit - lowerLiftLimit;

/* Allow inversion */
    if (diff < 0) {
        diff = -diff;
        encmin = upperLiftLimit;
    }

    int pos = encmin + ((zb_position - zb_min) * diff) / (zb_max - zb_min);
    if (pos > upperLiftLimit) pos = upperLiftLimit;
    else if (pos < lowerLiftLimit) pos = lowerLiftLimit;

    return pos;
}

static int zigbee_units_to_encoder(int zb_position)
{
    return zigbee_to_encoder(zb_position, zb_LowerLiftLimit, zb_UpperLiftLimit);
}

static int zigbee_percentage_to_encoder(int zb_position)
{
    int p = zb_position;
    if (p < 0) p = 0;
    if (p > 100) p = 100;
    if (!zb_Inversion)
        p = 100 - p;

    return zigbee_to_encoder(p, 0, 100);
}


/********************* Zigbee functions **************************/
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

static const int ZB_INIT_FAIL_COUNT_TO_REBOOT = 60;
static const int ZB_ZDO_FAIL_COUNT_TO_REBOOT = 10;
static int zb_fail_count = 0;

static void init_gpio(void)
{
    const gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << indication_led_pin),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };

    gpio_config(&io_conf);
    gpio_set_level(indication_led_pin, 1);
}


void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    uint8_t min_lqi;

    switch (sig_type) {
    case ESP_ZB_NLME_STATUS_INDICATION:
        ESP_LOGI(ZB_TAG, "%s, status: 0x%x\n", esp_zb_zdo_signal_to_string(sig_type), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
        break;
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(ZB_TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        min_lqi = esp_zb_secur_network_min_join_lqi_get();
        ESP_LOGI(ZB_TAG, "Min LQI = %u", (unsigned)min_lqi);
        esp_zb_secur_network_min_join_lqi_set(0);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(ZB_TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(ZB_TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(ZB_TAG, "Device rebooted");
            }

            zb_fail_count = 0;
        } else {
            /* commissioning failed */

            if (ZB_INIT_FAIL_COUNT_TO_REBOOT <= zb_fail_count) {
                ESP_LOGI(ZB_TAG, "ZB init has failed too many times. Restarting.");
                break;
            }

            ESP_LOGW(ZB_TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
            zb_fail_count += 1;
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(ZB_TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(ZB_TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGI(ZB_TAG, "Leave and network steering initiated.");
        esp_zb_factory_reset();
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        break;
    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
        if (ZB_ZDO_FAIL_COUNT_TO_REBOOT <= zb_fail_count) {
            ESP_LOGI(ZB_TAG, "ZDO device unavailable. Restarting.");
            esp_restart();
            break;
        }
        zb_fail_count += 1;
        __attribute__ ((fallthrough));
    default:
        ESP_LOGI(ZB_TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}


/* Setup window covering cluster */
static struct _zb_WindowCoveringAttrs {
    uint8_t windowCoveringType;
    uint16_t currentPosition;
    uint8_t configStatus;
    uint8_t currentPercentage;
    int32_t mode;
} zb_WindowCoveringAttrs = {
    .windowCoveringType = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_ROLLERSHADE_EXTERIOR,
    .currentPosition = 0, //cm
    .configStatus = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_OPERATIONAL     |
        ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_ONLINE                      |
        ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_LIFT_CONTROL_IS_CLOSED_LOOP |
        ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_LIFT_ENCODER_CONTROLLED,
    .currentPercentage = 0,
/*
    ESP32 Zigbee SDK has a bug that considers mode=0 (no flags set) an invalid value.
    Therefore, we assign an unused but harmless flag here to make SDK functions happy and accept this attribute.
*/
    .mode = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_LEDS_WILL_DISPLAY_FEEDBACK,
};

static struct {
    int id;
    void *value_ptr;
    int type;
    int access;
} window_covering_attr_desc[] = {
    { ESP_ZB_ZCL_ATTR_WINDOW_COVERING_WINDOW_COVERING_TYPE_ID      ,       &zb_WindowCoveringAttrs.windowCoveringType, .type = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY},
    { ESP_ZB_ZCL_ATTR_WINDOW_COVERING_PHYSICAL_CLOSED_LIMIT_LIFT_ID,       &zb_UpperLiftLimit,                         .type = ESP_ZB_ZCL_ATTR_TYPE_U16,       .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY},
    { ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_ID,            &zb_WindowCoveringAttrs.currentPosition,    .type = ESP_ZB_ZCL_ATTR_TYPE_U16,       .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING },
    { ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_STATUS_ID,                    &zb_WindowCoveringAttrs.configStatus,       .type = ESP_ZB_ZCL_ATTR_TYPE_8BITMAP,   .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID, &zb_WindowCoveringAttrs.currentPercentage,  .type = ESP_ZB_ZCL_ATTR_TYPE_U8,        .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING | ESP_ZB_ZCL_ATTR_ACCESS_SCENE },
    { ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_OPEN_LIMIT_LIFT_ID,        &zb_UpperLiftLimit,                         .type = ESP_ZB_ZCL_ATTR_TYPE_U16,       .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE },
    { ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_CLOSED_LIMIT_LIFT_ID,      &zb_LowerLiftLimit,                         .type = ESP_ZB_ZCL_ATTR_TYPE_U16,       .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE },
    { ESP_ZB_ZCL_ATTR_WINDOW_COVERING_VELOCITY_ID,                         &zbLiftVelocity,                            .type = ESP_ZB_ZCL_ATTR_TYPE_U16,       .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE },
    { ESP_ZB_ZCL_ATTR_WINDOW_COVERING_MODE_ID,                             &zb_WindowCoveringAttrs.mode,               .type = ESP_ZB_ZCL_ATTR_TYPE_8BITMAP,   .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE },
};

static esp_zb_attribute_list_t * create_window_covering_cluster(void)
{
    zb_Inversion = 0 != (zb_WindowCoveringAttrs.mode & ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_REVERSED_MOTOR_DIRECTION);

    if (zb_Inversion) {
        zb_WindowCoveringAttrs.configStatus |= ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_REVERSE_COMMANDS;
    } else {
        zb_WindowCoveringAttrs.configStatus &= ~ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_REVERSE_COMMANDS;
    }

    esp_zb_attribute_list_t * window_covering_cluster =
        esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING);


    if (window_covering_cluster) {
        unsigned i;
        for (i = 0; i < lengthof(window_covering_attr_desc); i++) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(
                        ((window_covering_attr_desc[i].id == ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_OPEN_LIMIT_LIFT_ID) ||
                         (window_covering_attr_desc[i].id == ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_CLOSED_LIMIT_LIFT_ID)) ?
                    esp_zb_cluster_add_attr(
                        window_covering_cluster,
                        ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                        window_covering_attr_desc[i].id,
                        window_covering_attr_desc[i].type,
                        window_covering_attr_desc[i].access,
                        window_covering_attr_desc[i].value_ptr
                    )
                    :
                    esp_zb_window_covering_cluster_add_attr(
                        window_covering_cluster,
                        window_covering_attr_desc[i].id,
                        window_covering_attr_desc[i].value_ptr
                    )
                );
        }
    }
    return window_covering_cluster;
}


/* Setup Temperature Sensor cluster */
static esp_zb_attribute_list_t * create_temperature_cluster(void)
{
    esp_zb_temperature_meas_cluster_cfg_t cfg = { 0, -10 * 100, 80 * 100 };
    esp_zb_attribute_list_t * temperature_measurement_cluster = esp_zb_temperature_meas_cluster_create(&cfg);

    return temperature_measurement_cluster;
}

/* Analog output clusters - to support user settings */

static struct {
    char     description[16];
    float    min_present;
    float    max_present;
    float    present_value;
    float    resolution;
    uint16_t status_flags;
    uint16_t units;
    uint32_t app_type;
    volatile int32_t *value_ptr;
} analog_output_attr_values[] = {
    {
    .description = "\x0e" "Coverage limit",
    .min_present = 0,
    .max_present = 100,
    .present_value = 100,
    .resolution = 1,
    .status_flags = 0,
    .units = 98, /* 98 = % */
    .app_type = 16 << 16, /* 16 = brightness-percent */
    .value_ptr = &zb_userPercentageLimit,
    },
};

#define ANALOG_ATTR_FIELD_OFFSET(field) (offsetof(typeof(analog_output_attr_values[0]), field))

static struct {
    int id;
    int offs;
    int access;
} analog_attr_offset[] = {
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_DESCRIPTION_ID      , ANALOG_ATTR_FIELD_OFFSET(description),   .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MAX_PRESENT_VALUE_ID, ANALOG_ATTR_FIELD_OFFSET(max_present),   .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MIN_PRESENT_VALUE_ID, ANALOG_ATTR_FIELD_OFFSET(min_present),   .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID    , ANALOG_ATTR_FIELD_OFFSET(present_value), .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_RESOLUTION_ID       , ANALOG_ATTR_FIELD_OFFSET(resolution),    .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_STATUS_FLAGS_ID     , ANALOG_ATTR_FIELD_OFFSET(status_flags),  .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_ENGINEERING_UNITS_ID, ANALOG_ATTR_FIELD_OFFSET(units),         .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY },
    { ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_APPLICATION_TYPE_ID , ANALOG_ATTR_FIELD_OFFSET(app_type),      .access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY }
};

static esp_zb_attribute_list_t * create_analog_cluster(unsigned cl_idx)
{
    esp_zb_attribute_list_t *esp_zb_analog_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT);
    int nattrs = 0;
    if (esp_zb_analog_cluster != NULL) {

        analog_output_attr_values[cl_idx].present_value = *analog_output_attr_values[cl_idx].value_ptr;

        int i;
        for (i = 0; i < lengthof(analog_attr_offset); i++) {
	    esp_err_t err = esp_zb_analog_output_cluster_add_attr(esp_zb_analog_cluster, analog_attr_offset[i].id,
	        ((void*)&analog_output_attr_values[cl_idx]) + analog_attr_offset[i].offs );
	    if (ESP_OK == err)
	        nattrs += 1;
        }
    }

    ESP_LOGI(ZB_TAG, "Analog output cluster %u created. Attrs: %d", cl_idx, nattrs);
    return esp_zb_analog_cluster;
}

/* Setup Basic cluster */
static esp_zb_attribute_list_t * create_basic_cluster(char *manufacturer)
{
    uint8_t zero = 0;
    uint8_t version = 3;

    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);

    if (esp_zb_basic_cluster != NULL) {
        esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer);
        esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER);
        esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &version);
        esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &zero);
    }
    return esp_zb_basic_cluster;
}

esp_zb_zcl_reporting_info_t server_side_reporting_info[] = {

/* Window covering current position */
    {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_CLOSURE_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 5, // Minimum interval of 5 seconds
        .u.send_info.max_interval = 600, // Maximum interval of 10 minutes
        .u.send_info.def_min_interval = 5, // Default minimum interval of 30 seconds
        .u.send_info.def_max_interval = 600, // Default maximum interval of 10 minutes
        .u.send_info.delta.u16 = 1, // Report on every state change
        .attr_id = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC, //0xFFFF,
    },


};

static void setup_reporting(void)
{

    unsigned idx;
/* Config the reporting info  */
    for (idx = 0; idx < lengthof(server_side_reporting_info); idx++) {

/* Attempt to update the reporting info */
        esp_err_t err = esp_zb_zcl_update_reporting_info(&server_side_reporting_info[idx]);
        if (err != ESP_OK) {
            ESP_LOGE(ZB_TAG, "Failed to update reporting info for cluster 0x%x: %s", server_side_reporting_info[idx].cluster_id, esp_err_to_name(err));
        } else {
            ESP_LOGI(ZB_TAG, "Successfully updated reporting info for cluster 0x%x.", server_side_reporting_info[idx].cluster_id);
        }
    }
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, ZB_TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, ZB_TAG, "Received message: error status(%d)",
                        message->info.status);
    unsigned val = 0;
    memcpy(&val, message->attribute.data.value, message->attribute.data.size < sizeof(val) ? message->attribute.data.size : sizeof(val));
    ESP_LOGI(ZB_TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data type(%#x), value(%#x)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.type, val);


/* Process window covering cluster */
    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING
        && message->info.dst_endpoint == HA_CLOSURE_ENDPOINT)
    {
        uint16_t main_task_message_param = 0;
        uint32_t main_task_message = QM_UNKNOWN;

        switch (message->attribute.id) {
            case ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID:
                if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                    main_task_message_param = *(uint8_t *)message->attribute.data.value;
                    main_task_message = QM_ZIGBEE_TO_PERCENTAGE;
                }
                break;

            case ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_OPEN_LIMIT_LIFT_ID:
                if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                    zb_UpperLiftLimit = *(uint16_t *)message->attribute.data.value;
                    main_task_message = QM_ZIGBEE_SET_UPPER;
                }
                break;

            case ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_CLOSED_LIMIT_LIFT_ID:
                if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                    zb_LowerLiftLimit = *(uint16_t *)message->attribute.data.value;
                    main_task_message = QM_ZIGBEE_SET_LOWER;
                }
                break;

            case ESP_ZB_ZCL_ATTR_WINDOW_COVERING_VELOCITY_ID:
                if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                    zbLiftVelocity = *(uint16_t *)message->attribute.data.value;
                    main_task_message = QM_ZIGBEE_SET_VELOCITY;
                }
                break;
            case ESP_ZB_ZCL_ATTR_WINDOW_COVERING_MODE_ID:
                if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BITMAP) {
                    main_task_message = QM_ZIGBEE_SET_MODE;
                    if (ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_REVERSED_MOTOR_DIRECTION & (*(uint8_t *)message->attribute.data.value)) {
                        zb_WindowCoveringAttrs.mode |= ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_REVERSED_MOTOR_DIRECTION;
                        zb_WindowCoveringAttrs.configStatus |= ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_REVERSE_COMMANDS;
                        zb_Inversion = 1;
                    } else {
                        zb_WindowCoveringAttrs.mode &= ~ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_REVERSED_MOTOR_DIRECTION;
                        zb_WindowCoveringAttrs.configStatus &= ~ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_REVERSE_COMMANDS;
                        zb_Inversion = 0;
                    }
                    if (ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_RUN_IN_CALIBRATION_MODE & (*(uint8_t *)message->attribute.data.value)) {
                    /* This is interpreted as a command to reset the limits and position. */
                        main_task_message = QM_ZIGBEE_RESET_LIMITS;
                    }
                    zb_WindowCoveringAttrs.mode |= ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_LEDS_WILL_DISPLAY_FEEDBACK;
                    ESP_LOGI(ZB_TAG, "WindowCovering.Mode = %ld", zb_WindowCoveringAttrs.mode);

		    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_zb_zcl_set_attribute_val(HA_CLOSURE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
		        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_MODE_ID, &zb_WindowCoveringAttrs.mode, false));

		    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_zb_zcl_set_attribute_val(HA_CLOSURE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
		        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_STATUS_ID, &zb_WindowCoveringAttrs.configStatus, false));
                }
                break;
        }

        if (QM_UNKNOWN != main_task_message) {
            uint32_t zigbee_message = ((uint32_t)main_task_message_param << 8) + main_task_message;
            if (xQueueSend(xMainQueue, &zigbee_message, pdMS_TO_TICKS(500)) != pdTRUE) {
                ESP_LOGW(ZB_TAG, "Send to main queue failed");
            }
        }

    }


    return ret;
}

static esp_err_t zb_window_covering_handler(const esp_zb_zcl_window_covering_movement_message_t *message)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, ZB_TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, ZB_TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(ZB_TAG, "Received message: endpoint(%d), cluster(0x%x), command(0x%x), payload(%d)", message->info.dst_endpoint, message->info.cluster,
             message->command, message->payload.lift_value);

    if (ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING == message->info.cluster
            && HA_CLOSURE_ENDPOINT == message->info.dst_endpoint) {

        uint32_t main_task_message = QM_UNKNOWN;
        uint16_t main_task_message_param = 0;

        switch (message->command) {
            case ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN               :
                main_task_message = zb_Inversion ? QM_ZIGBEE_DOWN : QM_ZIGBEE_UP;
                break;
            case ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE            :
                main_task_message = zb_Inversion ? QM_ZIGBEE_UP : QM_ZIGBEE_DOWN;
                break;
            case ESP_ZB_ZCL_CMD_WINDOW_COVERING_STOP                  :
                main_task_message = QM_ZIGBEE_STOP;
                break;

            case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_VALUE      :
                main_task_message = QM_ZIGBEE_TO_VALUE;
                main_task_message_param = message->payload.lift_value;
                break;

            case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE :
                main_task_message = QM_ZIGBEE_TO_PERCENTAGE;
                main_task_message_param = message->payload.lift_value;
                break;

            case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_TILT_VALUE      :
            case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_TILT_PERCENTAGE :
            default:
                ret = ESP_ERR_INVALID_ARG;
                break;
        }

        if (QM_UNKNOWN != main_task_message) {
            uint32_t zigbee_message = ((uint32_t)main_task_message_param << 8) + main_task_message;
            if (xQueueSend(xMainQueue, &zigbee_message, pdMS_TO_TICKS(500)) != pdTRUE) {
                ESP_LOGW(ZB_TAG, "Send to main queue failed");
            }
        }
    }

    return ret;
}

static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, ZB_TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, ZB_TAG, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(ZB_TAG, "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)", variable->status,
                 message->info.cluster, variable->attribute.id, variable->attribute.data.type,
                 variable->attribute.data.value ? *(uint8_t *)variable->attribute.data.value : 0);
        variable = variable->next;
    }

    return ESP_OK;
}

static esp_err_t zb_write_attr_resp_handler(const esp_zb_zcl_cmd_write_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, ZB_TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, ZB_TAG, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_write_attr_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(ZB_TAG, "Write attribute response: status(%d), cluster(0x%x), attribute(0x%x)", variable->status,
                 message->info.cluster, variable->attribute_id);
        variable = variable->next;
    }

    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_WRITE_ATTR_RESP_CB_ID:
        ret = zb_write_attr_resp_handler((esp_zb_zcl_cmd_write_attr_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_WINDOW_COVERING_MOVEMENT_CB_ID:
        ret = zb_window_covering_handler((esp_zb_zcl_window_covering_movement_message_t *)message);
        break;
    case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
        ret = zb_ota_upgrade_status_handler(*(esp_zb_zcl_ota_upgrade_value_message_t *)message);
        break;
    case ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID:
        ret = zb_ota_upgrade_query_image_resp_handler(*(esp_zb_zcl_ota_upgrade_query_image_resp_message_t *)message);
        break;
    default:
        ESP_LOGW(ZB_TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static void setup_endpoints(void)
{
    esp_zb_ep_list_t *endpoint_list = esp_zb_ep_list_create();

    int a_idx, ep_idx;
    for (a_idx = 0, ep_idx = HA_FIRST_ENDPOINT; a_idx <= HA_LAST_ENDPOINT; a_idx++, ep_idx++ ) {

        esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

        ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(
            cluster_list,
            create_basic_cluster(MANUFACTURER_NAME),
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
        ));

        if (ep_idx == HA_CLOSURE_ENDPOINT) {
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_window_covering_cluster(
                cluster_list,
                create_window_covering_cluster(),
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
            ));
        }

        if (ep_idx == HA_TEMPERATURE_ENDPOINT) {
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(
                cluster_list,
                create_temperature_cluster(),
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
            ));
        }

/* Finally setup a new endpoint */
        esp_zb_endpoint_config_t ep_config = {
            .endpoint = ep_idx,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = HA_DEVICE_ID,
            .app_device_version = 1,
        };


        ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(
            endpoint_list,
            cluster_list,
            ep_config
        ));

    }

/* Register endpoints */

    ESP_ERROR_CHECK(zb_register_ota_upgrade_client_device(endpoint_list, HA_OTA_ENDPOINT));
    ESP_ERROR_CHECK(esp_zb_device_register(endpoint_list));

    ESP_LOGW(ZB_TAG, "Device registered");

    setup_reporting();

}

static void esp_zb_task(void *pvParameters)
{
    esp_log_level_set(ZB_TAG, ESP_LOG_VERBOSE);

#if CONFIG_ESP_ZB_TRACE_ENABLE
   esp_zb_set_trace_level_mask(ESP_ZB_TRACE_LEVEL_DEBUG, ESP_ZB_TRACE_SUBSYSTEM_ZCL | ESP_ZB_TRACE_SUBSYSTEM_NWK | ESP_ZB_TRACE_SUBSYSTEM_TRANSPORT | ESP_ZB_TRACE_SUBSYSTEM_ZDO);
#endif

    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    setup_endpoints();

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_stack_main_loop();
}


static void setup_temperature_sensor(void)
{
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_handle));
}

static esp_err_t read_temperature_sensor(void)
{
    esp_err_t err = ESP_OK;
    float tsens_out = 0;
    // Enable temperature sensor
    ESP_RETURN_ON_ERROR(temperature_sensor_enable(temp_handle), ZB_TAG, "Error enabling temperature sensor!");
    // Get converted sensor data
    ESP_RETURN_ON_ERROR(temperature_sensor_get_celsius(temp_handle, &tsens_out), ZB_TAG, "Error reading temperature sensor!");

    temperature_sensor_value = (int16_t)(tsens_out * 100.0f);
    // Disable the temperature sensor if it is not needed and save the power
    ESP_RETURN_ON_ERROR(temperature_sensor_disable(temp_handle), ZB_TAG, "Error disabling temperature sensor!");
    return err;
}



/********************* NVS functions **************************/

static esp_err_t nvs_read_int_attribute(nvs_handle_t handle, const char *key, int32_t *value)
{
    if (handle) {
        if (ESP_OK == nvs_get_i32(handle, key, value)) {
            ESP_LOGI(MAIN_TAG, "Loaded attribute: %s = %ld", key, *value);
            return ESP_OK;
        }
    }
    return ESP_ERR_INVALID_ARG;
}

static const struct nvs_SavedParameters {
    char *key;
    int32_t *value_ptr;
} nvs_SavedParameters[] = {
    { nvs_CurrentPosition, &currentPosition   },
    { nvs_zbUpperLimit,    &zb_UpperLiftLimit },
    { nvs_zbLowerLimit,    &zb_LowerLiftLimit },
    { nvs_UpperLimit,      &upperLiftLimit    },
    { nvs_LowerLimit,      &lowerLiftLimit    },
    { nvs_Velocity,        &zbLiftVelocity    },
    { nvs_ModeAttr,        &zb_WindowCoveringAttrs.mode },
};

static esp_err_t nvs_main_task_read_vars(void)
{
    nvs_handle_t handle = 0;
    esp_err_t err;

    err = nvs_open(ZB_STORAGE_NAMESPACE, NVS_READONLY, &handle);
    if (ESP_OK != err) {
        ESP_LOGE(MAIN_TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        handle = 0;
        return err;
    }

/*  Attempt to restore saved attributes */
    unsigned i;
    for (i = 0; i < lengthof(nvs_SavedParameters); i++) {
        int32_t saved_value;
        err = nvs_read_int_attribute(handle, nvs_SavedParameters[i].key, &saved_value);
        if (ESP_OK == err) {
            *nvs_SavedParameters[i].value_ptr = saved_value;
        }
    }

    if (ESP_OK == err) {
        set_encoder_position(currentPosition);
    }

    zb_WindowCoveringAttrs.mode |= ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_LEDS_WILL_DISPLAY_FEEDBACK;
    nvs_close(handle);
    return err;
}

static esp_err_t nvs_main_task_save_vars(int32_t *varlist[], int count)
{
    nvs_handle_t handle = 0;
    esp_err_t err = ESP_OK;

    ESP_RETURN_ON_ERROR(nvs_open(ZB_STORAGE_NAMESPACE, NVS_READWRITE, &handle), MAIN_TAG, "Error opening NVS handle!");

    int j;
    for (j = 0; j < count && ESP_OK == err; j++) {
        unsigned i;
        for (i = 0; i < lengthof(nvs_SavedParameters) && ESP_OK == err; i++) {
            if (nvs_SavedParameters[i].value_ptr == varlist[j]) {
                ESP_LOGI(MAIN_TAG, "Saved attribute: %s = %ld", nvs_SavedParameters[i].key, *varlist[j]);
                err = nvs_set_i32(handle, nvs_SavedParameters[i].key, *varlist[j]);
            }
        }

    }

    if (err != ESP_OK)
    {
        ESP_LOGE(MAIN_TAG, "Failed to write to NVS (%s)!", esp_err_to_name(err));

    } else {

        err = nvs_commit(handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(MAIN_TAG, "Failed to commit to NVS (%s)!", esp_err_to_name(err));
        }
    }

    nvs_close(handle);
    return err;
}

/*******************     Motor / encoder task    **********************/


/*
    Returns 1 if currentPosition is further away from the targetPosition than was previousPosition,
    i.e. the motor should be stopped.
*/
static int read_and_validate_motor_direction(void)
{
    int save_current = currentPosition;
    if (ESP_OK == get_encoder_position(&currentPosition)) {
        previousPosition = save_current;
    /* if current position is between previous and target return 0, otherwise 1  */
        return (currentPosition != previousPosition) && ((currentPosition > previousPosition) ^ (currentPosition < targetPosition));
    }
    return 0;
}

/*
    Move covering to new position (in encoder units) using current velocity setting.
    Updates targetPosition.
*/
static void move_covering_to(int position)
{
    int direction = position < currentPosition ? MOTOR_DIRECTION_UP : !MOTOR_DIRECTION_UP;
    if (ESP_OK == encoder_start()) {
        targetPosition = position;
        unsigned int speed = (zbLiftVelocity * MOTOR_PWM_DUTY_MAX) / maxLiftVelocity;
        ESP_LOGI(MAIN_TAG, "Starting the motor to go to %d at speed %u", targetPosition, speed);
        pwm_motor_run(direction, speed);
    } else {
        ESP_LOGE(MAIN_TAG, "Error starting encoder.");
    }

}

static void motor_control_task(void *pvParameters)
{
    int16_t prev_temperature = 0;
    int32_t prev_position = ENCODER_MAX_LIMIT + 1; /* Invalid value to trigger attribute update  */
    uint32_t task_msg = 0;

    rotary_encoder_init();
    pwm_motor_init();
    setup_temperature_sensor();

    int32_t prev_saved_position = currentPosition;

/* Set if lift percentage attribute should be updated as a result of a change to another attribute */
    int update_percentage_flag = 0;

    while (1) {

/* This  also updates position reading from the encoder */
        int wrong_motor_direction = read_and_validate_motor_direction();
        if (wrong_motor_direction) {
            if (ESP_OK == pwm_motor_stop()) {
                encoder_stop();
                read_and_validate_motor_direction();
                ESP_LOGI(MAIN_TAG, "Target is reached, stopping motor. Position %ld, previous: %ld", currentPosition, prev_position);
            }
        }

        if (xQueueReceive( xMainQueue, &task_msg, pdMS_TO_TICKS(100) ) == pdPASS ) {

            enum _queue_message_code cmd = (enum _queue_message_code)(task_msg & 0xff);
            int zb_value = ((int32_t)task_msg >> 8);
            int position = 0;

            switch (cmd) {
                case QM_FROM_ENCODER: /* Encoder watchpoint signalled - stop the motor. */
                case QM_ZIGBEE_STOP: /* Stop command received via Zigbee */
                    if (ESP_OK == pwm_motor_stop()) {
                        encoder_stop();
                        read_and_validate_motor_direction();
                        ESP_LOGI(MAIN_TAG, "Motor has stopped. Position %ld", currentPosition);
                    }
                    break;
                case QM_ZIGBEE_UP:
                    move_covering_to(upperLiftLimit);
                    break;
                case QM_ZIGBEE_DOWN:
                    move_covering_to(lowerLiftLimit);
                    break;
                case QM_ZIGBEE_TO_VALUE:
                    position = zigbee_units_to_encoder(zb_value);
                    move_covering_to(position);
                    break;
                case QM_ZIGBEE_TO_PERCENTAGE:
                    position = zigbee_percentage_to_encoder(zb_value);
                    move_covering_to(position);
                    break;
                case QM_ZIGBEE_SET_UPPER:
                    upperLiftLimit = currentPosition;
                    {
                        int32_t *nvs_save[] = { &upperLiftLimit, &zb_UpperLiftLimit };
                        nvs_main_task_save_vars(nvs_save, lengthof(nvs_save));
                    }
                    update_percentage_flag = 1;
                    break;
                case QM_ZIGBEE_SET_LOWER:
                    lowerLiftLimit = currentPosition;
                    {
                        int32_t *nvs_save[] = { &lowerLiftLimit, &zb_LowerLiftLimit };
                        nvs_main_task_save_vars(nvs_save, lengthof(nvs_save));
                    }
                    update_percentage_flag = 1;
                    break;
                case QM_ZIGBEE_SET_VELOCITY:
                    {
                        int32_t *nvs_save[] = { &zbLiftVelocity };
                        nvs_main_task_save_vars(nvs_save, lengthof(nvs_save));
                    }
                    break;

                case QM_ZIGBEE_SET_MODE:
                    {
                        int32_t *nvs_save[] = { &zb_WindowCoveringAttrs.mode };
                        nvs_main_task_save_vars(nvs_save, lengthof(nvs_save));
                    }
                    update_percentage_flag = 1;
                    break;
                case QM_ZIGBEE_RESET_LIMITS:
                    upperLiftLimit = ENCODER_MAX_LIMIT;
                    lowerLiftLimit = ENCODER_MIN_LIMIT;
                    encoder_reset();
                    currentPosition = 0;
                    prev_saved_position = 0;
                    {
                        int32_t *nvs_save[] = { &currentPosition, &lowerLiftLimit, &upperLiftLimit };
                        nvs_main_task_save_vars(nvs_save, lengthof(nvs_save));
                    }
                    update_percentage_flag = 1;
                default:
                    break;
            }

        }

        if ((pwm_motor_running() && abs(prev_saved_position - currentPosition) >= movingSaveDelta)
            || (!pwm_motor_running() && abs(currentPosition - prev_saved_position) >= staticSaveDelta)
        ) {
            prev_saved_position = currentPosition;
            ESP_LOGI(MAIN_TAG, "Saving position: %ld", currentPosition);
            int32_t *nvs_save[] = { &currentPosition };
            nvs_main_task_save_vars(nvs_save, lengthof(nvs_save));
        }

        int have_lock = 0;

	if (prev_position != currentPosition || update_percentage_flag) {
	    if (have_lock || esp_zb_lock_acquire(portMAX_DELAY) ) {
	        have_lock = 1;
	        update_percentage_flag = 0;
	        prev_position = currentPosition;
	        int16_t zb_position = encoder_to_zigbee_units(currentPosition);
	        int16_t zb_percentage = encoder_to_zigbee_percentage(currentPosition);

		esp_zb_zcl_set_attribute_val(HA_CLOSURE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
		    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_ID, &zb_position, false);
		esp_zb_zcl_set_attribute_val(HA_CLOSURE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
		    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID, &zb_percentage, false);
	    }
	}

        read_temperature_sensor();

	if (prev_temperature != temperature_sensor_value) {
	    if (have_lock || esp_zb_lock_acquire(portMAX_DELAY) ) {
	        have_lock = 1;
	        prev_temperature = temperature_sensor_value;

		esp_zb_zcl_set_attribute_val(HA_TEMPERATURE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
		    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &prev_temperature, false);
	    }
	}

        if (have_lock) {
	    esp_zb_lock_release();
	    have_lock = 0;
        }

//	vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    init_gpio();

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    init_timer(&timer, timer_resolution);
    nvs_main_task_read_vars();

    xTaskCreate(esp_zb_task, "Zigbee_main", 1024*8, NULL, 2, NULL);

    gpio_set_level(indication_led_pin, 0);

    xMainQueue = xQueueCreateStatic(QUEUE_LENGTH, QUEUE_ITEM_SIZE, ucQueueStorageArea, &xStaticQueue );

    motor_control_task(NULL);
}
