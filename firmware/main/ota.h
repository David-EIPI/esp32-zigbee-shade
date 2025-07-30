/*
 * Zigbee OTA upgrade support functions
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 * 2025/07
 * License: BSD-2
 */

#pragma once

#include "esp_zigbee_core.h"
#include "main.h"

/* Zigbee configuration */
#define OTA_UPGRADE_MANUFACTURER            0xD3B1                                  /* The attribute indicates the file version of the downloaded image on the device*/
#define OTA_UPGRADE_IMAGE_TYPE              0x1011                                  /* The attribute indicates the value for the manufacturer of the device */
#define OTA_UPGRADE_RUNNING_FILE_VERSION    263                                     /* The attribute indicates the file version of the running firmware image on the device */
#define OTA_UPGRADE_DOWNLOADED_FILE_VERSION 263                                     /* The attribute indicates the file version of the downloaded firmware image on the device */
#define OTA_UPGRADE_HW_VERSION              0x0102                                  /* The parameter indicates the version of hardware */
#define OTA_UPGRADE_MAX_DATA_SIZE           223                                     /* The recommended OTA image block size */

#define OTA_ELEMENT_HEADER_LEN              6       /* OTA element format header size include tag identifier and length field */

/**
 * @name Enumeration for the tag identifier denotes the type and format of the data within the element
 * @anchor esp_ota_element_tag_id_t
 */
typedef enum esp_ota_element_tag_id_e {
    UPGRADE_IMAGE                               = 0x0000,           /*!< Upgrade image */
} esp_ota_element_tag_id_t;


esp_err_t zb_ota_upgrade_status_handler(esp_zb_zcl_ota_upgrade_value_message_t message);

esp_err_t zb_ota_upgrade_query_image_resp_handler(esp_zb_zcl_ota_upgrade_query_image_resp_message_t message);

esp_err_t zb_register_ota_upgrade_client_device(esp_zb_ep_list_t *endpoint_list, uint8_t endpoint_id);

esp_err_t zb_ota_validate(int valid);

