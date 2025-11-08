/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "esp_pm.h"

#define APP_ID_PLACEHOLDER  0
#define CONN_TAG            "BLE_GPIO_DEMO"
#define DEVICE_NAME         "Bluedroid_GPIO"
static const char *TAG = "LED";

// GPIO4
#define GPIO_OUTPUT_PIN     4
#define GPIO_OUTPUT_SEL     (1ULL << GPIO_OUTPUT_PIN)

// GATT UUIDs
#define GATTS_SERVICE_UUID          0x00FF
#define GATTS_CHAR_UUID_GPIO_STATE  0xFF01
#define GATTS_NUM_HANDLE            4

static const char device_name[] = DEVICE_NAME;

// 全局变量
static esp_gatt_if_t gatts_if = ESP_GATT_IF_NONE;
static uint16_t gatts_conn_id = 0xFFFF;
static bool gatts_connected = false;
static uint16_t gpio_state_handle = 0;
static uint8_t gpio_state_value = 0;

// 广告数据
static uint8_t adv_raw_data[] = {
    0x02, ESP_BLE_AD_TYPE_FLAG, 0x06,
    0x0F, ESP_BLE_AD_TYPE_NAME_CMPL, 'B', 'l', 'u', 'e', 'd', 'r', 'o', 'i', 'd', '_', 'G', 'P', 'I', 'O',
    0x02, ESP_BLE_AD_TYPE_TX_PWR, 0x09,
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x320,
    .adv_int_max        = 0x320,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// 静态 UUID 常量（避免 & 常量）
static uint16_t service_uuid_val = GATTS_SERVICE_UUID;
static uint16_t char_uuid_val = GATTS_CHAR_UUID_GPIO_STATE;
static uint16_t char_decl_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static uint16_t cccd_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static uint8_t char_prop = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

// GATT 属性表
static const esp_gatts_attr_db_t gatt_db[GATTS_NUM_HANDLE] = {
    // [0] Service Declaration
    [0] = {{ESP_GATT_AUTO_RSP}, {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p      = (uint8_t *)&service_uuid_val,
            .perm        = ESP_GATT_PERM_READ,
            .max_length  = sizeof(uint16_t),
            .length      = sizeof(uint16_t),
            .value       = (uint8_t *)&service_uuid_val,
    }},

    // [1] Characteristic Declaration
    [1] = {{ESP_GATT_AUTO_RSP}, {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p      = (uint8_t *)&char_decl_uuid,
            .perm        = ESP_GATT_PERM_READ,
            .max_length  = sizeof(uint8_t),
            .length      = sizeof(uint8_t),
            .value       = &char_prop,
    }},

    // [2] Characteristic Value (GPIO State)
    [2] = {{ESP_GATT_AUTO_RSP}, {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p      = (uint8_t *)&char_uuid_val,
            .perm        = ESP_GATT_PERM_READ,
            .max_length  = 1,
            .length      = 0,
            .value       = NULL,
    }},

    // [3] CCCD
    [3] = {{ESP_GATT_AUTO_RSP}, {
            .uuid_length = ESP_UUID_LEN_16,
            .uuid_p      = (uint8_t *)&cccd_uuid,
            .perm        = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            .max_length  = sizeof(uint16_t),
            .length      = 0,
            .value       = NULL,
    }},
};

// 函数声明
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void update_gpio_and_notify(void);
#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0;

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

static void update_gpio_and_notify(void)
{
    gpio_set_level(GPIO_OUTPUT_PIN, gpio_state_value);
    s_led_state = gpio_state_value;
    blink_led();
    ESP_LOGI(CONN_TAG, "set GPIO%d to %d", GPIO_OUTPUT_PIN, gpio_state_value);
    if (gatts_connected && gpio_state_handle != 0) {
        uint8_t data = gpio_state_value;
        esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, gatts_conn_id,
                                                    gpio_state_handle, 1, &data, false);
        if (ret != ESP_OK) {
            ESP_LOGW(CONN_TAG, "Notify failed: %s", esp_err_to_name(ret));
        }
    }
}

void app_main(void)
{
    esp_err_t ret;

    // 初始化 NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(APP_ID_PLACEHOLDER));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(500));
    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(device_name));

    // 安全参数
    uint32_t passkey = 123456;
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_MITM | ESP_LE_AUTH_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;

    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap)));

    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data_raw(adv_raw_data, sizeof(adv_raw_data)));

    // 初始化 GPIO
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTPUT_SEL,
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(GPIO_OUTPUT_PIN, 0);
    configure_led();

    esp_pm_config_t pm_config = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 10
    };
    esp_pm_configure(&pm_config);
}



static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        if (param->adv_data_raw_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            esp_ble_gap_start_advertising(&adv_params);
            ESP_LOGI(CONN_TAG, "Advertising started");
        }
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (param->ble_security.auth_cmpl.success) {
            ESP_LOGI(CONN_TAG, "Pairing successful");
            gpio_state_value = 1;
            update_gpio_and_notify();
        } else {
            ESP_LOGE(CONN_TAG, "Pairing failed");
        }
        break;

    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        if (param->reg.status == ESP_GATT_OK) {
            gatts_if = gatts_if;  // 修复：去掉 ::
            ESP_LOGI(CONN_TAG, "GATT registered, if=%d", gatts_if);
            esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, GATTS_NUM_HANDLE, 0);
        }
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.status == ESP_GATT_OK) {
            uint16_t handle = param->add_char.attr_handle;
            // 比较的是 value 的 handle
            if (handle == gatt_db[2].attr_control.auto_rsp) {
                gpio_state_handle = handle;
                ESP_LOGI(CONN_TAG, "GPIO state handle: %d", gpio_state_handle);
            }
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        gatts_conn_id = param->connect.conn_id;
        gatts_connected = true;
        ESP_LOGI(CONN_TAG, "Connected, conn_id=%d, requesting pairing...", gatts_conn_id);
        esp_ble_conn_update_params_t conn_params = {
            .latency = 0, .min_int = 0x10, .max_int = 0x20, .timeout = 400
        };
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_gap_update_conn_params(&conn_params);

        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(CONN_TAG, "Disconnected");
        gatts_connected = false;
        gpio_state_value = 0;
        update_gpio_and_notify();
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_READ_EVT:
        if (param->read.handle == gpio_state_handle) {
            esp_gatt_rsp_t rsp = {
                .attr_value = {
                    .handle = param->read.handle,
                    .len = 1,
                    .value = { gpio_state_value }
                }
            };
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                        param->read.trans_id, ESP_GATT_OK, &rsp);
        }
        break;

    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == gatt_db[3].attr_control.auto_rsp) {
            ESP_LOGI(CONN_TAG, "CCCD updated");
        }
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                    param->write.trans_id, ESP_GATT_OK, NULL);
        break;

    default:
        break;
    }
}