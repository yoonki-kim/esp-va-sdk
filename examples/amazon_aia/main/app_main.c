// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
// All rights reserved.

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <esp_event_loop.h>
#include <esp_pm.h>
#include <nvs_flash.h>

#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>

#include <voice_assistant.h>
#include <aia.h>
#include <alexa_local_config.h>

#include <va_mem_utils.h>
#include <scli.h>
#include <va_diag_cli.h>
#include <wifi_cli.h>
#include <tone.h>
#include <avs_config.h>
#include <auth_delegate.h>
#include <speech_recognizer.h>
#include "va_board.h"
#include "app_auth.h"
#include "app_wifi.h"
#include "app_aws_iot.h"

#ifdef CONFIG_ALEXA_ENABLE_OTA
#include "app_ota.h"
#endif

#define SERVICENAME_SSID_PREFIX  "ESP-Alexa-"
#define MAX_CLIENT_ID_LEN (12 + 1) /* 12 bytes of mac address + 1 byte of \0 */

static const char *TAG = "[app_main]";

static EventGroupHandle_t cm_event_group;
const int CONNECTED_BIT = BIT0;
const int PROV_DONE_BIT = BIT1;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
            esp_wifi_set_storage(WIFI_STORAGE_FLASH);
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            app_wifi_stop_timeout_timer();
            printf("%s: Disconnected. Connecting to the AP again\n", TAG);
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            app_wifi_stop_timeout_timer();
            printf("%s: Connected with IP Address: %s\n", TAG, ip4addr_ntoa(&event->ip_info.ip));
            xEventGroupSetBits(cm_event_group, CONNECTED_BIT);
        }
    }
}

static void wifi_init_sta()
{
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_start() );
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
}

void app_prov_done_cb()
{
    xEventGroupSetBits(cm_event_group, PROV_DONE_BIT);
}

/* Event handler for catching provisioning manager events */
static void app_wifi_prov_event_handler(void *user_data, wifi_prov_cb_event_t event, void *event_data)
{
    wifi_sta_config_t *wifi_sta_cfg = NULL;
    wifi_prov_sta_fail_reason_t *reason = NULL;
    size_t ssid_len;
    switch (event) {
        case WIFI_PROV_START:
            ESP_LOGI(TAG, "Provisioning started");
            break;

        case WIFI_PROV_CRED_RECV:
            wifi_sta_cfg = (wifi_sta_config_t *)event_data;
            ssid_len = strnlen((const char *)wifi_sta_cfg->ssid, sizeof(wifi_sta_cfg->ssid));
            ESP_LOGI(TAG, "Received Wi-Fi credentials:\n\tSSID: %.*s\n\tPassword: %s", ssid_len, (const char *)wifi_sta_cfg->ssid, (const char *)wifi_sta_cfg->password);
            break;

        case WIFI_PROV_CRED_FAIL:
            reason = (wifi_prov_sta_fail_reason_t *)event_data;
            ESP_LOGE(TAG, "Provisioning failed!\n\tReason: %s\n\tPlease reset to factory and retry provisioning", (*reason == WIFI_PROV_STA_AUTH_ERROR) ? "Wi-Fi AP password incorrect" : "Wi-Fi AP not found");
            break;

        case WIFI_PROV_CRED_SUCCESS:
            ESP_LOGI(TAG, "Provisioning successful");
            break;

        case WIFI_PROV_END:
            ESP_LOGI(TAG, "Provisioning stopped");
            wifi_prov_mgr_deinit();
            app_prov_done_cb();
            break;

        default:
            break;
    }
}

#define FACTORY_PARTITION_NAME    "fctry"
#define DEVICE_NAMESPACE         "device"

char *app_nvs_alloc_and_get_str(const char *key)
{
    nvs_handle handle;
    esp_err_t err;
    if ((err = nvs_open_from_partition(FACTORY_PARTITION_NAME, DEVICE_NAMESPACE,
                                NVS_READONLY, &handle)) != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed with error %d", err);
        return NULL;
    }
    size_t required_size = 0;
    if ((err = nvs_get_blob(handle, key, NULL, &required_size)) != ESP_OK) {
        ESP_LOGI(TAG, "Error reading %s from nvs of size %d. Error code: %d", key, required_size, err);
        return NULL;
    }
    char *value = va_mem_alloc(required_size + 1, VA_MEM_EXTERNAL); /* + 1 for NULL termination */
    if (value) {
        nvs_get_blob(handle, key, value, &required_size);
    }
    nvs_close(handle);
    return value;
}

void app_main()
{
    ESP_LOGI(TAG, "==== Voice Assistant SDK version: %s ====", va_get_sdk_version());

    amazon_config_t *amazon_cfg = va_mem_alloc(sizeof(amazon_config_t), VA_MEM_EXTERNAL);
    if (!amazon_cfg) {
        ESP_LOGE(TAG, "Failed to alloc voice assistant config");
        abort();
    }
    amazon_cfg->product_id = CONFIG_ALEXA_PRODUCT_ID;

    /* This will never be freed */
    aia_config_t *va_cfg = va_mem_alloc(sizeof(aia_config_t), VA_MEM_EXTERNAL);

    if (!va_cfg) {
        ESP_LOGE(TAG, "Failed to alloc voice assistant config");
        abort();
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ret = nvs_flash_init_partition(FACTORY_PARTITION_NAME);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS Flash init failed");
    }

    va_cfg->device_config.aws_root_ca_pem_cert = app_nvs_alloc_and_get_str("server_cert");
    va_cfg->device_config.certificate_pem_crt_cert = app_nvs_alloc_and_get_str("client_cert");
    va_cfg->device_config.private_pem_crt_cert = app_nvs_alloc_and_get_str("client_key");
    va_cfg->device_config.aws_endpoint = app_nvs_alloc_and_get_str("mqtt_host");        /* If this is not present in the 'fctry' partition, then the one mentioned the menuconfig is used. */
    if (!va_cfg->device_config.aws_endpoint) {
        va_cfg->device_config.aws_endpoint = CONFIG_ALEXA_AWS_ENDPOINT;
        printf("%s: Using the non-NVS aws_endpoint: %s\n", TAG, va_cfg->device_config.aws_endpoint);
    }
    va_cfg->device_config.aws_account_id = app_nvs_alloc_and_get_str("account_id");     /* If this is not present in the 'fctry' partition, then the one mentioned in the menuconfig is used. */
    if (!va_cfg->device_config.aws_account_id) {
        va_cfg->device_config.aws_account_id = CONFIG_ALEXA_AWS_ACCOUNT_ID;
        printf("%s: Using the non-NVS aws_account_id: %s\n", TAG, va_cfg->device_config.aws_account_id);
    }
    va_cfg->device_config.client_id = app_nvs_alloc_and_get_str("device_id");           /* If this is not present in the 'fctry' partition, then it is set as the MAC address of the device. */
    if (!va_cfg->device_config.client_id) {
        va_cfg->device_config.client_id = (char *)va_mem_alloc(MAX_CLIENT_ID_LEN, VA_MEM_EXTERNAL);
        if (va_cfg->device_config.client_id == NULL) {
            ESP_LOGE(TAG, "Client id not allocated");
        }
        /* Deriving client_id from the MAC address. */
        uint8_t mac_int[6] = {0};
        esp_wifi_get_mac(ESP_IF_WIFI_STA, mac_int);
        snprintf(va_cfg->device_config.client_id, MAX_CLIENT_ID_LEN, "%02x%02x%02x%02x%02x%02x", mac_int[0], mac_int[1], mac_int[2], mac_int[3], mac_int[4], mac_int[5]);
        printf("%s: Using the non-NVS client_id: %s\n", TAG, va_cfg->device_config.client_id);
    }

    /* thing_name is only used in case of thing shadow. */
    app_aws_iot_set_thing_name(va_cfg->device_config.client_id);    /* Setting thing_name same as client_id itself. */
    va_cfg->device_config.thing_name = app_aws_iot_get_thing_name();

    va_board_init(); /* Initialize media_hal, media_hal_playback, board buttons and led patters */

    scli_init(); /* Initialize CLI */
    va_diag_register_cli(); /* Add diagnostic functions to CLI */

    wifi_register_cli();
    app_wifi_reset_to_prov_init();
    app_auth_register_cli();
    cm_event_group = xEventGroupCreate();

    printf("\r");       // To remove a garbage print ">>"
    auth_delegate_init(alexa_signin_handler, alexa_signout_handler);

    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM,
        .app_event_handler = {
            .event_cb = app_wifi_prov_event_handler,
            .user_data = NULL
        }
    };

    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

    bool provisioned = false;
    if (wifi_prov_mgr_is_provisioned(&provisioned) != ESP_OK) {
        ESP_LOGE(TAG, "Error getting device provisioning state");
        abort();
    }
    if (app_wifi_get_reset_to_prov() > 0) {
        app_wifi_start_timeout_timer();
        provisioned = false;
        app_wifi_unset_reset_to_prov();
        esp_wifi_set_storage(WIFI_STORAGE_RAM);
    }

    char service_name[20];
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(service_name, sizeof(service_name), "%s%02X%02X", SERVICENAME_SSID_PREFIX, mac[4], mac[5]);

    if (!provisioned) {
        va_led_set(LED_RESET);
        printf("%s: Starting provisioning\n", TAG);
        uint8_t custom_service_uuid[16] = {
            /* This is a random uuid. This can be modified if you want to change the BLE uuid. */
            /* 12th and 13th bit will be replaced by internal bits. */
            0x21, 0x43, 0x65, 0x87, 0x09, 0xba, 0xdc, 0xfe,
            0xef, 0xcd, 0xab, 0x90, 0x78, 0x56, 0x34, 0x12,
        };
        wifi_prov_scheme_ble_set_service_uuid(custom_service_uuid);

        wifi_prov_security_t security = WIFI_PROV_SECURITY_1;
        const char *pop = CONFIG_VOICE_ASSISTANT_POP;
        const char *service_key = NULL;

        alexa_wifi_prov_event_handler(ALEXA_PROV_EVENT_CREATE_ENDPOINT, amazon_cfg);
        if (wifi_prov_mgr_start_provisioning(security, pop, service_name, service_key) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start provisioning");
        }
        alexa_wifi_prov_event_handler(ALEXA_PROV_EVENT_REGISTER_ENDPOINT, amazon_cfg);
        printf("%s: Provisioning started with: \n\tservice name: %s \n\tservice key: %s\n\tproof of possession (pop): %s\n", TAG, service_name, service_key ? service_key : "", pop);
    } else {
        va_led_set(VA_CAN_START);
        ESP_LOGI(TAG, "Already provisioned, starting station");
        wifi_prov_mgr_deinit();
        app_prov_done_cb();
        wifi_init_sta();
    }

    xEventGroupWaitBits(cm_event_group, CONNECTED_BIT | PROV_DONE_BIT, false, true, portMAX_DELAY);

    if (!provisioned) {
        va_led_set(VA_CAN_START);
    }

    ret = alexa_local_config_start(amazon_cfg, service_name);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start local SSDP instance. Some features might not work.");
    }

#ifdef CONFIG_ALEXA_ENABLE_OTA
    app_ota_init();
#endif

    ret = ais_mqtt_init(va_cfg, app_aws_iot_callback);

    if (ret != ESP_OK) {
        while(1) vTaskDelay(2);
    }
    /* This is a blocking call */
    va_dsp_init(speech_recognizer_recognize, speech_recognizer_record);
    /* This is only supported with minimum flash size of 8MB. */
    return;
}
