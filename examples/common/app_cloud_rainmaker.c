// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
// All rights reserved.

#include "sdkconfig.h"

#ifdef CONFIG_ALEXA_ENABLE_CLOUD

#include <string.h>
#include <esp_log.h>
#include <esp_console.h>

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_ota.h>
#include <esp_rmaker_user_mapping.h>

#include "app_cloud.h"

static const char *TAG = "[app_cloud]";

static int add_user_cli_handler(int argc, char *argv[])
{
    /* Just to go to the next line */
    printf("\n");
    if (argc != 3) {
        printf("%s: Incorrect arguments\n", TAG);
        return 0;
    }
    esp_rmaker_start_user_node_mapping(argv[1], argv[2]);
    return 0;
}

static esp_console_cmd_t cloud_cmds[] = {
    {
        .command = "add-user",
        .help = "add-user <user_id> <secret_key>",
        .func = add_user_cli_handler,
    }
};

static int app_cloud_register_cli()
{
    int cmds_num = sizeof(cloud_cmds) / sizeof(esp_console_cmd_t);
    int i;
    for (i = 0; i < cmds_num; i++) {
        ESP_LOGI(TAG, "Registering command: %s", cloud_cmds[i].command);
        esp_console_cmd_register(&cloud_cmds[i]);
    }
    return 0;
}

/* OTA event callback has not been implemented yet */
// static void app_cloud_ota_event_cb(cloud_ota_event_t event)
// {
//     switch (event) {
//         case CLOUD_OTA_START:
//             va_ui_set_state(VA_UI_OFF);
//             alexa_local_config_stop();
//             va_reset();
//             va_ui_set_state(VA_UI_OTA);
//             break;

//         case CLOUD_OTA_END:
//             va_ui_set_state(VA_UI_OFF);
//             break;

//         default:
//             break;
//     }
// }

void app_cloud_wifi_prov_event_handler(alexa_prov_event_t event, void *user_data)
{
    switch (event) {
        case ALEXA_PROV_EVENT_CREATE_ENDPOINT:
            esp_rmaker_user_mapping_endpoint_create();
            break;

        case ALEXA_PROV_EVENT_REGISTER_ENDPOINT:
            esp_rmaker_user_mapping_endpoint_register();
            break;

        default:
             break;
     }
}

void app_cloud_switch_driver_init()
{
    /* Initialize the device driver here */
}

void app_cloud_switch_state_update(bool state)
{
    /* Update the switch state here */
}

static esp_err_t switch_callback(const char *dev_name, const char *name, esp_rmaker_param_val_t val, void *priv_data)
{
    if (strcmp(name, "power") == 0) {
        printf("%s: **************************************************************\n", TAG);
        printf("%s: ********************** %s turned %s **********************\n", TAG, dev_name, val.val.b ? "ON" : "OFF");
        printf("%s: **************************************************************\n", TAG);
        app_cloud_switch_state_update(val.val.b);
        esp_rmaker_update_param(dev_name, name, val);
    }
    return ESP_OK;
}

static void app_cloud_switch_init()
{
    app_cloud_switch_driver_init();
    esp_rmaker_create_device("Switch", ESP_RMAKER_DEVICE_SWITCH, switch_callback, NULL);
    esp_rmaker_device_add_name_param("Switch", "name");
    esp_rmaker_device_add_power_param("Switch", "power", true);
    esp_rmaker_device_assign_primary_param("Switch", "power");
}

void app_cloud_init()
{
    app_cloud_register_cli();
    esp_rmaker_config_t rainmaker_cfg = {
        .info = {
            .name = "ESP RainMaker Device",
            .type = "Switch",
        },
        .enable_time_sync = false,
    };
    esp_err_t err = esp_rmaker_init(&rainmaker_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialise ESP RainMaker. Aborting!!!");
        return;
    }

    /* Create a device and add the relevant parameters to it */
    app_cloud_switch_init();

    /* Enable OTA */
    esp_rmaker_ota_config_t ota_config = {
        .server_cert = (char *)ESP_RMAKER_OTA_DEFAULT_SERVER_CERT,
    };
    esp_rmaker_ota_enable(&ota_config, OTA_USING_TOPICS);

    /* Start the ESP RainMaker Agent */
    esp_rmaker_start();
}

#endif /* CONFIG_ALEXA_ENABLE_CLOUD */
