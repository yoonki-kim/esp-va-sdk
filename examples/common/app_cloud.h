// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
// All rights reserved.

#pragma once

#include "sdkconfig.h"

#ifdef CONFIG_ALEXA_ENABLE_CLOUD
#include <avs_config.h>

void app_cloud_init();
void app_cloud_wifi_prov_event_handler(alexa_prov_event_t event, void *user_data);

#endif /* CONFIG_ALEXA_ENABLE_CLOUD */
