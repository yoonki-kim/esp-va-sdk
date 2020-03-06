// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
// All rights reserved.

#pragma once

#include "sdkconfig.h"

#ifdef CONFIG_ALEXA_ENABLE_OTA
void app_ota_init();
void app_ota_cloud_agent_callback();
void *app_ota_get_cloud_agent_callback();
#endif /* CONFIG_ALEXA_ENABLE_OTA */
