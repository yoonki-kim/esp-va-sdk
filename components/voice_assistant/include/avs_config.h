// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
// All rights reserved.

#ifndef _AVS_CONFIG_H_
#define _AVS_CONFIG_H_

#include <wifi_provisioning/manager.h>

typedef enum alexa_prov_event {
    ALEXA_PROV_EVENT_CREATE_ENDPOINT,
    ALEXA_PROV_EVENT_REGISTER_ENDPOINT,
} alexa_prov_event_t;

/** Callback event for provisioning AVS configuration
 *
 * This api can be set as the event_cb in conn_mgr_prov_t while starting provisioning if the application
 * wants to set the alexa_config_t via the companion app. The cb_user_data in conn_mgr_prov_t must also be 
 * set to an allocated alexa_config_t struct pointer.
 *
 */
void alexa_wifi_prov_event_handler(alexa_prov_event_t event, void *user_data);
int avs_config_data_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen, uint8_t **outbuf, ssize_t *outlen, void *priv_data);

#endif /* _AVS_CONFIG_H_ */
