idf_component_register(SRCS "app_wifi.c" "app_prov.c" "app_cloud_rainmaker.c"
                    INCLUDE_DIRS "."
                    REQUIRES misc wifi_provisioning)
