idf_component_register(INCLUDE_DIRS "include")

add_library(voice_assistant STATIC IMPORTED)

if(VOICE_ASSISTANT STREQUAL "gva")
    set_property(TARGET voice_assistant PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libgva.a)
elseif(VOICE_ASSISTANT STREQUAL "dialogflow")
    set_property(TARGET voice_assistant PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdialogflow.a)
elseif(VOICE_ASSISTANT STREQUAL "aia")
    set_property(TARGET voice_assistant PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libaia.a)
else()
    set_property(TARGET voice_assistant PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libalexa.a)
endif()

set(deps misc esp_ringbuf nvs_flash streams mbedtls uuid-gen codecs sh2lib protobuf-c audio_hal multipart_parser console sys_playback basic_player bt utils libsodium protocomm wpa_supplicant wifi_provisioning main)
foreach(dep ${deps})
   idf_component_get_property(dep_lib ${dep} COMPONENT_LIB)
   target_link_libraries(voice_assistant INTERFACE ${dep_lib})
endforeach()

target_link_libraries(${COMPONENT_LIB} INTERFACE voice_assistant)
