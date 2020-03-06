idf_component_register()

add_library(va_dsp STATIC IMPORTED)

set_property(TARGET va_dsp PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libva_dsp.a)

set(deps voice_assistant misc media_hal audio_hal dbmd5-ipc utils speech_recog)
foreach(dep ${deps})
   idf_component_get_property(dep_lib ${dep} COMPONENT_LIB)
   target_link_libraries(va_dsp INTERFACE ${dep_lib})
endforeach()

target_link_libraries(${COMPONENT_LIB} INTERFACE va_dsp)
