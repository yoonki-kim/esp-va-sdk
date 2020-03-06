idf_component_register(INCLUDE_DIRS "include")

add_library(downmix STATIC IMPORTED)

set_property(TARGET downmix PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libesp-downmix.a)

target_link_libraries(${COMPONENT_LIB} INTERFACE downmix)
