idf_component_register(INCLUDE_DIRS "include")

add_library(ssdp STATIC IMPORTED)

set_property(TARGET ssdp PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libssdp.a)

set(deps esp_http_server utils)
foreach(dep ${deps})
   idf_component_get_property(dep_lib ${dep} COMPONENT_LIB)
   target_link_libraries(ssdp INTERFACE ${dep_lib})
endforeach()

target_link_libraries(${COMPONENT_LIB} INTERFACE ssdp)
