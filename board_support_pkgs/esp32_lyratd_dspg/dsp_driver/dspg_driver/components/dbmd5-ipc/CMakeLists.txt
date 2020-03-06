idf_component_register(INCLUDE_DIRS "include")

add_library(dspg_ipc STATIC IMPORTED)

set_property(TARGET dspg_ipc PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdspg-ipc.a)

set(deps spi_flash misc audio_hal)
foreach(dep ${deps})
   idf_component_get_property(dep_lib ${dep} COMPONENT_LIB)
   target_link_libraries(dspg_ipc INTERFACE ${dep_lib})
endforeach()

target_link_libraries(${COMPONENT_LIB} INTERFACE dspg_ipc)
