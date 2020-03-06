idf_component_register(INCLUDE_DIRS "include")

add_library(codec STATIC IMPORTED)

set_property(TARGET codec PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libcodecs.a)

set(deps utils fatfs)
foreach(dep ${deps})
   idf_component_get_property(dep_lib ${dep} COMPONENT_LIB)
   target_link_libraries(codec INTERFACE ${dep_lib})
endforeach()

target_link_libraries(${COMPONENT_LIB} INTERFACE codec)
