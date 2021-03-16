set(controller_sources NUgus_Controller.cpp)

add_executable(nugus_controller ${controller_sources})

target_link_libraries(nugus_controller PRIVATE webots::webots)

add_custom_command(TARGET nugus_controller POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_CURRENT_BINARY_DIR}/nugus_controller ${CMAKE_SOURCE_DIR}
)

