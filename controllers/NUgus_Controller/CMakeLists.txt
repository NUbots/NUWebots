# Find/list all of the source files
set(controller_sources NUgus_Controller.cpp)

# Create an executable using all of the sources
add_executable(nugus_controller ${controller_sources})

# Link against the webots target (this will also add any necessary include directories to our target)
target_link_libraries(nugus_controller PRIVATE webots::webots)

# Copy the built executable to the project root directory
add_custom_command(TARGET nugus_controller POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_CURRENT_BINARY_DIR}/nugus_controller ${CMAKE_SOURCE_DIR}
)

