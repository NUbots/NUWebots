# Find the webots package
find_package(webots REQUIRED)

# Find/list all of the source files
set(controller_sources teleport_controller.cpp)

# Create an executable using all of the sources
add_executable(teleport_controller ${controller_sources})

# Link against the webots target (this will also add any necessary include directories to our target)
target_link_libraries(teleport_controller PRIVATE webots::webots)

# Copy the built executable to the project root directory
add_custom_command(TARGET teleport_controller POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_CURRENT_BINARY_DIR}/teleport_controller ${CMAKE_SOURCE_DIR}
)

