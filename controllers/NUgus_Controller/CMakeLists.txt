# Find the webots package
find_package(webots REQUIRED)

# Find/list all of the source files
set(controller_sources NUgus_Controller.cpp)

# Create an executable using all of the sources
add_executable(nugus_controller ${controller_sources})

# Link against the webots target (this will also add any necessary include directories to our target)
target_link_libraries(nugus_controller PRIVATE webots::webots)

set_target_properties(nugus_controller PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                                       OUTPUT_NAME nugus_controller)