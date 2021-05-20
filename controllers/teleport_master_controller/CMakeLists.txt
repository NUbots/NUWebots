# Find the webots package
find_package(webots REQUIRED)
# Find YAML
find_package(yaml-cpp REQUIRED)
# Find/list all of the source files
set(controller_sources "teleport_master_controller.cpp")
# Create an executable using all of the sources
add_executable(teleport_master_controller ${controller_sources})
# Make sure we can find the utility folder
target_include_directories(teleport_master_controller PRIVATE ${YAML_CPP_INCLUDE_DIRS})
target_include_directories(teleport_master_controller PRIVATE "${PROJECT_SOURCE_DIR}/shared")
# Link against the webots target (this will also add any necessary include directories to our target)
target_link_libraries(teleport_master_controller PRIVATE webots::webots)
target_link_libraries(teleport_master_controller PRIVATE ${YAML_CPP_LIBRARIES})
# Generate binary in controller source directory
set_target_properties(teleport_master_controller PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                                        OUTPUT_NAME teleport_master_controller)
