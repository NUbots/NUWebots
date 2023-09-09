# Find the webots package
find_package(webots REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Find/list all of the source files
set(controller_sources vision_teleport_controller.cpp)

# Create an executable using all of the sources
add_executable(vision_teleport_controller ${controller_sources})

# Make sure we can find the utility folder
target_include_directories(vision_teleport_controller PRIVATE ${YAML_CPP_INCLUDE_DIRS})
target_include_directories(vision_teleport_controller PRIVATE "${PROJECT_SOURCE_DIR}/shared")

# Link against the webots target (this will also add any necessary include directories to our target)
target_link_libraries(vision_teleport_controller PRIVATE webots::webots)
target_link_libraries(vision_teleport_controller PRIVATE Eigen3::Eigen)
target_link_libraries(vision_teleport_controller PRIVATE ${YAML_CPP_LIBRARIES})

# Generate binary in controller source directory
set_target_properties(
  vision_teleport_controller PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                                        OUTPUT_NAME vision_teleport_controller
)
