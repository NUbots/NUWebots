# Find the webots package
find_package(webots REQUIRED)
# Find YAML
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Find/list all of the source files
set(controller_sources "collect_vision_data.cpp")

# Create an executable using all of the sources
add_executable(collect_vision_data ${controller_sources})

# Make sure we can find the utility folder
target_include_directories(collect_vision_data PRIVATE ${YAML_CPP_INCLUDE_DIRS})
target_include_directories(collect_vision_data PRIVATE "${PROJECT_SOURCE_DIR}/shared")

# Link against the webots target (this will also add any necessary include directories to our target)
target_link_libraries(collect_vision_data PRIVATE webots::webots)
target_link_libraries(collect_vision_data PRIVATE ${YAML_CPP_LIBRARIES})
target_link_libraries(collect_vision_data PRIVATE Eigen3::Eigen)

# Generate binary in controller source directory
set_target_properties(collect_vision_data PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                                        OUTPUT_NAME collect_vision_data)
