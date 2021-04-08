# Find the webots package
find_package(webots REQUIRED)

# Find/list all of the source files
set(controller_sources "vision_test.cpp")

# Create an executable using all of the sources
add_executable(vision_test ${controller_sources})

# Make sure we can find the utility folder
target_include_directories(vision_test PRIVATE "${PROJECT_SOURCE_DIR}/shared")

# Link against the webots target (this will also add any necessary include directories to our target)
target_link_libraries(vision_test PRIVATE webots::webots)

# Generate binary in controller source directory
set_target_properties(vision_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                                        OUTPUT_NAME vision_test)
