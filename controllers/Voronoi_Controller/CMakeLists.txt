# Find the webots package
find_package(webots REQUIRED)

# Find/list all of the source files
set(controller_sources "Voronoi_Controller.cpp")

# Create an executable using all of the sources
add_executable(Voronoi_Controller ${controller_sources})

# Make sure we can find the utility folder
target_include_directories(Voronoi_Controller PRIVATE "${PROJECT_SOURCE_DIR}/shared")

# Link against the webots target (this will also add any necessary include directories to our target)
target_link_libraries(Voronoi_Controller PRIVATE webots::webots)

# Generate binary in controller source directory
set_target_properties(Voronoi_Controller PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                                        OUTPUT_NAME Voronoi_Controller)
