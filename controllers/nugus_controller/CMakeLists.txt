# Find the webots package
find_package(webots REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

# Find/list all of the source files
set(controller_sources nugus_controller.cpp)

# Create an executable using all of the sources and the non-clang-tidy suitable generated protobuf sources
add_executable(nugus_controller ${controller_sources} $<TARGET_OBJECTS:messages_protobuf>)

# Make sure we can find the utility folder
target_include_directories(nugus_controller PRIVATE "${PROJECT_SOURCE_DIR}/shared")

# Make sure we can find the generated protobuf header files
target_include_directories(nugus_controller SYSTEM PRIVATE "${PROJECT_BINARY_DIR}/shared/message")

# Use webots, Eigen and protobuf libraries
target_link_libraries(nugus_controller PRIVATE webots::webots)
target_link_libraries(nugus_controller PRIVATE Eigen3::Eigen)
# Link against the protobuf-lite target (this will also add any necessary include directories to our target)
target_link_libraries(nugus_controller PRIVATE protobuf::libprotobuf-lite)

# Generate binary in controller source directory
set_target_properties(
  nugus_controller PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" OUTPUT_NAME nugus_controller
)
