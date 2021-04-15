# Find the webots package
find_package(webots REQUIRED)

# Find the protobuf package
find_package(Protobuf REQUIRED)

# Find/list all of the source files
set(controller_sources nugus_controller.cpp)

# Make sure protobuf output directory exists
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/protobuf")

# Generate C++ code from protobuf message
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/protobuf/messages.pb.cc"
         "${CMAKE_CURRENT_BINARY_DIR}/protobuf/messages.pb.h"
  COMMAND "${PROTOBUF_PROTOC_EXECUTABLE}" ARGS --cpp_out=lite:"${CMAKE_CURRENT_BINARY_DIR}/protobuf"
          -I"${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/messages.proto"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/messages.proto"
  COMMENT "Compiling protocol buffer messages.proto"
)

# Add generated protobuf sources to controller sources
list(APPEND controller_sources "${CMAKE_CURRENT_BINARY_DIR}/protobuf/messages.pb.cc"
     "${CMAKE_CURRENT_BINARY_DIR}/protobuf/messages.pb.h"
)

# Prevent Effective C++ and unused parameter error checks being performed on generated files.
set_source_files_properties(
  "${CMAKE_CURRENT_BINARY_DIR}/protobuf/messages.pb.cc" "${CMAKE_CURRENT_BINARY_DIR}/protobuf/messages.pb.h"
  PROPERTIES GENERATED TRUE COMPILE_FLAGS "-Wno-unused-parameter -Wno-error=unused-parameter -Wno-error"
)

# Create an executable using all of the sources
add_executable(nugus_controller ${controller_sources})

# Make sure we can find the utility folder
target_include_directories(nugus_controller PRIVATE "${PROJECT_SOURCE_DIR}/shared")

# Make sure we can find the generated protobuf header files
target_include_directories(nugus_controller PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/protobuf")

# Link against the webots target (this will also add any necessary include directories to our target)
target_link_libraries(nugus_controller PRIVATE webots::webots)

# Link against the protobuf-lite target (this will also add any necessary include directories to our target)
target_link_libraries(nugus_controller PRIVATE protobuf::libprotobuf-lite)

# Generate binary in controller source directory
set_target_properties(
  nugus_controller PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" OUTPUT_NAME nugus_controller
)
