# Find the protobuf package
find_package(Protobuf REQUIRED)

# Make sure protobuf output directory exists
file(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/shared/message")

# Generate C++ code from protobuf message
add_custom_command(
  OUTPUT "${PROJECT_BINARY_DIR}/shared/message/messages.pb.cc" "${PROJECT_BINARY_DIR}/shared/message/messages.pb.h"
  COMMAND "${PROTOBUF_PROTOC_EXECUTABLE}" ARGS --cpp_out=lite:"${PROJECT_BINARY_DIR}/shared/message"
          -I"${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/messages.proto"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/messages.proto"
  COMMENT "Compiling protocol buffer messages.proto"
)

# * Prevent Effective C++ and unused parameter error checks being performed on generated files.
# * If the protobuf generation starts to generate more warnings, add '-Wno-<warning_name>' to this list of -Wno's
set_source_files_properties(
  "${PROJECT_BINARY_DIR}/shared/message/messages.pb.cc" "${PROJECT_BINARY_DIR}/shared/message/messages.pb.h"
  PROPERTIES
    GENERATED TRUE
    COMPILE_FLAGS
    "-Wno-unused-parameter -Wno-error=unused-parameter -Wno-error -Wno-conversion -Wno-sign-conversion -Wno-array-bounds"
)

# Add separate OBJECTs for the protobufs so that we can skip running clang-tidy on them
add_library(
  messages_protobuf OBJECT "${PROJECT_BINARY_DIR}/shared/message/messages.pb.cc"
                           "${PROJECT_BINARY_DIR}/shared/message/messages.pb.h"
)

# Turn off clang-tidy for them
set_target_properties(messages_protobuf PROPERTIES CXX_CLANG_TIDY "")
