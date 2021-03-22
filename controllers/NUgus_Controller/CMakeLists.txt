# Find the webots package
find_package(webots REQUIRED)

# Find the protobuf package
find_package(Protobuf REQUIRED)

# Find/list all of the source files
set(controller_sources NUgus_Controller.cpp)

# Make sure protobuf output directory exists
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/protobuf")

add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/protobuf/RobotControl.pb.cc"
         "${CMAKE_CURRENT_BINARY_DIR}/protobuf/RobotControl.pb.h"
  COMMAND "${PROTOBUF_PROTOC_EXECUTABLE}" ARGS --cpp_out=lite:"${CMAKE_CURRENT_BINARY_DIR}/protobuf"
          -I"${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/RobotControl.proto"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/RobotControl.proto"
  COMMENT "Compiling protocol buffer RobotControl.proto"
)

list(APPEND controller_sources "${CMAKE_CURRENT_BINARY_DIR}/protobuf/RobotControl.pb.cc"
     "${CMAKE_CURRENT_BINARY_DIR}/protobuf/RobotControl.pb.h"
)

# Prevent Effective C++ and unused parameter error checks being performed on generated files.
set_source_files_properties(
  "${CMAKE_CURRENT_BINARY_DIR}/protobuf/RobotControl.pb.cc" "${CMAKE_CURRENT_BINARY_DIR}/protobuf/RobotControl.pb.h"
  PROPERTIES COMPILE_FLAGS "-Wno-unused-parameter -Wno-error=unused-parameter -Wno-error"
)

# Create an executable using all of the sources
add_executable(NUgus_Controller ${controller_sources})

# Make sure we can find the generated protobuf header files
target_include_directories(NUgus_Controller PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/protobuf")

# Link against the webots target (this will also add any necessary include directories to our target)
target_link_libraries(NUgus_Controller PRIVATE webots::webots)

# Link against the webots target (this will also add any necessary include directories to our target)
target_link_libraries(NUgus_Controller PRIVATE protobuf::libprotobuf-lite)

# Copy the built executable to the project root directory
add_custom_command(
  TARGET NUgus_Controller
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/NUgus_Controller ${CMAKE_CURRENT_SOURCE_DIR}
)

# ######################################################################################################################

set(controller_test_sources NUgus_Client_Test.cpp "${CMAKE_CURRENT_BINARY_DIR}/protobuf/RobotControl.pb.cc"
                            "${CMAKE_CURRENT_BINARY_DIR}/protobuf/RobotControl.pb.h"
)

# Create an executable using all of the sources
add_executable(nugus_controller_test ${controller_test_sources})

# Make sure we can find the generated protobuf header files
target_include_directories(nugus_controller_test PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/protobuf")

# Link against the webots target (this will also add any necessary include directories to our target)
target_link_libraries(nugus_controller_test PRIVATE webots::webots)

# Link against the webots target (this will also add any necessary include directories to our target)
target_link_libraries(nugus_controller_test PRIVATE protobuf::libprotobuf-lite)

# Copy the built executable to the project root directory
add_custom_command(
  TARGET nugus_controller_test
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/nugus_controller_test ${CMAKE_SOURCE_DIR}
)
