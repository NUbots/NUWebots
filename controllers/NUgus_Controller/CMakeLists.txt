# Description: Webots Controller build using CMake.
# Example: Copy CMakeLists.txt in $WEBOTS_HOME/projects/robots/gctronic/e-puck/controllers/e-puck/ and execute the following command:
#     rm -fr build e-puck; mkdir build; cd build; cmake ..; make; cd ..

cmake_minimum_required(VERSION 3.0)

get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)  # controller directory name

project(${PROJECT})

set(WEBOTS_HOME "/usr/local/webots" CACHE PATH "The path to the webots folder.")

IF (WIN32)
  link_directories(${WEBOTS_HOME}/msys64/mingw64/bin)
ELSE()
  link_directories(${WEBOTS_HOME}/lib)
ENDIF()

file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})

if (NOT CPP_SOURCES STREQUAL "")  # Sources contain C++ files
  add_library(Controller ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX})
  add_library(CppController ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
  include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)
else()  # C
  add_library (Controller ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX})
  include_directories(${WEBOTS_HOME}/include/controller/c)
endif()

add_executable(${PROJECT} ${SOURCES})

target_link_libraries(${PROJECT} Controller CppController)

add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
)
