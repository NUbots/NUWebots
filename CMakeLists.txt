cmake_minimum_required(VERSION 3.0)

# Make sure a build type has been set
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

project(nugus_controller LANGUAGES CXX)

# We use additional modules that cmake needs to know about
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/")

# GNU Compiler
if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
  # Enable colours on g++ 4.9 or greater
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.9)
    add_compile_options(-fdiagnostics-color=always)
  endif()
endif()

# Output the compilation database
set(CMAKE_EXPORT_COMPILE_COMMANDS
    ON
    CACHE STRING "Enable/Disable output of compile commands during generation." FORCE
)

# If WEBOTS_HOME is not already set, set it to the value of the WEBOTS_HOME environment variable.
# If the environment variable is not defined, then choose some sort of sane default
if(NOT DEFINED WEBOTS_HOME)
  if(DEFINED ENV{WEBOTS_HOME})
    set(WEBOTS_HOME $ENV{WEBOTS_HOME} CACHE PATH "The path to the webots folder.")
  else()
    if(NOT WIN32)
      set(WEBOTS_HOME "/usr/local/webots" CACHE PATH "The path to the webots folder.")
    endif(NOT WIN32)
  endif(DEFINED ENV{WEBOTS_HOME})
endif(NOT DEFINED WEBOTS_HOME)

find_package(webots REQUIRED)

add_subdirectory(controllers)
