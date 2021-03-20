cmake_minimum_required(VERSION 3.0)

cmake_minimum_required(VERSION 3.0)

project(nugus_controller)

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

# We use additional modules that cmake needs to know about
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/")

find_package(webots REQUIRED)

add_subdirectory(controllers)

