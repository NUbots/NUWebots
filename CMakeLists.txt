cmake_minimum_required(VERSION 3.0)
project(nugus_controller LANGUAGES CXX)

# Make sure a build type has been set
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE
      "Release"
      CACHE STRING "Choose the type of build." FORCE
  )
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Default not to run the clang-tidy checks, default to whatever our CI_BUILD is
option(ENABLE_CLANG_TIDY "Enable building with clang-tidy checks." OFF)
if(ENABLE_CLANG_TIDY)
  find_package(PythonInterp 3 REQUIRED)
  set(CMAKE_CXX_CLANG_TIDY "${PYTHON_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/scripts/clang-tidy.py"
                           "${PROJECT_BINARY_DIR}/clang-tidy-fixes" clang-tidy
  )
  set(CMAKE_C_CLANG_TIDY ${CMAKE_CXX_CLANG_TIDY})

  # Create a target that will apply clang-tidy fixes to the codebase
  add_custom_target(
    apply-clang-tidy
    COMMAND clang-apply-replacements --format --style=file --style-config="${PROJECT_SOURCE_DIR}"
            --remove-change-desc-files "${PROJECT_BINARY_DIR}/clang-tidy-fixes"
    COMMENT "Applying fixes from clang-tidy to the codebase."
  )
endif()

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

# We use nested namespaces which are not available before C++17
set(CMAKE_CXX_STANDARD 17)

# If WEBOTS_HOME is not already set, set it to the value of the WEBOTS_HOME environment variable. If the environment
# variable is not defined, then choose some sort of sane default
if(NOT DEFINED WEBOTS_HOME)
  if(DEFINED ENV{WEBOTS_HOME})
    set(WEBOTS_HOME
        $ENV{WEBOTS_HOME}
        CACHE PATH "The path to the webots folder."
    )
  else()
    if(NOT WIN32)
      set(WEBOTS_HOME
          "/usr/local/webots"
          CACHE PATH "The path to the webots folder."
      )
    endif(NOT WIN32)
  endif(DEFINED ENV{WEBOTS_HOME})
endif(NOT DEFINED WEBOTS_HOME)

# RPath variables use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# Build the RPATH into the binary before install
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# Make OSX use the same RPATH as everyone else
set(CMAKE_MACOSX_RPATH ON)

# Add some useful places to the RPATH These will allow the binary to run from the build folder
list(APPEND CMAKE_INSTALL_RPATH "${WEBOTS_HOME}/lib/controller")

# Mac OSX RPATHs work differently
if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(CMAKE_MACOSX_RPATH ON)
    list(APPEND CMAKE_INSTALL_RPATH "${WEBOTS_HOME}")
endif()

add_subdirectory(controllers)
