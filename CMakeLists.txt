cmake_minimum_required(VERSION 3.16)
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

# If this option is set we are building using continous integration
option(CI_BUILD "Enable build options for building in the CI server" OFF)

# * We need to tell cmake where the function for the compiler warnings is (src/cmake)
# * We use additional modules that cmake needs to know about (src/cmake/Modules)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" "${PROJECT_SOURCE_DIR}/cmake/Modules/")
include(CompilerWarnings)

# A flag to set when you want set_project_warnings to turn warnings into errors
option(WARNINGS_AS_ERRORS "All compiler and clang-tidy warnings make compilation fail" OFF)

# If we are doing a CI build then we want warnings to be dealt with, so we fail compilation. We will also make it fail
# if clang-tidy has a warning
if(CI_BUILD)
  set(WARNINGS_AS_ERRORS ON)
endif()

# We can set our project warnings now that we know if warnings are errors
set_project_warnings()

# Default not to run the clang-tidy checks, default to whatever our CI_BUILD is
option(ENABLE_CLANG_TIDY "Enable building with clang-tidy checks." ON)
if(ENABLE_CLANG_TIDY OR CI_BUILD)
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

# Enable address sanitizer
option(USE_ASAN "Enable address sanitization" OFF)
if(USE_ASAN)
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer -U_FORTIFY_SOURCE -fno-common)
  add_link_options(-fsanitize=address)
  link_libraries(asan)
endif()

option(USE_UBSAN "Enable undefined behaviour sanitization" OFF)
if(USE_UBSAN)
  add_compile_options(-fsanitize=undefined -fno-sanitize-recover=all)
  add_link_options(-fsanitize=undefined)
  link_libraries(ubsan)
endif()

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
    endif()
  endif()
endif()

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

add_subdirectory(shared/message)
add_subdirectory(controllers)
