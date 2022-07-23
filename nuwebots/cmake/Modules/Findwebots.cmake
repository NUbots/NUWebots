if(NOT webots_FOUND)
  # If WEBOTS_HOME is not already set, set it to the value of the WEBOTS_HOME environment variable. If the environment
  # variable is not defined, then choose some sort of sane default. We also re-set WEBOTS_HOME if it's the default value,
  # we assign it in the case that it wasn't set previously which avoids a loop which happens if people run `./b
  # configure` before `source ~/.bashrc` during setup
  if(NOT DEFINED WEBOTS_HOME OR WEBOTS_HOME STREQUAL "${PROJECT_SOURCE_DIR}/../webots")
    if(DEFINED ENV{WEBOTS_HOME})
      set(WEBOTS_HOME
          $ENV{WEBOTS_HOME}
          CACHE PATH "The path to the webots folder."
      )
    else()
      if(NOT WIN32)
        set(WEBOTS_HOME
            "${PROJECT_SOURCE_DIR}/../webots"
            CACHE PATH "The path to the webots folder."
        )
      endif()
    endif()
  endif()

  # Clear our required_vars variable
  unset(required_vars)

  # Find all of the webots controller libraries
  set(webots_libraries "CppCar;CppController;CppDriver")
  foreach(lib ${webots_libraries})
    find_library(
      "webots_${lib}_LIBRARY"
      NAMES ${lib}
      PATHS ${WEBOTS_HOME}/lib/controller
      DOC "The Webots (${lib}) library"
    )

    # Setup an imported target for this library
    add_library(webots::${lib} UNKNOWN IMPORTED)
    set_target_properties(webots::${lib} PROPERTIES IMPORTED_LOCATION ${webots_${lib}_LIBRARY})

    # Setup and export our variables
    set(required_vars ${required_vars} "webots_${lib}_LIBRARY")
    list(APPEND webots_LIBRARIES webots::${lib})
    mark_as_advanced(webots_${lib}_LIBRARY)
  endforeach(lib ${webots_libraries})

  # Link all of our imported targets to our imported library
  add_library(webots::webots INTERFACE IMPORTED)
  target_link_libraries(webots::webots INTERFACE ${webots_LIBRARIES})

  # Make sure the libraries exist in the parent scope
  set(webots_LIBRARIES ${webots_LIBRARIES})

  # Find our include path
  find_path(
    "webots_INCLUDE_DIR"
    NAMES "webots/Robot.hpp"
    PATHS ${WEBOTS_HOME}/include/controller/cpp
    DOC "The Webots include directory"
  )

  # Add include directories to our imported library
  target_include_directories(webots::webots SYSTEM INTERFACE ${webots_INCLUDE_DIR})

  # Setup and export our variables
  list(APPEND required_vars "webots_INCLUDE_DIR")
  set(webots_INCLUDE_DIRS ${webots_INCLUDE_DIR})

  mark_as_advanced(webots_LIBRARIES webots_INCLUDE_DIR webots_INCLUDE_DIRS)

  # Make sure all of thje package variables/components were found
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(
    webots
    FOUND_VAR webots_FOUND
    REQUIRED_VARS ${required_vars}
    VERSION_VAR webots_VERSION
  )
endif()
