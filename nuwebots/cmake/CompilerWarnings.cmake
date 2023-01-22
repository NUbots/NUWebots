# * Comprehensive warnings set
# * Sources:
# * https://github.com/lefticus/cppbestpractices/blob/master/02-Use_the_Tools_Available.md
# * https://github.com/lefticus/cpp_starter_project/blob/master/cmake/CompilerWarnings.cmake

function(set_project_warnings)

  set(COMPILER_WARNINGS
      -Wall
      -Wextra # reasonable and standard
      -Wshadow # warn the user if a variable declaration shadows one from a parent context
      -Wnon-virtual-dtor # warn the user if a class with virtual functions has a non-virtual destructor. This helps
                         # catch hard to track down memory errors
      -Wold-style-cast # warn for c-style casts
      -Wcast-align # warn for potential performance problem casts
      -Wunused # warn on anything being unused
      -Woverloaded-virtual # warn if you overload (not override) a virtual function
      -Wpedantic # warn if non-standard C++ is used
      -Wconversion # warn on type conversions that may lose data
      -Wsign-conversion # warn on sign conversions
      -Wdouble-promotion # warn if float is implicit promoted to double
      -Wnull-dereference # warn if a null dereference is detected
      -Wmisleading-indentation # warn if indentation implies blocks where blocks do not exist
      # -Wformat=2 # warn on security issues around functions that format output (ie printf)
  )

  if(WARNINGS_AS_ERRORS)
    set(COMPILER_WARNINGS ${COMPILER_WARNINGS} -Werror)
  endif()

  if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(PROJECT_WARNINGS ${COMPILER_WARNINGS})
  else()
    # Using some other unsupported compiler, such as MSVC
    message(AUTHOR_WARNING "No compiler warnings set for '${CMAKE_CXX_COMPILER_ID}' compiler.")
  endif()

  add_compile_options(${PROJECT_WARNINGS})

endfunction()
