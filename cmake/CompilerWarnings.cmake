# * Comprehensive warnings set
# * Sources:
# * https://github.com/lefticus/cppbestpractices/blob/master/02-Use_the_Tools_Available.md
# * https://github.com/lefticus/cpp_starter_project/blob/master/cmake/CompilerWarnings.cmake

function(set_project_warnings)

  set(CLANG_WARNINGS
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
      # -Wformat=2 # warn on security issues around functions that format output (ie printf)
  )

  if(WARNINGS_AS_ERRORS)
    set(CLANG_WARNINGS ${CLANG_WARNINGS} -Werror)
  endif()

  set(GCC_WARNINGS
      ${CLANG_WARNINGS}
      -Wnull-dereference # warn if a null dereference is detected
      -Wmisleading-indentation # warn if indentation implies blocks where blocks do not exist
      -Wduplicated-cond # warn if if / else chain has duplicated conditions
      -Wduplicated-branches # warn if if / else branches have duplicated code
      -Wlogical-op # warn about logical operations being used where bitwise were probably wanted
      -Wuseless-cast # warn if you perform a cast to the same type
  )

  # We can't run the full set of GCC warnings when using clang-tidy, because clang-tidy doesn't know them
  if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang" OR ENABLE_CLANG_TIDY)
    set(PROJECT_WARNINGS ${CLANG_WARNINGS})
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(PROJECT_WARNINGS ${GCC_WARNINGS})
  else()
    message(AUTHOR_WARNING "No compiler warnings set for '${CMAKE_CXX_COMPILER_ID}' compiler.")
  endif()

  add_compile_options(${PROJECT_WARNINGS})

endfunction()
