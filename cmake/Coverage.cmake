# cmake/Coverage.cmake
cmake_minimum_required(VERSION 3.15)

option(ENABLE_COVERAGE "Enable coverage" OFF)

function(enable_coverage_flags)
  if(NOT ENABLE_COVERAGE)
    return()
  endif()

  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(WARNING "Coverage enabled but compiler is ${CMAKE_CXX_COMPILER_ID}; skipping coverage flags.")
    return()
  endif()

  # Coverage compile/link flags
  add_compile_options(-O0 -g --coverage)
  add_link_options(--coverage)
endfunction()

# Create a generic coverage target and optional per-label targets.
# Usage:
#   add_coverage_target(coverage)                     # overall
#   add_coverage_target(coverage-my_lib LABEL my_lib) # per label
function(add_coverage_target target_name)
  if(NOT ENABLE_COVERAGE)
    return()
  endif()

  set(options)
  set(oneValueArgs LABEL)
  set(multiValueArgs EXCLUDE_PATTERNS)
  cmake_parse_arguments(COV "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  find_program(LCOV_EXEC lcov REQUIRED)
  find_program(GENHTML_EXEC genhtml REQUIRED)

  # Where reports go
  set(COVERAGE_DIR "${CMAKE_BINARY_DIR}/coverage/${target_name}")
  set(INFO_FILE     "${COVERAGE_DIR}/coverage.info")
  set(FILTERED_FILE "${COVERAGE_DIR}/coverage.filtered.info")
  set(HTML_DIR      "${COVERAGE_DIR}/html")

  # Default excludes (tweak to taste)
  set(DEFAULT_EXCLUDES
    "/usr/*"
    "*/_deps/*"
    "*/tests/*"
    "*/test/*"
    "*/CMakeFiles/*"
  )
  set(EXCLUDES ${DEFAULT_EXCLUDES} ${COV_EXCLUDE_PATTERNS})

  # If user provided a CTest label, only run tests with that label
  if(COV_LABEL)
    set(CTEST_FILTER_ARGS -L "${COV_LABEL}")
  else()
    set(CTEST_FILTER_ARGS)
  endif()

  # We do:
  #  - baseline capture (optional but improves accuracy for unexecuted lines)
  #  - run tests
  #  - capture
  #  - filter
  #  - genhtml with branch coverage enabled
  add_custom_target(${target_name}
    COMMAND ${LCOV_EXEC} --capture --directory . --output-file ${INFO_FILE} --ignore-errors inconsistent,unsupported
    COMMAND ${LCOV_EXEC} --remove ${INFO_FILE} ${EXCLUDES} --output-file ${FILTERED_FILE} --ignore-errors inconsistent,unsupported
    COMMAND ${GENHTML_EXEC} ${FILTERED_FILE} --output-directory ${HTML_DIR} --ignore-errors inconsistent,unsupported
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
endfunction()
