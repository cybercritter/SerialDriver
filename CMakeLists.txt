cmake_minimum_required(VERSION 3.30.0)
project(SerialDriver VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_C_EXTENSIONS OFF )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_PREFIX_PATH "/opt/homebrew/opt/llvm")

include(CTest) # enables ctest targets & testing
enable_testing()

include(cmake/Coverage.cmake)

# Apply coverage flags if enabled
enable_coverage_flags()

# Overall coverage target: `ninja coverage` / `make coverage`
add_coverage_target(coverage)

# Per-target coverage (by CTest label)
# Example: `ninja coverage-my_lib` / `make coverage-my_lib`
add_coverage_target(coverage-my_lib LABEL my_lib)
add_coverage_target(coverage-my_app LABEL my_app)

add_subdirectory(src)
add_subdirectory(tests)
