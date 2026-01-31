cmake_minimum_required(VERSION 3.30)
project(SerialDriver VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF )
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(EXPORT_COMPILE_COMMANDS ON)

# set(LIB_SOURCE_FILES
#     ./src/SerialDriver.c
#     ./src/circular_buffer.c
# )

# add_library(lib${PROJECT_NAME} ${LIB_SOURCE_FILES})
# target_include_directories(lib${PROJECT_NAME} PRIVATE inc)

# add_executable(${PROJECT_NAME} src/main.c)
# target_link_libraries(${PROJECT_NAME} PRIVATE lib${PROJECT_NAME})
# target_include_directories(${PROJECT_NAME} PRIVATE inc)

add_subdirectory(src)