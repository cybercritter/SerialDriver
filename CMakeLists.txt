cmake_minimum_required(VERSION 3.10)
project(SerialDriver C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(SerialDriver
    main.c
    SerialDriver.c
    circular_buffer.c
)

target_include_directories(SerialDriver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
