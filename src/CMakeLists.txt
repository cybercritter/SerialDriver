set(SOURCE_FILES
    SerialDriver.c
    circular_buffer.c
)

add_library(lib${PROJECT_NAME} ${SOURCE_FILES})
target_include_directories(lib${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/inc)

add_executable(${PROJECT_NAME} main.c)
target_link_libraries(${PROJECT_NAME} PRIVATE lib${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/inc)