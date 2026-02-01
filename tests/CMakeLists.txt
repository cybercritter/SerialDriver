enable_testing()

set(TEST_SOURCES
    SerialDriverTest.cpp
)

if(EXISTS "${CMAKE_SOURCE_DIR}/third_party/googletest/CMakeLists.txt")
    add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/googletest ${CMAKE_BINARY_DIR}/googletest)
else()
    include(FetchContent)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
    )
    FetchContent_MakeAvailable(googletest)
endif()

add_executable(${PROJECT_NAME}Tests ${TEST_SOURCES})


target_link_libraries(${PROJECT_NAME}Tests PRIVATE lib${PROJECT_NAME} gtest_main)
target_include_directories(${PROJECT_NAME}Tests PRIVATE ${CMAKE_SOURCE_DIR}/inc)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME}Tests)
