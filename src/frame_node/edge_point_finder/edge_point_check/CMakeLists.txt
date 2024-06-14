cmake_minimum_required(VERSION 3.14)

project(EdgePointCheck)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

# Find OpenCV package
find_package(OpenCV REQUIRED)

add_library(
    edge_point_check
    edge_point_check.cpp
)

target_include_directories(
    edge_point_check
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(
    edge_point_check_test
    edge_point_check_test.cpp
)

target_link_libraries(
    edge_point_check_test
    PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
    edge_point_check
)

include(GoogleTest)
gtest_discover_tests(edge_point_check_test)

