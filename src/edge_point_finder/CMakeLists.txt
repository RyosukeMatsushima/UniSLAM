cmake_minimum_required(VERSION 3.14)

project(EdgePointFinder)

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

add_subdirectory(frame)
add_subdirectory(edge_point_check)

add_library(
    edge_point_finder
    edge_point_finder.cpp
)

target_include_directories(
    edge_point_finder
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    edge_point_finder
    PRIVATE
    frame
    edge_point
    edge
    discrete_angle_edge_intensity
    debug_view
    edge_point_check
    GTest::gtest_main
    ${OpenCV_LIBS}
)

add_executable(
    edge_point_finder_test
    edge_point_finder_test.cpp
)

target_link_libraries(
    edge_point_finder_test
    PRIVATE
    frame
    edge_point
    edge
    discrete_angle_edge_intensity
    debug_view
    edge_point_check
    edge_point_finder
    GTest::gtest_main
    ${OpenCV_LIBS}
)

target_compile_definitions(edge_point_finder_test PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include(GoogleTest)
gtest_discover_tests(edge_point_finder_test)

