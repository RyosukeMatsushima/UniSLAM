cmake_minimum_required(VERSION 3.14)

project(Frame)

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

# Find Eigen package
find_package(Eigen3 REQUIRED)

add_subdirectory(edge)
add_subdirectory(discrete_angle_edge_intensity)

add_library(
    frame
    frame.cpp
    frame.h
)

target_include_directories(
    frame
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    frame
    PRIVATE
    discrete_angle_edge_intensity
    edge
    edge_point
    GTest::gtest_main
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

add_executable(
    frame_test
    frame_test.cpp
)

target_link_libraries(
    frame_test
    PRIVATE
    frame
    discrete_angle_edge_intensity
    edge
    edge_point
    GTest::gtest_main
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

include(GoogleTest)
gtest_discover_tests(frame_test)

add_library(
    debug_view
    debug_view.cpp
)

target_include_directories(
    debug_view
    PUBLIC
    edge
    edge/edge_point
    ${OpenCV_INCLUDE_DIRS}
)

