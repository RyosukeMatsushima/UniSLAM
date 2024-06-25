cmake_minimum_required(VERSION 3.14)

project(LocalSlam)

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

# Find Eigen3 package
find_package(Eigen3 REQUIRED)

add_subdirectory(frame_node)
add_subdirectory(edge_space_dynamics)
add_subdirectory(polygons_space)

add_library(
    local_slam
    local_slam.cpp
)

target_include_directories(
    local_slam
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    local_slam
    PUBLIC #TODO: check other modules
    frame_node
    frame
    edge
    edge_point
    edge_point_finder
    edge_point_check
    discrete_angle_edge_intensity
    edge_space_dynamics
    force_calculation
    line_3d
    Eigen3::Eigen
    ${OpenCV_LIBS}
)

add_executable(
    local_slam_test
    local_slam_test.cpp
)

target_link_libraries(
    local_slam_test
    PRIVATE
    local_slam
    polygons_space
    GTest::gtest_main
    ${OpenCV_LIBS}
)

include(GoogleTest)
gtest_discover_tests(local_slam_test)

