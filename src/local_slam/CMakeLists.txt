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

# Fetch yaml-cpp
FetchContent_Declare(
  yaml-cpp
  URL https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.6.3.zip
)
# Disable tests for yaml-cpp to avoid conflict
set(YAML_CPP_BUILD_TESTS OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(yaml-cpp)

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
    camera_model
    Eigen3::Eigen
    ${OpenCV_LIBS}
    yaml-cpp
)

add_executable(
    local_slam_test
    local_slam_test.cpp
)

target_link_libraries(
    local_slam_test
    PRIVATE
    local_slam
    double_squares_space
    GTest::gtest_main
    ${OpenCV_LIBS}
)

add_executable(
    matching_edge_test
    matching_edge_test.cpp
)

target_link_libraries(
    matching_edge_test
    PRIVATE
    frame_node
    edge
    frame
    line_3d
    edge_point_check
    edge_point_finder
    discrete_angle_edge_intensity
    camera_model
    polygons_space
    GTest::gtest_main
    ${OpenCV_LIBS}
)

# Camera model
add_library(
    camera_model
    camera_model.cpp
)

target_include_directories(
    camera_model
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    camera_model
    PUBLIC
    force_calculation
    edge_point
    Eigen3::Eigen
    ${OpenCV_LIBS}
)

add_executable(
    camera_model_test
    camera_model_test.cpp
)

target_link_libraries(
    camera_model_test
    PRIVATE
    camera_model
    GTest::gtest_main
)

# Tests
include(GoogleTest)
gtest_discover_tests(local_slam_test)
gtest_discover_tests(matching_edge_test)
gtest_discover_tests(camera_model_test)


target_compile_definitions(local_slam_test PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
target_compile_definitions(matching_edge_test PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
