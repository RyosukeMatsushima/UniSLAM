cmake_minimum_required(VERSION 3.14)

project(MyCppProject)

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

set(SOURCES
  src/main.cpp
  src/MyClass.cpp
  src/ImageProcessor.cpp
  src/frame.cpp
  src/edges_space.cpp
  src/edge.cpp
  src/edge_point_check.cpp
  src/discrete_angle_edge_intensity.cpp
)

include_directories(
  include
)

add_executable(
  frame_test
  test/frame_test.cpp
  src/frame.cpp
  src/edge.cpp
  src/debug_view.cpp
  src/discrete_angle_edge_intensity.cpp
  src/edge_point.cpp
)

target_link_libraries(
  frame_test
  GTest::gtest_main
  ${OpenCV_LIBS}
)

add_executable(
  edges_space_test
  test/edges_space_test.cpp
  src/edges_space.cpp
)

target_link_libraries(
  edges_space_test
  GTest::gtest_main
  ${OpenCV_LIBS}
)

add_executable(
  polygons_space_test
  test/tools/polygons_space_test.cpp
  test/tools/polygons_space.cpp
)

target_link_libraries(
  polygons_space_test
  GTest::gtest_main
  ${OpenCV_LIBS}
)

add_executable(
  edge_test
  test/edge_test.cpp
  src/edge.cpp
  src/edge_point.cpp
)

target_link_libraries(
  edge_test
  GTest::gtest_main
  ${OpenCV_LIBS}
)

add_executable(
    cpp_copy_test
    test/cpp_copy_test.cpp
)

target_link_libraries(
    cpp_copy_test
    GTest::gtest_main
    ${OpenCV_LIBS}
)

add_executable(
    edge_point_check_test
    test/edge_point_check_test.cpp
    src/edge_point_check.cpp
)

target_link_libraries(
    edge_point_check_test
    GTest::gtest_main
    ${OpenCV_LIBS}
)

add_executable(
    discrete_angle_edge_intensity_test
    test/discrete_angle_edge_intensity_test.cpp
    src/discrete_angle_edge_intensity.cpp
)

target_link_libraries(
    discrete_angle_edge_intensity_test
    GTest::gtest_main
    ${OpenCV_LIBS}
)

add_executable(
    edge_point_finder_test
    test/edge_point_finder_test.cpp
    src/discrete_angle_edge_intensity.cpp
    src/edge_point_finder.cpp
    src/edge_point_check.cpp
    src/edge.cpp
    src/edge_point.cpp
    src/frame.cpp
    src/debug_view.cpp
)

target_link_libraries(
    edge_point_finder_test
    GTest::gtest_main
    ${OpenCV_LIBS}
)

add_executable(
    line_3d_test
    test/line_3d_test.cpp
    src/line_3d.cpp
)

target_link_libraries(
    line_3d_test
    GTest::gtest_main
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

add_executable(
    frame_node_test
    test/frame_node_test.cpp
    src/frame_node.cpp
    src/frame.cpp
    src/edge.cpp
    src/edge_point.cpp
    src/edge_point_check.cpp
    src/edge_point_finder.cpp
    src/discrete_angle_edge_intensity.cpp
    src/debug_view.cpp
)

target_link_libraries(
    frame_node_test
    GTest::gtest_main
    ${OpenCV_LIBS}
)

add_executable(
    edge_point_test
    test/edge_point_test.cpp
    src/edge_point.cpp
)

target_link_libraries(
    edge_point_test
    GTest::gtest_main
    ${OpenCV_LIBS}
)

# Add the test executable
add_executable(
    pose_3d_test
    test/pose_3d_test.cpp
)

target_link_libraries(
    pose_3d_test
    GTest::gtest_main
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

add_executable(
    edge_space_dynamics_test
    test/edge_space_dynamics_test.cpp
    src/edge_space_dynamics.cpp
    src/line_3d.cpp
    src/force_calculation.cpp
)

target_link_libraries(
    edge_space_dynamics_test
    GTest::gtest_main
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

add_executable(
    force_calculation_test
    test/force_calculation_test.cpp
    src/force_calculation.cpp
    src/line_3d.cpp
)

target_link_libraries(
    force_calculation_test
    GTest::gtest_main
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

