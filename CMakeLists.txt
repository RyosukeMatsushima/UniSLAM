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

add_subdirectory(src/line_3d)
add_subdirectory(src/edges_space)
add_subdirectory(src/polygons_space)
add_subdirectory(src/frame_node)

include_directories(
  include
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
    src/force_calculation.cpp
)

target_link_libraries(
    edge_space_dynamics_test
    line_3d
    GTest::gtest_main
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

add_executable(
    force_calculation_test
    test/force_calculation_test.cpp
    src/force_calculation.cpp
)

target_link_libraries(
    force_calculation_test
    line_3d
    GTest::gtest_main
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

