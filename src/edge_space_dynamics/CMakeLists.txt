cmake_minimum_required(VERSION 3.14)

project(EdgeSpaceDynamics)

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

# Find Eigen
find_package(Eigen3 REQUIRED)

add_subdirectory(force_calculation)

add_library(
    edge_space_dynamics
    edge_space_dynamics.cpp
)

target_include_directories(
    edge_space_dynamics
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    edge_space_dynamics
    PRIVATE
    force_calculation
    line_3d
    GTest::gtest_main
    Eigen3::Eigen
)

add_executable(
    get_frame_pose_test
    get_frame_pose_test.cpp
)

target_link_libraries(
    get_frame_pose_test
    PRIVATE
    edge_space_dynamics
    force_calculation
    line_3d
    GTest::gtest_main
    Eigen3::Eigen
)

add_executable(
    add_new_edge_test
    add_new_edge_test.cpp
)

target_link_libraries(
    add_new_edge_test
    PRIVATE
    edge_space_dynamics
    force_calculation
    line_3d
    GTest::gtest_main
    Eigen3::Eigen
)

include(GoogleTest)
gtest_discover_tests(add_new_edge_test)
gtest_discover_tests(get_frame_pose_test)

