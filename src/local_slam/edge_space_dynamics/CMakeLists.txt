cmake_minimum_required(VERSION 3.14)

project(EdgeSpaceDynamics)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# Fetch GoogleTest
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
    yaml-cpp
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
    yaml-cpp
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
    yaml-cpp
)

add_executable(
    optimize_test
    optimize_test.cpp
)

target_link_libraries(
    optimize_test
    PRIVATE
    edge_space_dynamics
    force_calculation
    line_3d
    GTest::gtest_main
    Eigen3::Eigen
    yaml-cpp
)

include(GoogleTest)
gtest_discover_tests(add_new_edge_test)
gtest_discover_tests(get_frame_pose_test)
gtest_discover_tests(optimize_test)

target_compile_definitions(add_new_edge_test PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
target_compile_definitions(get_frame_pose_test PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
target_compile_definitions(optimize_test PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

