cmake_minimum_required(VERSION 3.14)

project(ForceCalculation)

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

# Find Eigen package
find_package(Eigen3 REQUIRED)

add_subdirectory(line_3d)

add_library(
    force_calculation
    force_calculation.cpp
)

target_include_directories(
    force_calculation
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    force_calculation
    PRIVATE
    line_3d
    Eigen3::Eigen
)

add_executable(
    force_calculation_test
    force_calculation_test.cpp
)

target_link_libraries(
    force_calculation_test
    PRIVATE
    force_calculation
    line_3d
    GTest::gtest_main
    Eigen3::Eigen
)

include(GoogleTest)
gtest_discover_tests(force_calculation_test)

add_executable(
    pose_3d_test
    pose_3d_test.cpp
)

target_link_libraries(
    pose_3d_test
    GTest::gtest_main
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

include(GoogleTest)
gtest_discover_tests(pose_3d_test)

