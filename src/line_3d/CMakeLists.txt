cmake_minimum_required(VERSION 3.14)

project(Line3D)

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

add_library(
    line_3d
    line_3d.cpp
)

target_include_directories(
    line_3d
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(
    line_3d_test
    line_3d_test.cpp
)

target_link_libraries(
    line_3d_test
    GTest::gtest_main
    Eigen3::Eigen
    line_3d
)

