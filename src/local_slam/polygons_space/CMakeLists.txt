cmake_minimum_required(VERSION 3.14)

project(PolygonsSpace)

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

add_library(
    polygons_space
    polygons_space.cpp
)

target_include_directories(
    polygons_space
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    polygons_space
    PUBLIC
    ${OpenCV_LIBS}
)

add_library(
    double_squares_space
    double_squares_space.cpp
)

target_link_libraries(
    double_squares_space
    PUBLIC
    polygons_space
)

add_executable(
    double_squares_space_test
    double_squares_space_test.cpp
)

target_link_libraries(
    double_squares_space_test
    PRIVATE
    double_squares_space
    GTest::gtest_main
)

target_compile_definitions(double_squares_space_test PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include(GoogleTest)
gtest_discover_tests(double_squares_space_test)

