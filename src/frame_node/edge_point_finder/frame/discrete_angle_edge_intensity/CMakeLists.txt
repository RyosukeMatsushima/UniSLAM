cmake_minimum_required(VERSION 3.14)

project(DiscreteAngleEdgeIntensity)

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

add_library(
    discrete_angle_edge_intensity
    discrete_angle_edge_intensity.cpp
)

target_include_directories(
    discrete_angle_edge_intensity
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(
    discrete_angle_edge_intensity_test
    discrete_angle_edge_intensity_test.cpp
)

target_link_libraries(
    discrete_angle_edge_intensity_test
    PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
    discrete_angle_edge_intensity
)

include(GoogleTest)
gtest_discover_tests(discrete_angle_edge_intensity_test)
