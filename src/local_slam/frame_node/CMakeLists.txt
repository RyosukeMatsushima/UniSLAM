cmake_minimum_required(VERSION 3.14)

project(FrameNode)

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

add_subdirectory(edge_point_finder)

add_library(
    frame_node
    frame_node.cpp
)

target_include_directories(
    frame_node
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    frame_node
    PRIVATE
    edge_point
    edge
    frame
    discrete_angle_edge_intensity
    debug_view
    edge_point_check
    edge_point_finder
    GTest::gtest_main
    ${OpenCV_LIBS}
)

add_executable(
    frame_node_test
    frame_node_test.cpp
)

target_link_libraries(
    frame_node_test
    PRIVATE
    frame_node
    edge_point
    edge
    frame
    discrete_angle_edge_intensity
    debug_view
    edge_point_check
    edge_point_finder
    GTest::gtest_main
    ${OpenCV_LIBS}
)

include(GoogleTest)
gtest_discover_tests(frame_node_test)

target_compile_definitions(frame_node_test PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
