cmake_minimum_required(VERSION 3.14)

project(Edge)

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

add_subdirectory(edge_point)

add_library(edge edge.cpp)

target_include_directories(
    edge
    PUBLIC
    edge_point
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(
    edge_test
    edge_test.cpp
)

target_link_libraries(
    edge_test
    PRIVATE
    edge
    edge_point
    GTest::gtest_main
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

include(GoogleTest)
gtest_discover_tests(edge_test)
