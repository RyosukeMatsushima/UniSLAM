cmake_minimum_required(VERSION 3.14)

project(EdgesSpace)

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

# Find OpenCV package
find_package(OpenCV REQUIRED)

add_library(
    edges_space
    edges_space.cpp
)

target_include_directories(
    edges_space
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(
    edges_space_test
    edges_space_test.cpp
)

target_link_libraries(
    edges_space_test
    GTest::gtest_main
    Eigen3::Eigen
    ${OpenCV_LIBS}
    edges_space
)

include(GoogleTest)
gtest_discover_tests(edges_space_test)
