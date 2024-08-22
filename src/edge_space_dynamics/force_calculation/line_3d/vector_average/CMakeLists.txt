cmake_minimum_required(VERSION 3.14)

project(VectorAverage)

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

enable_testing()

# Find Eigen
find_package(Eigen3 REQUIRED)

# vector_average
add_library(
    vector_average
    vector_average.cpp
)

target_include_directories(
    vector_average
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    vector_average
    PRIVATE
    Eigen3::Eigen
)

add_executable(
    vector_average_test
    vector_average_test.cpp
)

target_link_libraries(
    vector_average_test
    PRIVATE
    vector_average
    GTest::gtest_main
    Eigen3::Eigen
)
include(GoogleTest)
gtest_discover_tests(vector_average_test)

