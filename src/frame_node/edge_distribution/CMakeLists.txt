cmake_minimum_required(VERSION 3.14)

project(EdgeDistribution)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
# Download and include Google Test
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        release-1.11.0
)
FetchContent_MakeAvailable(googletest)

# Enable testing
enable_testing()

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Add library
add_library(
    edge_distribution
    edge_distribution.cpp
)

target_include_directories(
    edge_distribution
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    edge_distribution
    PRIVATE
    ${OpenCV_LIBS}
)

# Add test executable
add_executable(
    edge_distribution_test
    edge_distribution_test.cpp
)

target_link_libraries(
    edge_distribution_test
    PRIVATE
    edge_distribution
    GTest::gtest_main
    ${OpenCV_LIBS}
)

include(GoogleTest)
gtest_discover_tests(edge_distribution_test)