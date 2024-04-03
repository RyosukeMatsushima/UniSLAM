cmake_minimum_required(VERSION 3.14)

project(MyCppProject)

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

set(SOURCES
  src/main.cpp
  src/MyClass.cpp
  src/ImageProcessor.cpp
  src/frame.cpp
  src/edges_space.cpp
  src/edge.cpp
)

include_directories(
  include
)

add_executable(
  frame_test
  test/frame_test.cpp
  src/frame.cpp
  src/edge.cpp
)

target_link_libraries(
  frame_test
  GTest::gtest_main
  ${OpenCV_LIBS}
)

add_executable(
  edges_space_test
  test/edges_space_test.cpp
  src/edges_space.cpp
)

target_link_libraries(
  edges_space_test
  GTest::gtest_main
  ${OpenCV_LIBS}
)

add_executable(
  polygons_space_test
  test/tools/polygons_space_test.cpp
  test/tools/polygons_space.cpp
)

target_link_libraries(
  polygons_space_test
  GTest::gtest_main
  ${OpenCV_LIBS}
)

add_executable(
  edge_test
  test/edge_test.cpp
  src/edge.cpp
)

target_link_libraries(
  edge_test
  GTest::gtest_main
  ${OpenCV_LIBS}
)

add_executable(
    cpp_copy_test
    test/cpp_copy_test.cpp
)

target_link_libraries(
    cpp_copy_test
    GTest::gtest_main
    ${OpenCV_LIBS}
)

