cmake_minimum_required(VERSION 3.10)

project(MyCppProject)

set(CMAKE_CXX_STANDARD 14)

# Find OpenCV package
find_package(OpenCV REQUIRED)

set(SOURCES
    src/main.cpp
    src/MyClass.cpp
    src/ImageProcessor.cpp
)

include_directories(include)

add_executable(MyCppProject ${SOURCES})

target_link_libraries(MyCppProject ${OpenCV_LIBS})

