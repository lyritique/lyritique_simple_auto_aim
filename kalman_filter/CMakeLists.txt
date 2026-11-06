cmake_minimum_required(VERSION 3.16)
project(kalman_filter VERSION 0.1.0 LANGUAGES CXX )

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(
    kalman_filter
    src/kalman_filter.cc
    src/imu_alignment.cc
    src/armor_detector.cc
    src/main.cc
    src/data_interaction.cc
    src/draw.cc
)

target_include_directories(kalman_filter PRIVATE include/ ${OpenCV_INCLUDE_DIRS})
target_link_libraries(kalman_filter PRIVATE Eigen3::Eigen ${OpenCV_LIBS})

