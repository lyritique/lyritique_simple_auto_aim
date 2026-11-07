cmake_minimum_required(VERSION 3.16)
project(camera_calibration VERSION 0.1.0 LANGUAGES CXX )

# 构建风格照抄 kalman_filter_fixed：C++23 + Release + 导出 compile_commands.json
# （唯一的小改动是：只有用户没指定时才强制 Release，这样 -DCMAKE_BUILD_TYPE=Debug 才生效）
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)

# 这个工程不碰 Eigen / ONNX / 装甲板视频，只用到 OpenCV。
# 需要的模块：core / imgproc / calib3d / imgcodecs / videoio / highgui
# 直接用 ${OpenCV_LIBS} 全链接，和 fixed 版保持一致，避免不同发行版组件名不一致。
find_package(OpenCV REQUIRED)

add_executable(
    camera_calibration
    src/main.cc
    src/calib_config.cc
    src/board_detector.cc
    src/calibrator.cc
    src/calib_io.cc
)

target_include_directories(camera_calibration PRIVATE include/ ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_calibration PRIVATE ${OpenCV_LIBS})

add_executable(
    measure_line_straightness
    tools/measure_line_straightness.cpp
    src/calib_config.cc
    src/board_detector.cc
)

target_include_directories(measure_line_straightness PRIVATE include/ ${OpenCV_INCLUDE_DIRS})
target_link_libraries(measure_line_straightness PRIVATE ${OpenCV_LIBS})
