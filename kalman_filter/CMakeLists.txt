cmake_minimum_required(VERSION 3.16)
project(kalman_filter VERSION 0.1.0 LANGUAGES CXX )

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
include(CTest)

set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK root")
find_path(
    MVS_INCLUDE_DIR
    NAMES MvCameraControl.h
    PATHS "${MVS_ROOT}/include"
    REQUIRED
)
find_library(
    MVS_CAMERA_LIBRARY
    NAMES MvCameraControl
    PATHS "${MVS_ROOT}/lib/64"
    REQUIRED
)

set(ORT_ROOT "/opt/onnxruntime" CACHE PATH "ONNX Runtime GPU root")
find_path(
    ORT_INCLUDE_DIR
    NAMES onnxruntime_cxx_api.h
    PATHS "${ORT_ROOT}/include"
    REQUIRED
)
find_library(
    ORT_LIBRARY
    NAMES onnxruntime
    PATHS "${ORT_ROOT}/lib"
    REQUIRED
)

add_executable(
    kalman_filter
    src/app/cli.cc
    src/app/main.cc
    src/app/mode_runner.cc
    src/debug/draw.cc
    src/estimation/kalman_filter.cc
    src/runtime/frame_processor.cc
    src/runtime/realtime_pipeline.cc
    src/runtime/replay_pipeline.cc
    src/sensors/mvs_camera.cc
    src/sensors/video_source.cc
    src/timing/clock_sync.cc
    src/timing/gimbal_history.cc
    src/timing/replay_imu.cc
    src/timing/latency_estimator.cc
    src/transport/protocol.cc
    src/transport/serial_link.cc
    src/vision/armor_detector.cc
    src/vision/pnp_solver.cc
)

target_include_directories(
    kalman_filter
    PRIVATE
        src/
        ${OpenCV_INCLUDE_DIRS}
        ${MVS_INCLUDE_DIR}
        ${ORT_INCLUDE_DIR}
)
target_link_libraries(
    kalman_filter
    PRIVATE
        Eigen3::Eigen
        ${OpenCV_LIBS}
        ${MVS_CAMERA_LIBRARY}
        ${ORT_LIBRARY}
        Threads::Threads
)
set_target_properties(
    kalman_filter
    PROPERTIES
        BUILD_RPATH "${MVS_ROOT}/lib/64;${ORT_ROOT}/lib"
)

if(BUILD_TESTING)
    add_executable(
        protocol_serial_test
        tests/protocol_serial_test.cpp
        src/transport/protocol.cc
        src/transport/serial_link.cc
    )
    target_include_directories(protocol_serial_test PRIVATE src/)
    target_link_libraries(
        protocol_serial_test
        PRIVATE
            Threads::Threads
            util
    )
    add_test(
        NAME protocol_serial_test
        COMMAND protocol_serial_test
    )

    add_executable(
        clock_sync_test
        tests/clock_sync_test.cpp
        src/timing/clock_sync.cc
    )
    target_include_directories(clock_sync_test PRIVATE src/)
    add_test(
        NAME clock_sync_test
        COMMAND clock_sync_test
    )

    add_executable(
        gimbal_history_test
        tests/gimbal_history_test.cpp
        src/timing/gimbal_history.cc
    )
    target_include_directories(gimbal_history_test PRIVATE src/)
    target_link_libraries(gimbal_history_test PRIVATE Eigen3::Eigen)
    add_test(
        NAME gimbal_history_test
        COMMAND gimbal_history_test
    )

    add_executable(
        latency_estimator_test
        tests/latency_estimator_test.cpp
        src/timing/latency_estimator.cc
    )
    target_include_directories(latency_estimator_test PRIVATE src/)
    add_test(
        NAME latency_estimator_test
        COMMAND latency_estimator_test
    )

    add_executable(
        kalman_lost_timeout_test
        tests/kalman_lost_timeout_test.cpp
        src/estimation/kalman_filter.cc
    )
    target_include_directories(kalman_lost_timeout_test PRIVATE src/)
    target_link_libraries(kalman_lost_timeout_test PRIVATE Eigen3::Eigen)
    add_test(
        NAME kalman_lost_timeout_test
        COMMAND kalman_lost_timeout_test
    )

    add_executable(
        kalman_yaw_wrap_test
        tests/kalman_yaw_wrap_test.cpp
        src/estimation/kalman_filter.cc
    )
    target_include_directories(kalman_yaw_wrap_test PRIVATE src/)
    target_link_libraries(kalman_yaw_wrap_test PRIVATE Eigen3::Eigen)
    add_test(
        NAME kalman_yaw_wrap_test
        COMMAND kalman_yaw_wrap_test
    )

    add_executable(
        kalman_noise_frame_test
        tests/kalman_noise_frame_test.cpp
        src/estimation/kalman_filter.cc
    )
    target_include_directories(kalman_noise_frame_test PRIVATE src/)
    target_link_libraries(kalman_noise_frame_test PRIVATE Eigen3::Eigen)
    add_test(
        NAME kalman_noise_frame_test
        COMMAND kalman_noise_frame_test
    )
endif()

