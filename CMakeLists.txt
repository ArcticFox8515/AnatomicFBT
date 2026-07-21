cmake_minimum_required(VERSION 3.15)
project(TrackingCorrector CXX)

set(CMAKE_CXX_STANDARD 20)

find_package(imgui REQUIRED)
find_package(imguizmo REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glew REQUIRED)
find_package(glm REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(spdlog REQUIRED)
find_package(openvr REQUIRED)
find_package(GTest REQUIRED)

add_library(TrackingCorrectorLib STATIC
    src/model/Error.h
    src/model/GlmJson.h
    src/model/BoneNames.h
    src/model/BodyProportions.cpp
    src/model/BodyProportions.h
    src/model/Skeleton.cpp
    src/model/Skeleton.h
    src/model/IkRigConfig.cpp
    src/model/IkRigConfig.h
    src/model/IkRig.cpp
    src/model/IkRig.h
    src/model/IkMath.cpp
    src/model/IkMath.h
    src/model/IkSolvers.cpp
    src/model/IkSolvers.h
    src/model/Retarget.cpp
    src/model/Retarget.h
    src/model/Pose.h
    src/model/ModeController.cpp
    src/model/ModeController.h
    src/model/Recording.cpp
    src/model/Recording.h
    src/model/ReplaySession.cpp
    src/model/ReplaySession.h
    src/model/SessionRecorder.cpp
    src/model/SessionRecorder.h
    src/model/TrackedDevice.cpp
    src/model/TrackedDevice.h
    src/model/TrackerCalibration.cpp
    src/model/TrackerCalibration.h
    src/view/Scene.cpp
    src/view/Scene.h)

target_include_directories(TrackingCorrectorLib PUBLIC src)

target_link_libraries(TrackingCorrectorLib PUBLIC
    GLEW::GLEW
    glfw
    glm::glm
    nlohmann_json::nlohmann_json
)

add_executable(${PROJECT_NAME} WIN32
    src/main.cpp

    src/vr/OpenVrTracking.cpp
    src/vr/OpenVrTracking.h

    src/bindings/imgui_impl_glfw.cpp
    src/bindings/imgui_impl_glfw.h
    src/bindings/imgui_impl_opengl3.cpp
    src/bindings/imgui_impl_opengl3.h
    src/bindings/imgui_impl_opengl3_loader.h)

target_compile_definitions(${PROJECT_NAME} PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLEW)
target_link_libraries(${PROJECT_NAME}
    TrackingCorrectorLib
    imgui::imgui
    imguizmo::imguizmo
    spdlog::spdlog
    openvr::openvr
)

enable_testing()

add_executable(TrackingCorrectorTests
    tests/SkeletonTest.cpp
    tests/BodyProportionsTest.cpp
    tests/IkRigTest.cpp
    tests/IkMathTest.cpp
    tests/IkSolversTest.cpp
    tests/ModeControllerTest.cpp
    tests/RecordingTest.cpp
    tests/ReplaySessionTest.cpp
    tests/RetargetTest.cpp
    tests/SessionRecorderTest.cpp
    tests/TrackedDeviceTest.cpp
    tests/TrackerCalibrationTest.cpp)

target_link_libraries(TrackingCorrectorTests
    TrackingCorrectorLib
    GTest::gtest
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(TrackingCorrectorTests)
