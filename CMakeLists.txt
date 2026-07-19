cmake_minimum_required(VERSION 3.15)
project(TrackingCorrector CXX)

set(CMAKE_CXX_STANDARD 20)

find_package(imgui REQUIRED)
find_package(imguizmo REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glew REQUIRED)
find_package(glm REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(GTest REQUIRED)

add_library(TrackingCorrectorLib STATIC
    src/model/Skeleton.cpp
    src/model/Skeleton.h
    src/model/IkRigConfig.cpp
    src/model/IkRigConfig.h
    src/model/IkRig.cpp
    src/model/IkRig.h
    src/model/IkMath.cpp
    src/model/IkMath.h
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
)

enable_testing()

add_executable(TrackingCorrectorTests
    tests/SkeletonTest.cpp
    tests/IkRigTest.cpp
    tests/IkMathTest.cpp)

target_link_libraries(TrackingCorrectorTests
    TrackingCorrectorLib
    GTest::gtest
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(TrackingCorrectorTests)
