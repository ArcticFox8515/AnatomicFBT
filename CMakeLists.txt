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
find_package(minhook REQUIRED)
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

# --- Driver link: IPC channel + protocol (doc/driver-plan.md phase A) -------
# Shared by the driver DLL and the app; depends on the standard library only
# (no model code, no glm, no openvr, no spdlog), so the driver links no model
# code and the types evolve independently. The real Win32 pipe (step 5) is the
# only part of this layer not covered by unit tests.

add_library(LinkLib STATIC
    src/link/Pipe.h
    src/link/Pipe.cpp
    src/link/Protocol.h
    src/link/MessageChannel.cpp
    src/link/MessageChannel.h)

target_include_directories(LinkLib PUBLIC src)

# --- Step-1 spike of doc/driver-plan.md: throwaway, observation-only ---------
# Everything under src/spike is deleted once the real driver exists.

# All spike logic lives in this static library, linked by the driver DLL, the spike
# client and the tests alike, so every branch can be driven from a unit test with the
# openvr / MinHook / Win32 dependencies injected as seams. SpikeDriver.cpp and
# SpikeClient.cpp are the only files left outside it, and they are pure adapters.
# openvr_driver.h is used for its types only — no openvr_api link.
add_library(SpikeLib STATIC
    src/spike/SpikeClientReport.cpp
    src/spike/SpikeClientReport.h
    src/spike/SpikeDriverEnvironment.cpp
    src/spike/SpikeDriverEnvironment.h
    src/spike/SpikeDriverHooks.cpp
    src/spike/SpikeDriverHooks.h
    src/spike/SpikeGuard.h
    src/spike/SpikeHooks.cpp
    src/spike/SpikeHooks.h
    src/spike/SpikeInterfaces.h
    src/spike/SpikeLog.cpp
    src/spike/SpikeLog.h
    src/spike/SpikeLogFile.cpp
    src/spike/SpikeLogFile.h
    src/spike/SpikeNames.cpp
    src/spike/SpikeNames.h
    src/spike/SpikeObserver.cpp
    src/spike/SpikeObserver.h
    src/spike/SpikePoseMath.h
    src/spike/SpikeServer.cpp
    src/spike/SpikeServer.h)

target_include_directories(SpikeLib PUBLIC src)
target_include_directories(SpikeLib PUBLIC
    $<TARGET_PROPERTY:openvr::openvr,INTERFACE_INCLUDE_DIRECTORIES>)

# SpikeLib uses the link layer (MessageChannel, PipeFactoryFn) for the
# driver-side server (doc/driver-plan.md step 3). PUBLIC so the driver DLL,
# spike_client and tests all get LinkLib transitively.
target_link_libraries(SpikeLib PUBLIC LinkLib)

# The driver DLL links NO openvr_api: openvr_driver.h is header-only for drivers
# (vrserver.exe supplies the implementations through the vtables it hands to Init),
# so only the include dirs are taken from the conan target.
add_library(driver_00trackingcorrector SHARED
    src/spike/SpikeDriver.cpp
    src/spike/SpikeDriverPipe.cpp
    src/spike/SpikeDriverPipe.h)

target_link_libraries(driver_00trackingcorrector PRIVATE SpikeLib minhook::minhook spdlog::spdlog)

# SteamVR layout: <driver dir>/driver.vrdrivermanifest + bin/win64/driver_<name>.dll,
# where <name> must match the manifest name and the directory name. Only the Release
# build is staged there — the config SteamVR loads must not be silently replaced by a
# Debug build of the whole solution (other configs keep the default output dir, which
# is what the tests load).
set(DRIVER_STAGE_DIR ${CMAKE_BINARY_DIR}/driver/00trackingcorrector)
set_target_properties(driver_00trackingcorrector PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${DRIVER_STAGE_DIR}/bin/win64)

add_custom_command(TARGET driver_00trackingcorrector POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/src/driverdll/00trackingcorrector/driver.vrdrivermanifest
        ${DRIVER_STAGE_DIR}/driver.vrdrivermanifest)

# Console tool printing client-side raw poses, for the composition proof.
add_executable(spike_client
    src/spike/SpikeClient.cpp)

target_link_libraries(spike_client PRIVATE SpikeLib openvr::openvr spdlog::spdlog)

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
    tests/LinkProtocolTest.cpp
    tests/LinkMessageChannelTest.cpp
    tests/SpikeClientReportTest.cpp
    tests/SpikeDriverEnvironmentTest.cpp
    tests/SpikeDriverHooksTest.cpp
    tests/SpikeDriverTest.cpp
    tests/SpikeHooksTest.cpp
    tests/SpikeLogFileTest.cpp
    tests/SpikeLogTest.cpp
    tests/SpikeObserverTest.cpp
    tests/SpikeServerTest.cpp
    tests/TrackedDeviceTest.cpp
    tests/TrackerCalibrationTest.cpp)

target_link_libraries(TrackingCorrectorTests
    TrackingCorrectorLib
    SpikeLib
    LinkLib
    GTest::gtest
    GTest::gtest_main
)

# SpikeDriverTest loads the spike driver DLL and drives it through a fake vrserver,
# so it needs the DLL of its own config (the driver headers come with SpikeLib).
target_compile_definitions(TrackingCorrectorTests PRIVATE
    SPIKE_DRIVER_DLL="$<TARGET_FILE:driver_00trackingcorrector>")
add_dependencies(TrackingCorrectorTests driver_00trackingcorrector)

include(GoogleTest)
gtest_discover_tests(TrackingCorrectorTests)
