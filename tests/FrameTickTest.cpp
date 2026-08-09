#include <gtest/gtest.h>

#include <glm/gtc/quaternion.hpp>

#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>

#include "model/BoneNames.h"
#include "model/BodyProportions.h"
#include "model/FrameTick.h"
#include "model/IkRig.h"
#include "model/IkRigConfig.h"
#include "model/ModeController.h"
#include "model/Pose.h"
#include "model/Recording.h"
#include "model/ReplaySession.h"
#include "model/Retarget.h"
#include "model/SessionRecorder.h"
#include "model/Skeleton.h"
#include "model/TrackedDevice.h"
#include "model/TrackerCalibration.h"
#include "model/TrackerCorrection.h"

namespace
{
IkRig makeRig()
{
    IkRig rig(Skeleton::makeDefault());
    rig.loadConfig(IkRigConfig::makeDefault());
    return rig;
}

// One device exactly on each target's rest world position (identity
// rotation) — a perfect T-pose; calibration offsets are then identity.
std::vector<TrackedDevice> tPoseDevices(const IkRig& rig)
{
    const WorldTransforms rest = computeWorldTransforms(rig.skeleton);
    std::vector<TrackedDevice> devices;
    for (size_t i = 0; i < rig.targets.size(); ++i)
    {
        const std::string& name = rig.skeleton.joints[rig.targets[i].jointIndex].name;
        const TrackedDeviceKind kind = (name == BoneNames::LeftHand || name == BoneNames::RightHand)
            ? TrackedDeviceKind::Controller
            : TrackedDeviceKind::Tracker;
        devices.push_back({static_cast<int>(100 + i), kind,
                           {rest.positions[rig.targets[i].jointIndex],
                            glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}});
    }
    return devices;
}

// Records the calls the frame tick makes on the pose source so the tests can
// assert which methods ran and how often. `poses` is what pollPoses returns.
struct FakePoseSource : IPoseSource
{
    bool initialized = false;
    std::vector<TrackedDevice> poses;
    int pollCount = 0;
    int sendCount = 0;
    std::vector<DeviceOffset> lastSent;

    bool isInitialized() const override { return initialized; }
    std::vector<TrackedDevice> pollPoses() override { ++pollCount; return poses; }
    void sendOffsets(const std::vector<DeviceOffset>& offsets) override { ++sendCount; lastSent = offsets; }
};

struct FakeGestureSource : IGestureSource
{
    bool initialized = false;
    bool gesture = false;
    int gestureCount = 0;

    bool isInitialized() const override { return initialized; }
    bool bothTriggersJustPressed() override { ++gestureCount; return gesture; }
};

// Captures every line emitted through a link::Logger so the tests can assert
// which messages fired. Mirrors the ServerTest/ObserverTest pattern.
struct RecordingLogger
{
    std::vector<std::string> lines;
    link::Logger logger;

    RecordingLogger()
    {
        logger.setSink([this](const char* message) { lines.emplace_back(message); });
    }

    bool contains(const std::string& needle) const
    {
        return std::any_of(lines.begin(), lines.end(),
                           [&](const std::string& m) { return m.find(needle) != std::string::npos; });
    }
};

// A recorder factory that yields an in-memory stringstream, so no filesystem.
struct MemoryRecorder
{
    std::shared_ptr<std::stringstream> stream =
        std::make_shared<std::stringstream>(std::ios::in | std::ios::out | std::ios::binary);
    SessionRecorder recorder{[this] { return stream; }};
};

// A recorder whose stream is null — openStream returns nothing, so the
// recorder reports an error on the first capture frame.
struct FailingRecorder
{
    SessionRecorder recorder{[] { return std::shared_ptr<std::ostream>(); }};
};

// The full set of model state the app builds at startup and drives the tick
// with. Returned by value (NRVO); IkRig is movable.
struct TickEnv
{
    IkRig rig;
    Skeleton avatar;
    RetargetMap retargetMap;
    CorrectionMap correctionMap;
    MemoryRecorder recorder;
};
TickEnv makeEnv()
{
    IkRig rig = makeRig();
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    RetargetMap retargetMap = buildRetargetMap(rig.skeleton, avatar);
    CorrectionMap correctionMap = buildCorrectionMap(rig, avatar);
    return {std::move(rig), std::move(avatar), std::move(retargetMap),
            std::move(correctionMap), MemoryRecorder{}};
}

// Calibrates every target at rest (identity offsets); leaves the rig in the
// calibration rest pose.
struct CalibratedRig
{
    TrackerCalibration calibration;
    std::vector<TrackedDevice> devices;
};
CalibratedRig calibrateAtRest(IkRig& rig)
{
    const std::vector<TrackedDevice> devices = tPoseDevices(rig);
    const CalibrationFrame frame = updateCalibrationFrame(rig, devices);
    TrackerCalibration calibration;
    captureOffsets(calibration, frame, devices);
    return {calibration, devices};
}

// --- pollAndUpdate: device source selection --------------------------------

TEST(FrameTick, ManualPosePollsWhenPoseSourceConnected)
{
    TickEnv env = makeEnv();
    ModeController controller;  // ManualPose
    FakePoseSource poses;
    poses.initialized = true;
    poses.poses = tPoseDevices(env.rig);
    FakeGestureSource gesture;
    RecordingLogger sink;

    const UpdateResult r = pollAndUpdate(controller, env.rig, nullptr, env.recorder.recorder,
                                         poses, &gesture, 1.0, sink.logger);

    EXPECT_EQ(poses.pollCount, 1);
    EXPECT_EQ(r.devices.size(), poses.poses.size());
    EXPECT_EQ(r.plan.solve, SolveMode::Targets);
    // ManualPose is not Calibration: the caller must tear the trigger reader down.
    EXPECT_TRUE(r.tearDownGestureSource);
    // No events in ManualPose (no capture, no recording).
    EXPECT_TRUE(sink.lines.empty());
}

TEST(FrameTick, ManualPoseReturnsEmptyDevicesWhenDisconnected)
{
    TickEnv env = makeEnv();
    ModeController controller;  // ManualPose
    FakePoseSource poses;
    poses.initialized = false;
    poses.poses = tPoseDevices(env.rig);  // must NOT be returned
    link::Logger logger;

    const UpdateResult r = pollAndUpdate(controller, env.rig, nullptr, env.recorder.recorder,
                                         poses, nullptr, 1.0, logger);

    EXPECT_EQ(poses.pollCount, 0);
    EXPECT_TRUE(r.devices.empty());
    EXPECT_EQ(r.plan.solve, SolveMode::Targets);
    EXPECT_TRUE(r.tearDownGestureSource);
}

TEST(FrameTick, CalibrationReadsGestureFromInitializedSource)
{
    TickEnv env = makeEnv();
    ModeController controller(Mode::Calibration);
    FakePoseSource poses;
    poses.initialized = true;
    poses.poses = tPoseDevices(env.rig);
    FakeGestureSource gesture;
    gesture.initialized = true;
    gesture.gesture = false;
    RecordingLogger sink;

    const UpdateResult r = pollAndUpdate(controller, env.rig, nullptr, env.recorder.recorder,
                                         poses, &gesture, 1.0, sink.logger);

    EXPECT_EQ(gesture.gestureCount, 1);
    EXPECT_EQ(r.plan.solve, SolveMode::None);
    // Still Calibration — do not tear the trigger reader down.
    EXPECT_FALSE(r.tearDownGestureSource);
    EXPECT_TRUE(sink.lines.empty());  // no capture, no recording
}

TEST(FrameTick, CalibrationWithoutGestureSourceIsSafe)
{
    TickEnv env = makeEnv();
    ModeController controller(Mode::Calibration);
    FakePoseSource poses;
    poses.initialized = true;
    poses.poses = tPoseDevices(env.rig);
    link::Logger logger;

    const UpdateResult r = pollAndUpdate(controller, env.rig, nullptr, env.recorder.recorder,
                                         poses, nullptr, 1.0, logger);

    // No crash; gesture simply never read.
    EXPECT_EQ(r.plan.solve, SolveMode::None);
    EXPECT_FALSE(r.tearDownGestureSource);
}

TEST(FrameTick, CalibrationGestureSourceUninitializedIsNotRead)
{
    TickEnv env = makeEnv();
    ModeController controller(Mode::Calibration);
    FakePoseSource poses;
    poses.initialized = true;
    FakeGestureSource gesture;
    gesture.initialized = false;
    gesture.gesture = true;  // would transition if read
    RecordingLogger sink;

    const UpdateResult r = pollAndUpdate(controller, env.rig, nullptr, env.recorder.recorder,
                                         poses, &gesture, 1.0, sink.logger);

    EXPECT_EQ(gesture.gestureCount, 0);
    EXPECT_EQ(r.plan.solve, SolveMode::None);
    EXPECT_TRUE(sink.lines.empty());  // no capture
}

// The calibration->capture transition: update flips the mode, and the
// tearDown flag reflects the POST-update mode (Capture), so the caller
// tears down the trigger reader on the very transition frame. The sink
// receives CalibrationCaptured + RecordingStarted (frame 0 = these devices).
TEST(FrameTick, CalibrationGestureTransitionsToCaptureAndTearsDown)
{
    TickEnv env = makeEnv();
    ModeController controller(Mode::Calibration);
    FakePoseSource poses;
    poses.initialized = true;
    poses.poses = tPoseDevices(env.rig);
    FakeGestureSource gesture;
    gesture.initialized = true;
    gesture.gesture = true;
    RecordingLogger sink;

    const UpdateResult r = pollAndUpdate(controller, env.rig, nullptr, env.recorder.recorder,
                                         poses, &gesture, 1.0, sink.logger);

    EXPECT_EQ(controller.mode(), Mode::Capture);
    EXPECT_EQ(r.plan.solve, SolveMode::Goals);
    ASSERT_EQ(r.plan.goals.size(), env.rig.targets.size());
    EXPECT_TRUE(r.tearDownGestureSource);
    // Two events: capture first, then recording start.
    ASSERT_EQ(sink.lines.size(), 2u);
    EXPECT_NE(sink.lines[0].find("Calibration captured"), std::string::npos);
    EXPECT_NE(sink.lines[1].find(kRecordingPath), std::string::npos);
}

// Outside Calibration the gesture source is never consulted even if set.
TEST(FrameTick, GestureIsNotReadInManualPose)
{
    TickEnv env = makeEnv();
    ModeController controller;  // ManualPose
    FakePoseSource poses;
    poses.initialized = true;
    FakeGestureSource gesture;
    gesture.initialized = true;
    gesture.gesture = true;
    RecordingLogger sink;

    const UpdateResult r = pollAndUpdate(controller, env.rig, nullptr, env.recorder.recorder,
                                         poses, &gesture, 1.0, sink.logger);

    EXPECT_EQ(gesture.gestureCount, 0);
    EXPECT_EQ(r.plan.solve, SolveMode::Targets);
    EXPECT_TRUE(sink.lines.empty());
}

// Replay pulls devices from the loaded recording; the pose source is never
// polled (the driver link is not a Replay input). Mirrors main.cpp's
// enterReplay + loadReplayFile sequence.
TEST(FrameTick, ReplayPullsDevicesFromRecordingNotPoseSource)
{
    TickEnv env = makeEnv();
    const std::vector<TrackedDevice> devices = tPoseDevices(env.rig);
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / "TrackingCorrectorFrameTickReplay";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    {
        std::ofstream file(dir / "rec.tcrec", std::ios::binary | std::ios::trunc);
        RecordingWriter writer(file);
        writer.writeFrame(0.0f, devices);
    }

    ModeController controller;
    controller.switchToReplay();
    env.rig.resetTargets();
    ReplaySession replay;
    replay.scan(dir);
    ASSERT_EQ(replay.files().size(), 1u);
    ASSERT_NO_THROW(replay.load(0, controller, env.rig));  // calibrates from frame 0

    FakePoseSource poses;
    poses.initialized = true;
    poses.poses = {TrackedDevice{999, TrackedDeviceKind::Hmd, {glm::vec3(0.0f), glm::quat(1.0f, 0, 0, 0)}}};
    link::Logger logger;

    const UpdateResult r = pollAndUpdate(controller, env.rig, &replay, env.recorder.recorder,
                                         poses, nullptr, 1.0, logger);

    EXPECT_EQ(poses.pollCount, 0);
    ASSERT_EQ(r.devices.size(), devices.size());
    EXPECT_EQ(r.devices.front().id, devices.front().id);
    EXPECT_EQ(r.plan.solve, SolveMode::Goals);
    EXPECT_TRUE(r.tearDownGestureSource);  // Replay is not Calibration

    std::filesystem::remove_all(dir);
}

// --- pollAndUpdate: event reporting (the logic moved out of main.cpp) ------

// Leaving Capture for ManualPose stops the recording and emits RecordingSaved.
TEST(FrameTick, LeavingCaptureEmitsRecordingSaved)
{
    TickEnv env = makeEnv();
    ModeController controller(Mode::Calibration);
    FakePoseSource poses;
    poses.initialized = true;
    poses.poses = tPoseDevices(env.rig);
    FakeGestureSource gesture;
    gesture.initialized = true;
    gesture.gesture = true;
    RecordingLogger sink;

    // Frame 1: calibrate + start recording.
    pollAndUpdate(controller, env.rig, nullptr, env.recorder.recorder, poses, &gesture, 1.0,
                  sink.logger);
    ASSERT_EQ(controller.mode(), Mode::Capture);
    sink.lines.clear();

    // Frame 2: switch to ManualPose — the recorder stops.
    controller.switchToManual();
    const UpdateResult r = pollAndUpdate(controller, env.rig, nullptr, env.recorder.recorder,
                                         poses, nullptr, 2.0, sink.logger);

    EXPECT_EQ(controller.mode(), Mode::ManualPose);
    ASSERT_EQ(sink.lines.size(), 1u);
    EXPECT_NE(sink.lines[0].find("Recording saved"), std::string::npos);
    EXPECT_TRUE(r.tearDownGestureSource);
}

// A recorder whose stream cannot be opened emits RecordingFailed at Error
// level with the error detail, not RecordingStarted.
TEST(FrameTick, FailingRecorderEmitsRecordingFailed)
{
    TickEnv env = makeEnv();
    FailingRecorder failing;
    ModeController controller(Mode::Calibration);
    FakePoseSource poses;
    poses.initialized = true;
    poses.poses = tPoseDevices(env.rig);
    FakeGestureSource gesture;
    gesture.initialized = true;
    gesture.gesture = true;
    RecordingLogger sink;

    pollAndUpdate(controller, env.rig, nullptr, failing.recorder, poses, &gesture, 1.0,
                  sink.logger);

    EXPECT_EQ(controller.mode(), Mode::Capture);
    // CalibrationCaptured fires (the mode flipped); RecordingStarted does NOT
    // (the stream failed); RecordingFailed fires with the error detail.
    EXPECT_TRUE(sink.contains("Calibration captured"));
    EXPECT_FALSE(sink.contains("Recording capture session"));
    ASSERT_TRUE(sink.contains("Recording failed"));
    // The failure line carries the recorder's error detail.
    for (const std::string& m : sink.lines)
        if (m.find("Recording failed") != std::string::npos)
            EXPECT_GT(m.size(), std::string("Recording failed; stopping it: ").size());
}

// A logger with no sink installed is safe — lines dropped, no crash.
TEST(FrameTick, LoggerWithoutSinkIsSafe)
{
    TickEnv env = makeEnv();
    ModeController controller(Mode::Calibration);
    FakePoseSource poses;
    poses.initialized = true;
    poses.poses = tPoseDevices(env.rig);
    FakeGestureSource gesture;
    gesture.initialized = true;
    gesture.gesture = true;
    link::Logger logger;  // no sink installed

    EXPECT_NO_THROW(pollAndUpdate(controller, env.rig, nullptr, env.recorder.recorder,
                                  poses, &gesture, 1.0, logger));
    EXPECT_EQ(controller.mode(), Mode::Capture);
}

// --- retargetAndShip -------------------------------------------------------

TEST(FrameTick, RetargetAndShipSendsOffsetsWhenConnected)
{
    TickEnv env = makeEnv();
    CalibratedRig cal = calibrateAtRest(env.rig);
    env.rig.solve();  // pose the skeleton so retargetPose has a pose to read
    FakePoseSource poses;
    poses.initialized = true;

    const std::vector<CorrectedPose> corrected =
        retargetAndShip(env.rig, env.avatar, env.retargetMap, env.correctionMap,
                        cal.calibration, cal.devices, poses);

    EXPECT_EQ(poses.sendCount, 1);
    // Every corrected tracker produces one offset.
    EXPECT_EQ(poses.lastSent.size(), corrected.size());
    EXPECT_FALSE(corrected.empty());
}

TEST(FrameTick, RetargetAndShipSkipsSendWhenDisconnected)
{
    TickEnv env = makeEnv();
    CalibratedRig cal = calibrateAtRest(env.rig);
    env.rig.solve();
    FakePoseSource poses;
    poses.initialized = false;

    const std::vector<CorrectedPose> corrected =
        retargetAndShip(env.rig, env.avatar, env.retargetMap, env.correctionMap,
                        cal.calibration, cal.devices, poses);

    // Corrected poses are still produced (for rendering), but nothing ships.
    EXPECT_EQ(poses.sendCount, 0);
    EXPECT_FALSE(corrected.empty());
}

TEST(FrameTick, RetargetAndShipEmptyWhenUncalibrated)
{
    TickEnv env = makeEnv();
    TrackerCalibration empty;  // not calibrated
    const std::vector<TrackedDevice> devices = tPoseDevices(env.rig);
    env.rig.solve();
    FakePoseSource poses;
    poses.initialized = true;

    const std::vector<CorrectedPose> corrected =
        retargetAndShip(env.rig, env.avatar, env.retargetMap, env.correctionMap,
                        empty, devices, poses);

    EXPECT_TRUE(corrected.empty());
    // The ship call still goes through (empty vector) — the driver loop is a
    // no-op, but the connection is pumped for the upstream direction.
    EXPECT_EQ(poses.sendCount, 1);
    EXPECT_TRUE(poses.lastSent.empty());
}
} // namespace
