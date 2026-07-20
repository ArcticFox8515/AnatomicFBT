#include <gtest/gtest.h>

#include <glm/gtc/epsilon.hpp>
#include <glm/gtc/quaternion.hpp>

#include "model/IkRig.h"
#include "model/IkRigConfig.h"
#include "model/ModeController.h"
#include "model/Skeleton.h"

namespace
{
IkRig makeRig()
{
    IkRig rig(Skeleton::makeDefault());
    rig.loadConfig(IkRigConfig::makeDefault());
    return rig;
}

// One device exactly on each target's rest world position (identity
// rotation) — a perfect T-pose; the calibration offsets are then identity,
// so capture goals equal the raw device poses.
std::vector<TrackedDevice> tPoseDevices(const IkRig& rig)
{
    const WorldTransforms rest = computeWorldTransforms(rig.skeleton);
    std::vector<TrackedDevice> devices;
    for (size_t i = 0; i < rig.targets.size(); ++i)
        devices.push_back({static_cast<int>(100 + i), TrackedDeviceKind::Tracker,
                           {rest.positions[rig.targets[i].jointIndex],
                            glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}});
    return devices;
}

std::vector<TrackedDevice> movedDevices(std::vector<TrackedDevice> devices, const glm::vec3& delta)
{
    for (TrackedDevice& device : devices)
        device.pose.position += delta;
    return devices;
}

void expectPoseNear(const Pose& actual, const Pose& expected, float eps = 1e-5f)
{
    EXPECT_TRUE(glm::all(glm::epsilonEqual(actual.position, expected.position, eps)))
        << "position (" << actual.position.x << ", " << actual.position.y << ", "
        << actual.position.z << ")";
    EXPECT_NEAR(std::abs(glm::dot(actual.rotation, expected.rotation)), 1.0f, eps);
}
} // namespace

TEST(ModeController, ManualPoseSolvesFromTargets)
{
    IkRig rig = makeRig();
    ModeController controller;  // defaults to ManualPose

    const FramePlan plan = controller.update(rig, tPoseDevices(rig), false);

    EXPECT_EQ(controller.mode(), Mode::ManualPose);
    EXPECT_EQ(plan.solve, SolveMode::Targets);
    EXPECT_TRUE(plan.goals.empty());
    EXPECT_FALSE(plan.capturedOffsets);
}

TEST(ModeController, CalibrationWithoutGestureSolvesNothing)
{
    IkRig rig = makeRig();
    ModeController controller(Mode::Calibration);

    const FramePlan plan = controller.update(rig, tPoseDevices(rig), false);

    EXPECT_EQ(controller.mode(), Mode::Calibration);
    EXPECT_EQ(plan.solve, SolveMode::None);
    EXPECT_FALSE(plan.capturedOffsets);
    // The live assignment is exposed for the UI.
    EXPECT_EQ(controller.liveAssignment().deviceIndex.size(), rig.targets.size());
}

// Regression: the calibration->capture transition frame must already produce
// fresh, right-sized goals. Handing IkRig::solve an empty or stale goals
// vector throws — this crashed the app on the first capture frame when the
// goal production lived in the render loop and was skipped on the transition.
TEST(ModeController, CaptureTransitionProducesFreshGoalsOnTheSameFrame)
{
    IkRig rig = makeRig();
    ModeController controller(Mode::Calibration);
    const std::vector<TrackedDevice> devices = tPoseDevices(rig);

    const FramePlan plan = controller.update(rig, devices, true);

    EXPECT_EQ(controller.mode(), Mode::Capture);
    EXPECT_TRUE(plan.capturedOffsets);
    ASSERT_EQ(plan.solve, SolveMode::Goals);
    ASSERT_EQ(plan.goals.size(), rig.targets.size());
    EXPECT_NO_THROW(rig.solve(plan.goals));
    // Identity offsets (perfect T-pose): goals equal the raw device poses.
    for (size_t i = 0; i < devices.size(); ++i)
        expectPoseNear({plan.goals[i].position, plan.goals[i].rotation}, devices[i].pose);
}

TEST(ModeController, CaptureProducesFreshGoalsEveryFrame)
{
    IkRig rig = makeRig();
    ModeController controller(Mode::Calibration);
    controller.update(rig, tPoseDevices(rig), true);  // -> Capture

    const std::vector<TrackedDevice> moved = movedDevices(tPoseDevices(rig), {0.1f, -0.2f, 0.05f});
    const FramePlan plan = controller.update(rig, moved, false);

    ASSERT_EQ(plan.solve, SolveMode::Goals);
    ASSERT_EQ(plan.goals.size(), rig.targets.size());
    for (size_t i = 0; i < moved.size(); ++i)
        expectPoseNear({plan.goals[i].position, plan.goals[i].rotation}, moved[i].pose);
    EXPECT_NO_THROW(rig.solve(plan.goals));
}

TEST(ModeController, CaptureWithEmptyDevicesStillProducesSolvableGoals)
{
    IkRig rig = makeRig();
    ModeController controller(Mode::Calibration);
    controller.update(rig, tPoseDevices(rig), true);  // -> Capture

    // All trackers disconnected: targets keep their last raw poses, and goals
    // are still derived from them this frame.
    const FramePlan plan = controller.update(rig, {}, false);

    ASSERT_EQ(plan.solve, SolveMode::Goals);
    EXPECT_EQ(plan.goals.size(), rig.targets.size());
    EXPECT_NO_THROW(rig.solve(plan.goals));
}

TEST(ModeController, SwitchToCalibrationClearsOffsetsAndRecaptures)
{
    IkRig rig = makeRig();
    ModeController controller(Mode::Calibration);
    controller.update(rig, tPoseDevices(rig), true);  // -> Capture
    ASSERT_EQ(controller.mode(), Mode::Capture);

    controller.switchToCalibration();

    EXPECT_EQ(controller.mode(), Mode::Calibration);
    const FramePlan plan = controller.update(rig, tPoseDevices(rig), false);
    EXPECT_EQ(plan.solve, SolveMode::None);

    // Re-capturing works: a second T-pose + gesture freezes new offsets.
    const FramePlan recapture = controller.update(rig, tPoseDevices(rig), true);
    EXPECT_TRUE(recapture.capturedOffsets);
    EXPECT_EQ(controller.mode(), Mode::Capture);
    ASSERT_EQ(recapture.solve, SolveMode::Goals);
    EXPECT_EQ(recapture.goals.size(), rig.targets.size());
}

TEST(ModeController, GestureIsIgnoredOutsideCalibration)
{
    IkRig rig = makeRig();
    ModeController controller;  // ManualPose
    FramePlan plan = controller.update(rig, tPoseDevices(rig), true);
    EXPECT_EQ(controller.mode(), Mode::ManualPose);
    EXPECT_EQ(plan.solve, SolveMode::Targets);

    controller.switchToCalibration();
    controller.update(rig, tPoseDevices(rig), true);  // -> Capture
    plan = controller.update(rig, tPoseDevices(rig), true);
    EXPECT_EQ(controller.mode(), Mode::Capture);
    EXPECT_FALSE(plan.capturedOffsets);  // no re-capture in Capture mode
    EXPECT_EQ(plan.solve, SolveMode::Goals);
}
