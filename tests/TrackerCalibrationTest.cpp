#include <gtest/gtest.h>

#include <glm/gtc/epsilon.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>

#include "model/IkRig.h"
#include "model/IkRigConfig.h"
#include "model/Skeleton.h"
#include "model/TrackedDevice.h"
#include "model/TrackerCalibration.h"

namespace
{
Pose makePose(glm::vec3 position, glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f))
{
    return {position, rotation};
}

void expectPoseNear(const Pose& actual, const Pose& expected, float eps = 1e-5f)
{
    EXPECT_TRUE(glm::all(glm::epsilonEqual(actual.position, expected.position, eps)))
        << "position (" << actual.position.x << ", " << actual.position.y << ", "
        << actual.position.z << ")";
    // q and -q are the same rotation, so compare via the dot product.
    const float dot = glm::dot(actual.rotation, expected.rotation);
    EXPECT_NEAR(std::abs(dot), 1.0f, eps)
        << "rotation (" << actual.rotation.w << ", " << actual.rotation.x << ", "
        << actual.rotation.y << ", " << actual.rotation.z << ")";
}

IkRig makeRig()
{
    IkRig rig(Skeleton::makeDefault());
    rig.loadConfig(IkRigConfig::makeDefault());
    return rig;
}

int rootJointIndex(const Skeleton& skeleton)
{
    for (size_t i = 0; i < skeleton.joints.size(); ++i)
        if (!skeleton.joints[i].parentIndex)
            return static_cast<int>(i);
    return -1;
}
} // namespace

TEST(PoseComposeInverse, ComposeAppliesBThenA)
{
    const Pose a = makePose({1.0f, 2.0f, 3.0f}, glm::angleAxis(glm::half_pi<float>(), glm::vec3(0, 1, 0)));
    const Pose b = makePose({0.5f, 0.0f, 0.0f}, glm::angleAxis(glm::half_pi<float>(), glm::vec3(1, 0, 0)));

    const Pose result = compose(a, b);

    // a's yaw maps +X to -Z, so b's translation lands at a.position + (0,0,-0.5).
    expectPoseNear(result, makePose({1.0f, 2.0f, 2.5f}, a.rotation * b.rotation));
}

TEST(PoseComposeInverse, ComposeWithInverseIsIdentity)
{
    const Pose p = makePose({-0.3f, 1.7f, 0.2f},
                            glm::normalize(glm::quat(0.8f, 0.1f, -0.5f, 0.3f)));

    expectPoseNear(compose(inverse(p), p), Pose{});
}

TEST(PoseYawOnly, KeepsHeadingStripsPitchAndRoll)
{
    const float yaw = glm::radians(35.0f);
    const glm::quat tilted = glm::angleAxis(yaw, glm::vec3(0, 1, 0))
        * glm::angleAxis(glm::radians(40.0f), glm::vec3(1, 0, 0))
        * glm::angleAxis(glm::radians(-25.0f), glm::vec3(0, 0, 1));

    const glm::quat result = yawOnly(tilted);

    const glm::vec3 forward = result * glm::vec3(0, 0, -1);
    EXPECT_NEAR(forward.y, 0.0f, 1e-5f);
    const glm::vec3 up = result * glm::vec3(0, 1, 0);
    EXPECT_NEAR(up.x, 0.0f, 1e-5f);
    EXPECT_NEAR(up.z, 0.0f, 1e-5f);
    // Heading preserved: forward matches the tilted forward's horizontal direction.
    const glm::vec3 tiltedForward = tilted * glm::vec3(0, 0, -1);
    EXPECT_NEAR(forward.x, glm::normalize(glm::vec2(tiltedForward.x, tiltedForward.z)).x, 1e-5f);
    EXPECT_NEAR(forward.z, glm::normalize(glm::vec2(tiltedForward.x, tiltedForward.z)).y, 1e-5f);
}

TEST(PoseYawOnly, LookingStraightDownYieldsIdentity)
{
    const glm::quat down = glm::angleAxis(-glm::half_pi<float>(), glm::vec3(1, 0, 0));

    expectPoseNear(makePose({0, 0, 0}, yawOnly(down)), Pose{});
}

TEST(DeviceAssignment, MatchesNearestPairs)
{
    const std::vector<glm::vec3> devices = {{0.0f, 1.8f, 0.01f}, {0.9f, 1.4f, 0.0f}, {-0.9f, 1.45f, 0.0f}};
    const std::vector<glm::vec3> targets = {{0.0f, 1.7f, 0.0f}, {0.85f, 1.4f, 0.0f}, {-0.85f, 1.4f, 0.0f}};

    const DeviceAssignment a = assignDevicesToTargets(devices, targets);

    EXPECT_EQ(a.deviceIndex, (std::vector<int>{0, 1, 2}));
    EXPECT_EQ(a.targetIndex, (std::vector<int>{0, 1, 2}));
}

TEST(DeviceAssignment, LeavesExtraDevicesAndTargetsUnassigned)
{
    const std::vector<glm::vec3> devices = {{0.0f, 0.0f, 0.0f}};
    const std::vector<glm::vec3> targets = {{0.1f, 0.0f, 0.0f}, {5.0f, 0.0f, 0.0f}};

    const DeviceAssignment a = assignDevicesToTargets(devices, targets);

    EXPECT_EQ(a.deviceIndex, (std::vector<int>{0, -1}));
    EXPECT_EQ(a.targetIndex, (std::vector<int>{0}));
}

TEST(DeviceAssignment, EmptyInputs)
{
    const DeviceAssignment a = assignDevicesToTargets({}, {{1.0f, 0.0f, 0.0f}});

    EXPECT_EQ(a.deviceIndex, (std::vector<int>{-1}));
    EXPECT_TRUE(a.targetIndex.empty());
}

TEST(TrackerCalibrationCalibrate, DevicePosesPlusOffsetsReproduceBonePoses)
{
    const std::vector<std::pair<int, Pose>> devices = {
        {7, makePose({0.05f, 1.75f, 0.02f}, glm::angleAxis(0.3f, glm::vec3(0, 1, 0)))},
        {3, makePose({0.9f, 1.4f, -0.1f}, glm::angleAxis(-0.7f, glm::vec3(1, 0, 0)))}};
    const std::vector<Pose> bones = {
        makePose({0.0f, 1.7f, 0.0f}),
        makePose({0.85f, 1.4f, 0.0f}, glm::angleAxis(0.2f, glm::vec3(0, 0, 1)))};

    TrackerCalibration calibration;
    DeviceAssignment assignment;
    assignment.deviceIndex = {7, 3};
    assignment.targetIndex = {0, 1};
    calibration.calibrate(assignment, devices, bones);

    std::vector<IkTarget> targets(2);
    EXPECT_EQ(calibration.applyDevicePoses(devices, targets), 2u);
    // Targets hold the raw device poses (what gets rendered)...
    expectPoseNear({targets[0].position, targets[0].rotation}, devices[0].second);
    expectPoseNear({targets[1].position, targets[1].rotation}, devices[1].second);
    // ...and offsets transform them into the solver goals (the bone poses).
    calibration.applyOffsets(targets);
    expectPoseNear({targets[0].position, targets[0].rotation}, bones[0]);
    expectPoseNear({targets[1].position, targets[1].rotation}, bones[1]);
}

TEST(TrackerCalibrationApply, GoalFollowsDeviceRigidly)
{
    TrackerCalibration calibration;
    DeviceAssignment assignment;
    assignment.deviceIndex = {5};
    assignment.targetIndex = {0};
    calibration.calibrate(assignment, {{5, makePose({1.0f, 1.0f, 0.0f})}},
                          {makePose({1.1f, 1.0f, 0.2f})});

    const std::vector<std::pair<int, Pose>> moved = {
        {5, makePose({-0.5f, 1.2f, 0.4f}, glm::angleAxis(glm::half_pi<float>(), glm::vec3(0, 1, 0)))}};
    std::vector<IkTarget> targets(1);
    calibration.applyDevicePoses(moved, targets);
    expectPoseNear({targets[0].position, targets[0].rotation}, moved[0].second);
    calibration.applyOffsets(targets);

    // Offset was (+0.1, 0, +0.2) in the device frame; a 90-degree yaw maps
    // device +X to world -Z and device +Z to world +X.
    expectPoseNear({targets[0].position, targets[0].rotation},
                   makePose({-0.3f, 1.2f, 0.3f}, moved[0].second.rotation));
}

TEST(TrackerCalibrationApply, MissingDeviceLeavesTargetUntouched)
{
    TrackerCalibration calibration;
    DeviceAssignment assignment;
    assignment.deviceIndex = {5};
    assignment.targetIndex = {0};
    calibration.calibrate(assignment, {{5, makePose({0.0f, 1.0f, 0.0f})}}, {makePose({0.0f, 1.0f, 0.0f})});

    IkTarget target;
    target.position = {3.0f, 3.0f, 3.0f};
    std::vector<IkTarget> targets = {target};

    EXPECT_EQ(calibration.applyDevicePoses({}, targets), 0u);
    EXPECT_EQ(targets[0].position, glm::vec3(3.0f, 3.0f, 3.0f));
}

TEST(TrackerCalibrationApply, UnassignedTargetStaysUnbound)
{
    TrackerCalibration calibration;
    DeviceAssignment assignment;
    assignment.deviceIndex = {5, -1};  // target 1 has no device
    assignment.targetIndex = {0};
    calibration.calibrate(assignment, {{5, makePose({0.0f, 1.0f, 0.0f})}},
                          {makePose({0.0f, 1.0f, 0.0f}), makePose({5.0f, 0.0f, 0.0f})});

    EXPECT_TRUE(calibration.isCalibrated());
    EXPECT_TRUE(calibration.boundDevice(0).has_value());
    EXPECT_FALSE(calibration.boundDevice(1).has_value());

    std::vector<IkTarget> targets(2);
    EXPECT_EQ(calibration.applyDevicePoses({{5, makePose({0.0f, 2.0f, 0.0f})}}, targets), 1u);
    calibration.applyOffsets(targets);
    EXPECT_EQ(targets[0].position.y, 2.0f);
    EXPECT_EQ(targets[1].position, glm::vec3(0.0f, 0.0f, 0.0f));
}

TEST(TrackerCalibrationClear, DropsAllBindings)
{
    TrackerCalibration calibration;
    DeviceAssignment assignment;
    assignment.deviceIndex = {5};
    assignment.targetIndex = {0};
    calibration.calibrate(assignment, {{5, makePose({0.0f, 1.0f, 0.0f})}}, {makePose({0.0f, 1.0f, 0.0f})});
    ASSERT_TRUE(calibration.isCalibrated());

    calibration.clear();

    EXPECT_FALSE(calibration.isCalibrated());
    std::vector<IkTarget> targets(1);
    EXPECT_EQ(calibration.applyDevicePoses({{5, makePose({0.0f, 1.0f, 0.0f})}}, targets), 0u);
}

TEST(CalibrationFrame, NoDevicesResetsPoseAndLeavesEverythingElse)
{
    IkRig rig = makeRig();
    rig.skeleton.joints[1].localRot = glm::angleAxis(0.5f, glm::vec3(1, 0, 0));
    rig.skeleton.rootPosition = {1.0f, 2.0f, 3.0f};
    const std::vector<IkTarget> targetsBefore = rig.targets;

    const CalibrationFrame frame = updateCalibrationFrame(rig, {});

    for (const Joint& joint : rig.skeleton.joints)
        EXPECT_EQ(joint.localRot, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    // No HMD: the root position stays where it was.
    EXPECT_EQ(rig.skeleton.rootPosition, glm::vec3(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(frame.assignment.deviceIndex,
              std::vector<int>(rig.targets.size(), -1));
    ASSERT_EQ(frame.boneWorldPoses.size(), rig.targets.size());
    // No device mirrored anything into the targets.
    for (size_t i = 0; i < rig.targets.size(); ++i)
        expectPoseNear({rig.targets[i].position, rig.targets[i].rotation},
                       {targetsBefore[i].position, targetsBefore[i].rotation});
}

TEST(CalibrationFrame, HmdAlignsRootPositionAndHeading)
{
    IkRig rig = makeRig();
    const int root = rootJointIndex(rig.skeleton);
    ASSERT_GE(root, 0);

    // 90-degree yaw plus some pitch: the pitch must be stripped.
    const glm::quat hmdRot = glm::angleAxis(glm::half_pi<float>(), glm::vec3(0, 1, 0))
        * glm::angleAxis(0.4f, glm::vec3(1, 0, 0));
    const std::vector<TrackedDevice> devices = {
        {0, TrackedDeviceKind::Hmd, {{1.0f, 1.65f, -2.0f}, hmdRot}}};

    const CalibrationFrame frame = updateCalibrationFrame(rig, devices);

    EXPECT_EQ(rig.skeleton.rootPosition, glm::vec3(1.0f, 1.65f, -2.0f));
    expectPoseNear({{0, 0, 0}, rig.skeleton.joints[root].localRot}, {{0, 0, 0}, yawOnly(hmdRot)});
    for (size_t i = 0; i < rig.skeleton.joints.size(); ++i)
        if (static_cast<int>(i) != root)
            EXPECT_EQ(rig.skeleton.joints[i].localRot, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));

    // Reported bone world poses match FK of the aligned skeleton.
    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    ASSERT_EQ(frame.boneWorldPoses.size(), rig.targets.size());
    for (size_t i = 0; i < rig.targets.size(); ++i)
    {
        const int j = rig.targets[i].jointIndex;
        expectPoseNear(frame.boneWorldPoses[i], {wt.positions[j], wt.rotations[j]});
    }
}

TEST(CalibrationFrame, MirrorsAssignedDevicePosesIntoTargets)
{
    IkRig rig = makeRig();
    const WorldTransforms rest = computeWorldTransforms(rig.skeleton);

    // Perfect T-pose: one device exactly on each target's rest position.
    std::vector<TrackedDevice> devices;
    for (size_t i = 0; i < rig.targets.size(); ++i)
    {
        devices.push_back({static_cast<int>(100 + i), TrackedDeviceKind::Tracker,
                           {rest.positions[rig.targets[i].jointIndex],
                            glm::angleAxis(0.1f * static_cast<float>(i), glm::vec3(0, 1, 0))}});
    }

    const CalibrationFrame frame = updateCalibrationFrame(rig, devices);

    // Each device is closest to its own target: identity assignment, and
    // every target mirrors its device's raw pose.
    for (size_t i = 0; i < rig.targets.size(); ++i)
    {
        EXPECT_EQ(frame.assignment.deviceIndex[i], static_cast<int>(i)) << "target " << i;
        expectPoseNear({rig.targets[i].position, rig.targets[i].rotation}, devices[i].pose);
    }
}

TEST(CalibrationFrame, UnmatchedTargetKeepsItsPose)
{
    IkRig rig = makeRig();
    const WorldTransforms rest = computeWorldTransforms(rig.skeleton);
    const std::vector<IkTarget> targetsBefore = rig.targets;

    const std::vector<TrackedDevice> devices = {
        {42, TrackedDeviceKind::Tracker,
         {rest.positions[rig.targets[0].jointIndex], glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}}};

    const CalibrationFrame frame = updateCalibrationFrame(rig, devices);

    ASSERT_EQ(frame.assignment.deviceIndex.size(), rig.targets.size());
    EXPECT_EQ(frame.assignment.deviceIndex[0], 0);
    expectPoseNear({rig.targets[0].position, rig.targets[0].rotation}, devices[0].pose);
    for (size_t i = 1; i < rig.targets.size(); ++i)
    {
        EXPECT_EQ(frame.assignment.deviceIndex[i], -1) << "target " << i;
        expectPoseNear({rig.targets[i].position, rig.targets[i].rotation},
                       {targetsBefore[i].position, targetsBefore[i].rotation});
    }
}

TEST(CalibrationFrame, CaptureOffsetsBindsStableIdsAndReproducesBonePoses)
{
    IkRig rig = makeRig();
    const WorldTransforms rest = computeWorldTransforms(rig.skeleton);

    std::vector<TrackedDevice> devices;
    for (size_t i = 0; i < rig.targets.size(); ++i)
    {
        devices.push_back({static_cast<int>(100 + i), TrackedDeviceKind::Tracker,
                           {rest.positions[rig.targets[i].jointIndex],
                            glm::angleAxis(0.1f * static_cast<float>(i), glm::vec3(0, 1, 0))}});
    }
    const CalibrationFrame frame = updateCalibrationFrame(rig, devices);

    TrackerCalibration calibration;
    captureOffsets(calibration, frame, devices);

    // Targets are bound to the devices' stable ids, not list positions.
    for (size_t i = 0; i < rig.targets.size(); ++i)
    {
        ASSERT_TRUE(calibration.boundDevice(i).has_value()) << "target " << i;
        EXPECT_EQ(*calibration.boundDevice(i), static_cast<int>(100 + i));
    }
    // Offsets transform the raw device poses (already mirrored into the
    // targets by updateCalibrationFrame) back onto the bone poses.
    std::vector<IkTarget> goals = rig.targets;
    calibration.applyOffsets(goals);
    for (size_t i = 0; i < goals.size(); ++i)
        expectPoseNear({goals[i].position, goals[i].rotation}, frame.boneWorldPoses[i]);
}

TEST(CaptureFrame, GoalsAreOffsetCopiesOfRawDevicePoses)
{
    IkRig rig = makeRig();
    const WorldTransforms rest = computeWorldTransforms(rig.skeleton);

    // Bone poses at rest (identity rotations).
    std::vector<Pose> bonePoses(rig.targets.size());
    for (size_t i = 0; i < rig.targets.size(); ++i)
        bonePoses[i] = {rest.positions[rig.targets[i].jointIndex], glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};

    // Bind targets 0 and 2 to devices sitting off the bones (non-trivial offsets).
    DeviceAssignment assignment;
    assignment.deviceIndex.assign(rig.targets.size(), -1);
    assignment.deviceIndex[0] = 11;
    assignment.deviceIndex[2] = 22;
    const Pose calibA = makePose(bonePoses[0].position + glm::vec3(0.03f, -0.05f, 0.02f),
                                 glm::angleAxis(0.4f, glm::vec3(0, 1, 0)));
    const Pose calibB = makePose(bonePoses[2].position + glm::vec3(-0.02f, 0.01f, 0.04f),
                                 glm::angleAxis(-0.3f, glm::vec3(1, 0, 0)));
    TrackerCalibration calibration;
    calibration.calibrate(assignment, {{11, calibA}, {22, calibB}}, bonePoses);

    // Capture frame: the devices moved elsewhere.
    const Pose movedA = makePose({0.4f, 1.2f, -0.3f}, glm::angleAxis(0.9f, glm::vec3(0, 1, 0)));
    const Pose movedB = makePose({-0.5f, 0.8f, 0.2f}, glm::angleAxis(0.2f, glm::vec3(0, 0, 1)));
    const std::vector<TrackedDevice> devices = {
        {11, TrackedDeviceKind::Tracker, movedA},
        {22, TrackedDeviceKind::Tracker, movedB}};

    const std::vector<IkTarget> goals = updateCaptureFrame(rig, calibration, devices);

    ASSERT_EQ(goals.size(), rig.targets.size());
    // Targets mirror the raw device poses (what gets rendered)...
    expectPoseNear({rig.targets[0].position, rig.targets[0].rotation}, movedA);
    expectPoseNear({rig.targets[2].position, rig.targets[2].rotation}, movedB);
    // ...while goals carry the calibrated offsets: goal = moved * offset with
    // offset = inverse(calibrationPose) * bonePose.
    expectPoseNear({goals[0].position, goals[0].rotation},
                   compose(movedA, compose(inverse(calibA), bonePoses[0])));
    expectPoseNear({goals[2].position, goals[2].rotation},
                   compose(movedB, compose(inverse(calibB), bonePoses[2])));
    // Unbound targets pass through untouched.
    for (size_t i = 0; i < rig.targets.size(); ++i)
    {
        if (i == 0 || i == 2)
            continue;
        expectPoseNear({goals[i].position, goals[i].rotation},
                       {rig.targets[i].position, rig.targets[i].rotation});
    }
}

TEST(CaptureFrame, UncalibratedPassesTargetsThrough)
{
    IkRig rig = makeRig();
    const TrackerCalibration calibration;  // never calibrated
    const std::vector<IkTarget> targetsBefore = rig.targets;

    const std::vector<TrackedDevice> devices = {
        {5, TrackedDeviceKind::Tracker, makePose({1.0f, 2.0f, 3.0f})}};

    const std::vector<IkTarget> goals = updateCaptureFrame(rig, calibration, devices);

    ASSERT_EQ(goals.size(), rig.targets.size());
    for (size_t i = 0; i < rig.targets.size(); ++i)
    {
        expectPoseNear({rig.targets[i].position, rig.targets[i].rotation},
                       {targetsBefore[i].position, targetsBefore[i].rotation});
        expectPoseNear({goals[i].position, goals[i].rotation},
                       {targetsBefore[i].position, targetsBefore[i].rotation});
    }
}
