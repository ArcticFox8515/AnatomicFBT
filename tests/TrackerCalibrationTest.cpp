#include <gtest/gtest.h>

#include <glm/gtc/epsilon.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>

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
