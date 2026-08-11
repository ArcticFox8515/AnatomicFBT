#include <gtest/gtest.h>

#include <glm/gtc/quaternion.hpp>

#include <cmath>

#include "model/GripOffsets.h"

namespace
{
TrackedDevice makeDevice(int id, TrackedDeviceKind kind, const glm::vec3& position,
                         const glm::quat& rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f))
{
    return {id, kind, {position, rotation}};
}

GripOffset makeOffset(int id, const glm::vec3& position,
                      const glm::quat& rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f))
{
    return {id, {position, rotation}};
}

// Knuckles `grip` component_local: origin [0, 0.003, 0.097], rotate_xyz
// [5.037, 0, 0] — the canonical case the whole feature exists to support.
// Rx(5.037 deg): cos ~= 0.99613, sin ~= 0.08793. Returns the angle in radians
// for the caller to build the expected quaternion.
float knucklesGripMatrix(float m[3][4])
{
    const float rad = glm::radians(5.037f);
    const float c = std::cos(rad);
    const float s = std::sin(rad);
    m[0][0] = 1.0f; m[0][1] = 0.0f; m[0][2] = 0.0f; m[0][3] = 0.0f;
    m[1][0] = 0.0f; m[1][1] = c;     m[1][2] = -s;    m[1][3] = 0.003f;
    m[2][0] = 0.0f; m[2][1] = s;     m[2][2] = c;     m[2][3] = 0.097f;
    return rad;
}
} // namespace

// --- poseFromHmdMatrix34 ----------------------------------------------------

TEST(GripOffsets, PoseFromHmdMatrix34MatchesKnucklesGripComponent)
{
    float m[3][4] = {};
    const float rad = knucklesGripMatrix(m);

    const Pose pose = poseFromHmdMatrix34(m);

    // Translation is the last column, verbatim.
    EXPECT_FLOAT_EQ(pose.position.x, 0.0f);
    EXPECT_FLOAT_EQ(pose.position.y, 0.003f);
    EXPECT_FLOAT_EQ(pose.position.z, 0.097f);

    // Rotation is Rx(5.037 deg). Compare against glm's own axis-angle.
    const glm::quat expected = glm::angleAxis(rad, glm::vec3(1.0f, 0.0f, 0.0f));
    EXPECT_NEAR(pose.rotation.x, expected.x, 1e-5f);
    EXPECT_NEAR(pose.rotation.y, expected.y, 1e-5f);
    EXPECT_NEAR(pose.rotation.z, expected.z, 1e-5f);
    EXPECT_NEAR(pose.rotation.w, expected.w, 1e-5f);
}

TEST(GripOffsets, PoseFromHmdMatrix34IdentityForIdentityMatrix)
{
    float m[3][4] = {};
    m[0][0] = m[1][1] = m[2][2] = 1.0f;
    m[0][3] = m[1][3] = m[2][3] = 0.0f;

    const Pose pose = poseFromHmdMatrix34(m);

    EXPECT_EQ(pose.position, glm::vec3(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(pose.rotation, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
}

TEST(GripOffsets, PoseFromHmdMatrix34TranslationOnly)
{
    float m[3][4] = {};
    m[0][0] = m[1][1] = m[2][2] = 1.0f;
    m[0][3] = 0.1f; m[1][3] = -0.2f; m[2][3] = 0.3f;

    const Pose pose = poseFromHmdMatrix34(m);

    EXPECT_EQ(pose.position, glm::vec3(0.1f, -0.2f, 0.3f));
    EXPECT_EQ(pose.rotation, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
}

// --- applyGripOffsets -------------------------------------------------------

TEST(GripOffsets, ApplyComposesControllerPoseWithOffset)
{
    const std::vector<TrackedDevice> devices = {
        makeDevice(0, TrackedDeviceKind::Hmd, {0.0f, 1.7f, 0.0f}),
        makeDevice(3, TrackedDeviceKind::Controller, {1.0f, 0.0f, 0.0f}),
    };
    const std::vector<GripOffset> offsets = {
        makeOffset(3, {0.0f, 0.003f, 0.097f}),
    };

    const std::vector<TrackedDevice> result = applyGripOffsets(devices, offsets);

    ASSERT_EQ(result.size(), 2u);
    // HMD untouched.
    EXPECT_EQ(result[0].pose.position, glm::vec3(0.0f, 1.7f, 0.0f));
    // Controller shifted by the offset in the controller's local frame
    // (identity rotation here, so a plain translation).
    EXPECT_FLOAT_EQ(result[1].pose.position.x, 1.0f);
    EXPECT_FLOAT_EQ(result[1].pose.position.y, 0.003f);
    EXPECT_FLOAT_EQ(result[1].pose.position.z, 0.097f);
}

TEST(GripOffsets, ApplyRotatesOffsetByDeviceRotation)
{
    // Controller rotated 90 deg about Y (its local +Z becomes world +X), with
    // a grip offset of [0, 0, 0.1] in the controller's local frame. The grip
    // point should land at world [0.1, 0, 0].
    const glm::quat rotY = glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.0f, 1.0f, 0.0f));
    const std::vector<TrackedDevice> devices = {
        makeDevice(3, TrackedDeviceKind::Controller, {0.0f, 0.0f, 0.0f}, rotY),
    };
    const std::vector<GripOffset> offsets = {
        makeOffset(3, {0.0f, 0.0f, 0.1f}),
    };

    const std::vector<TrackedDevice> result = applyGripOffsets(devices, offsets);

    ASSERT_EQ(result.size(), 1u);
    EXPECT_NEAR(result[0].pose.position.x, 0.1f, 1e-6f);
    EXPECT_NEAR(result[0].pose.position.y, 0.0f, 1e-6f);
    EXPECT_NEAR(result[0].pose.position.z, 0.0f, 1e-6f);
}

TEST(GripOffsets, ApplyCarriesTheOffsetRotationIntoTheShiftedPose)
{
    // The grip offset's own rotation must compose onto the device rotation,
    // not be dropped. Knuckles grip = Rx(5.037 deg); with an identity-rotation
    // device, the shifted pose carries that rotation exactly.
    float m[3][4] = {};
    knucklesGripMatrix(m);
    const Pose deviceToGrip = poseFromHmdMatrix34(m);
    const std::vector<TrackedDevice> devices = {
        makeDevice(3, TrackedDeviceKind::Controller, {0.0f, 0.0f, 0.0f}),
    };
    const std::vector<GripOffset> offsets = {{3, deviceToGrip}};

    const std::vector<TrackedDevice> result = applyGripOffsets(devices, offsets);

    ASSERT_EQ(result.size(), 1u);
    EXPECT_NEAR(result[0].pose.rotation.x, deviceToGrip.rotation.x, 1e-6f);
    EXPECT_NEAR(result[0].pose.rotation.y, deviceToGrip.rotation.y, 1e-6f);
    EXPECT_NEAR(result[0].pose.rotation.z, deviceToGrip.rotation.z, 1e-6f);
    EXPECT_NEAR(result[0].pose.rotation.w, deviceToGrip.rotation.w, 1e-6f);
}

TEST(GripOffsets, ApplyLeavesTrackerAndOtherDevicesUntouched)
{
    const std::vector<TrackedDevice> devices = {
        makeDevice(0, TrackedDeviceKind::Hmd, {0.0f, 1.7f, 0.0f}),
        makeDevice(5, TrackedDeviceKind::Tracker, {0.1f, 0.1f, 0.05f}),
        makeDevice(9, TrackedDeviceKind::Other, {2.0f, 2.0f, 2.0f}),
        makeDevice(3, TrackedDeviceKind::Controller, {1.0f, 0.0f, 0.0f}),
    };
    const std::vector<GripOffset> offsets = {
        makeOffset(3, {0.0f, 0.0f, 0.1f}),
    };

    const std::vector<TrackedDevice> result = applyGripOffsets(devices, offsets);

    ASSERT_EQ(result.size(), 4u);
    EXPECT_EQ(result[0].pose.position, glm::vec3(0.0f, 1.7f, 0.0f));
    EXPECT_EQ(result[1].pose.position, glm::vec3(0.1f, 0.1f, 0.05f));
    EXPECT_EQ(result[2].pose.position, glm::vec3(2.0f, 2.0f, 2.0f));
    EXPECT_NEAR(result[3].pose.position.z, 0.1f, 1e-6f);
}

TEST(GripOffsets, ApplyUnknownIdPassesThrough)
{
    const std::vector<TrackedDevice> devices = {
        makeDevice(3, TrackedDeviceKind::Controller, {1.0f, 0.0f, 0.0f}),
    };
    // Offset for a device that is not in the snapshot this frame.
    const std::vector<GripOffset> offsets = {
        makeOffset(7, {0.0f, 0.0f, 0.1f}),
    };

    const std::vector<TrackedDevice> result = applyGripOffsets(devices, offsets);

    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0].pose.position, glm::vec3(1.0f, 0.0f, 0.0f));
}

TEST(GripOffsets, ApplyEmptyOffsetsIsIdentity)
{
    const std::vector<TrackedDevice> devices = {
        makeDevice(3, TrackedDeviceKind::Controller, {1.0f, 0.0f, 0.0f}),
        makeDevice(5, TrackedDeviceKind::Tracker, {0.0f, 0.0f, 0.0f}),
    };

    const std::vector<TrackedDevice> result = applyGripOffsets(devices, {});

    ASSERT_EQ(result.size(), 2u);
    EXPECT_EQ(result[0].pose.position, glm::vec3(1.0f, 0.0f, 0.0f));
    EXPECT_EQ(result[1].pose.position, glm::vec3(0.0f, 0.0f, 0.0f));
}

TEST(GripOffsets, ApplyEmptyDevicesIsEmpty)
{
    EXPECT_TRUE(applyGripOffsets({}, {makeOffset(3, {0.0f, 0.0f, 0.1f})}).empty());
}

// --- mergeGripOffsets -------------------------------------------------------

TEST(GripOffsets, MergeFreshOverwritesCachedForSameId)
{
    const std::vector<GripOffset> cached = {
        makeOffset(3, {0.0f, 0.0f, 0.1f}),
        makeOffset(7, {0.0f, 0.0f, 0.2f}),
    };
    const std::vector<GripOffset> fresh = {
        makeOffset(3, {0.0f, 0.0f, 0.3f}),
    };

    const std::vector<GripOffset> result = mergeGripOffsets(cached, fresh);

    ASSERT_EQ(result.size(), 2u);
    EXPECT_EQ(result[0].deviceId, 3);
    EXPECT_FLOAT_EQ(result[0].deviceToGrip.position.z, 0.3f);  // fresh
    EXPECT_EQ(result[1].deviceId, 7);
    EXPECT_FLOAT_EQ(result[1].deviceToGrip.position.z, 0.2f);  // cached kept
}

TEST(GripOffsets, MergeKeepsCachedWhenFreshIsEmpty)
{
    const std::vector<GripOffset> cached = {
        makeOffset(3, {0.0f, 0.0f, 0.1f}),
    };

    const std::vector<GripOffset> result = mergeGripOffsets(cached, {});

    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0].deviceId, 3);
}

TEST(GripOffsets, MergeFreshOnlyWhenCachedIsEmpty)
{
    const std::vector<GripOffset> fresh = {
        makeOffset(3, {0.0f, 0.0f, 0.1f}),
        makeOffset(7, {0.0f, 0.0f, 0.2f}),
    };

    const std::vector<GripOffset> result = mergeGripOffsets({}, fresh);

    EXPECT_EQ(result.size(), fresh.size());
    EXPECT_EQ(result[0].deviceId, 3);
    EXPECT_EQ(result[1].deviceId, 7);
}

TEST(GripOffsets, MergeBothEmptyIsEmpty)
{
    EXPECT_TRUE(mergeGripOffsets({}, {}).empty());
}
