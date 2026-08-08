#include <gtest/gtest.h>

#include <glm/gtc/epsilon.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>

#include "model/BoneNames.h"
#include "model/IkRig.h"
#include "model/IkRigConfig.h"
#include "model/Pose.h"
#include "model/Retarget.h"
#include "model/Skeleton.h"
#include "model/TrackedDevice.h"
#include "model/TrackerCalibration.h"
#include "model/TrackerCorrection.h"

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

int findJointIndex(const Skeleton& skeleton, const char* name)
{
    for (size_t i = 0; i < skeleton.joints.size(); ++i)
        if (skeleton.joints[i].name == name)
            return static_cast<int>(i);
    return -1;
}

int findTargetIndex(const IkRig& rig, const char* jointName)
{
    const int joint = findJointIndex(rig.skeleton, jointName);
    for (size_t t = 0; t < rig.targets.size(); ++t)
        if (rig.targets[t].jointIndex == joint)
            return static_cast<int>(t);
    return -1;
}

// Calibrates every target against a device placed exactly on its rest bone
// pose (identity rotation), so deviceInBone == identity for every binding.
// Returns the calibration and the device list; the rig is left in the
// calibration rest pose.
struct RestCalibration
{
    TrackerCalibration calibration;
    std::vector<TrackedDevice> devices;
};

RestCalibration calibrateAtRest(IkRig& rig)
{
    const WorldTransforms rest = computeWorldTransforms(rig.skeleton);
    std::vector<TrackedDevice> devices;
    for (size_t i = 0; i < rig.targets.size(); ++i)
    {
        devices.push_back({static_cast<int>(100 + i), TrackedDeviceKind::Tracker,
                           {rest.positions[rig.targets[i].jointIndex],
                            glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}});
    }
    const CalibrationFrame frame = updateCalibrationFrame(rig, devices);
    TrackerCalibration calibration;
    captureOffsets(calibration, frame, devices);
    return {calibration, devices};
}
} // namespace

TEST(BuildCorrectionMap, MatchesTargetsByNameAndEnablesThem)
{
    IkRig rig = makeRig();
    Skeleton avatar = Skeleton::makeDefaultHipRooted();

    const CorrectionMap map = buildCorrectionMap(rig, avatar);

    ASSERT_EQ(map.avatarJoint.size(), rig.targets.size());
    ASSERT_EQ(map.enabled.size(), rig.targets.size());
    const int headTarget = findTargetIndex(rig, BoneNames::Head);
    ASSERT_GE(headTarget, 0);
    for (size_t t = 0; t < rig.targets.size(); ++t)
    {
        if (static_cast<int>(t) == headTarget)
        {
            // Head is the HMD (anchor), never a tracker — skipped outright.
            EXPECT_FALSE(map.avatarJoint[t].has_value()) << "Head should be skipped";
            EXPECT_FALSE(map.enabled[t]) << "Head should be disabled";
            continue;
        }
        ASSERT_TRUE(map.avatarJoint[t].has_value()) << "target " << t;
        const std::string& name = rig.skeleton.joints[rig.targets[t].jointIndex].name;
        EXPECT_EQ(avatar.joints[*map.avatarJoint[t]].name, name);
        EXPECT_TRUE(map.enabled[t]) << "target " << t << " should start enabled";
    }
}

TEST(BuildCorrectionMap, HeadAnchorIsNeverCorrectable)
{
    IkRig rig = makeRig();
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    const CorrectionMap map = buildCorrectionMap(rig, avatar);
    const int headTarget = findTargetIndex(rig, BoneNames::Head);
    ASSERT_GE(headTarget, 0);
    EXPECT_FALSE(map.avatarJoint[static_cast<size_t>(headTarget)].has_value());
    EXPECT_FALSE(map.enabled[static_cast<size_t>(headTarget)]);
}

TEST(BuildCorrectionMap, UnmatchedTargetIsNeverEnabled)
{
    IkRig rig = makeRig();
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    // Break the LeftFoot name so that target has no avatar counterpart.
    avatar.joints[findJointIndex(avatar, BoneNames::LeftFoot)].name = "LeftFootX";

    const CorrectionMap map = buildCorrectionMap(rig, avatar);

    const int leftFootTarget = findTargetIndex(rig, BoneNames::LeftFoot);
    ASSERT_GE(leftFootTarget, 0);
    EXPECT_FALSE(map.avatarJoint[static_cast<size_t>(leftFootTarget)].has_value());
    EXPECT_FALSE(map.enabled[static_cast<size_t>(leftFootTarget)]);

    const int headTarget = findTargetIndex(rig, BoneNames::Head);
    // Every other target (except Head + the unmatched LeftFoot) is mapped.
    for (size_t t = 0; t < rig.targets.size(); ++t)
    {
        if (static_cast<int>(t) == leftFootTarget || static_cast<int>(t) == headTarget)
            continue;
        EXPECT_TRUE(map.avatarJoint[t].has_value()) << "target " << t;
        EXPECT_TRUE(map.enabled[t]) << "target " << t;
    }
}

TEST(CorrectDevicePoses, IdenticalProportionsPlaceTrackersAtBoneCenters)
{
    IkRig rig = makeRig();
    const RestCalibration calib = calibrateAtRest(rig);
    // Same proportions, hip-rooted: rest world poses match the head-rooted src.
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    const RetargetMap retargetMap = buildRetargetMap(rig.skeleton, avatar);
    retargetPose(rig.skeleton, avatar, retargetMap);
    const CorrectionMap map = buildCorrectionMap(rig, avatar);

    const std::vector<CorrectedPose> corrected =
        correctDevicePoses(calib.calibration, map, avatar);
    const WorldTransforms avatarWorld = computeWorldTransforms(avatar);

    // Every bound, enabled, mapped target — minus Head (the HMD anchor).
    const int headTarget = findTargetIndex(rig, BoneNames::Head);
    ASSERT_GE(headTarget, 0);
    EXPECT_EQ(corrected.size(), rig.targets.size() - 1);
    for (const CorrectedPose& c : corrected)
    {
        ASSERT_GE(c.deviceId, 0);
        ASSERT_LT(c.deviceId, 100 + static_cast<int>(rig.targets.size()));
        EXPECT_NE(static_cast<int>(c.targetIndex), headTarget);
        // Corrected pose == the avatar joint's world pose (bone center, no
        // strap offset), NOT the raw device pose.
        const int avatarJoint = *map.avatarJoint[c.targetIndex];
        const Pose expected{avatarWorld.positions[static_cast<size_t>(avatarJoint)],
                             avatarWorld.rotations[static_cast<size_t>(avatarJoint)]};
        expectPoseNear(c.pose, expected, 1e-4f);
    }
}

TEST(CorrectDevicePoses, ScaledAvatarShiftsByAvatarBoneDelta)
{
    IkRig rig = makeRig();
    const RestCalibration calib = calibrateAtRest(rig);
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    for (Joint& joint : avatar.joints)
        if (joint.parentIndex)
            joint.restOffset *= 1.2f;
    const RetargetMap retargetMap = buildRetargetMap(rig.skeleton, avatar);
    retargetPose(rig.skeleton, avatar, retargetMap);
    const CorrectionMap map = buildCorrectionMap(rig, avatar);

    const std::vector<CorrectedPose> corrected =
        correctDevicePoses(calib.calibration, map, avatar);
    const WorldTransforms avatarWorld = computeWorldTransforms(avatar);
    const WorldTransforms srcWorld = computeWorldTransforms(rig.skeleton);

    const int leftFootTarget = findTargetIndex(rig, BoneNames::LeftFoot);
    ASSERT_GE(leftFootTarget, 0);
    auto findCorrected = [&](size_t target)
    {
        for (const CorrectedPose& c : corrected)
            if (c.targetIndex == target)
                return c;
        ADD_FAILURE() << "no corrected pose for target " << target;
        return CorrectedPose{};
    };

    // The corrected pose is the avatar bone center — no strap offset.
    const CorrectedPose foot = findCorrected(static_cast<size_t>(leftFootTarget));
    const int avatarFoot = *map.avatarJoint[static_cast<size_t>(leftFootTarget)];
    const int srcFoot = rig.targets[leftFootTarget].jointIndex;
    EXPECT_TRUE(glm::all(glm::epsilonEqual(
        foot.pose.position, avatarWorld.positions[static_cast<size_t>(avatarFoot)], 1e-4f)));
    // ...which differs from the reference foot by the proportion gap.
    const float footDistance = glm::distance(
        avatarWorld.positions[static_cast<size_t>(avatarFoot)],
        srcWorld.positions[static_cast<size_t>(srcFoot)]);
    EXPECT_GT(footDistance, 0.01f);
}

TEST(CorrectDevicePoses, DisabledTargetIsSkipped)
{
    IkRig rig = makeRig();
    const RestCalibration calib = calibrateAtRest(rig);
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    retargetPose(rig.skeleton, avatar, buildRetargetMap(rig.skeleton, avatar));
    CorrectionMap map = buildCorrectionMap(rig, avatar);
    const int leftFootTarget = findTargetIndex(rig, BoneNames::LeftFoot);
    ASSERT_GE(leftFootTarget, 0);
    map.enabled[static_cast<size_t>(leftFootTarget)] = false;

    const std::vector<CorrectedPose> corrected =
        correctDevicePoses(calib.calibration, map, avatar);

    // Head (anchor, never mapped) + disabled LeftFoot are both absent.
    EXPECT_EQ(corrected.size(), rig.targets.size() - 2);
    for (const CorrectedPose& c : corrected)
        EXPECT_NE(static_cast<int>(c.targetIndex), leftFootTarget);
}

TEST(CorrectDevicePoses, UnboundTargetIsSkipped)
{
    IkRig rig = makeRig();
    // Place devices on every target except LeftFoot, so that target stays
    // unassigned (and unbound) after calibration.
    const WorldTransforms rest = computeWorldTransforms(rig.skeleton);
    std::vector<TrackedDevice> devices;
    for (size_t i = 0; i < rig.targets.size(); ++i)
    {
        if (rig.skeleton.joints[rig.targets[i].jointIndex].name == BoneNames::LeftFoot)
            continue;
        devices.push_back({static_cast<int>(100 + i), TrackedDeviceKind::Tracker,
                           {rest.positions[rig.targets[i].jointIndex], glm::quat(1, 0, 0, 0)}});
    }
    const CalibrationFrame frame = updateCalibrationFrame(rig, devices);
    TrackerCalibration calibration;
    captureOffsets(calibration, frame, devices);
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    retargetPose(rig.skeleton, avatar, buildRetargetMap(rig.skeleton, avatar));
    const CorrectionMap map = buildCorrectionMap(rig, avatar);

    const std::vector<CorrectedPose> corrected = correctDevicePoses(calibration, map, avatar);

    const int leftFootTarget = findTargetIndex(rig, BoneNames::LeftFoot);
    for (const CorrectedPose& c : corrected)
        EXPECT_NE(static_cast<int>(c.targetIndex), leftFootTarget);
}

TEST(CorrectDevicePoses, UncalibratedReturnsEmpty)
{
    IkRig rig = makeRig();
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    const CorrectionMap map = buildCorrectionMap(rig, avatar);

    const TrackerCalibration uncalibrated;
    EXPECT_TRUE(correctDevicePoses(uncalibrated, map, avatar).empty());
}
