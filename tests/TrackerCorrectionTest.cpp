#include <gtest/gtest.h>

#include <glm/gtc/epsilon.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>

#include <algorithm>

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
        // Hand targets are driven by controllers in real usage — mark them so
        // the controller rotation-lock path is exercised by every test that
        // uses this helper.
        const std::string& name = rig.skeleton.joints[rig.targets[i].jointIndex].name;
        const TrackedDeviceKind kind = (name == BoneNames::LeftHand || name == BoneNames::RightHand)
            ? TrackedDeviceKind::Controller
            : TrackedDeviceKind::Tracker;
        devices.push_back({static_cast<int>(100 + i), kind,
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
    ASSERT_EQ(map.rotationEnabled.size(), rig.targets.size());
    const int headTarget = findTargetIndex(rig, BoneNames::Head);
    ASSERT_GE(headTarget, 0);
    for (size_t t = 0; t < rig.targets.size(); ++t)
    {
        if (static_cast<int>(t) == headTarget)
        {
            // Head is the HMD (anchor), never a tracker — skipped outright.
            EXPECT_FALSE(map.avatarJoint[t].has_value()) << "Head should be skipped";
            EXPECT_FALSE(map.enabled[t]) << "Head should be disabled";
            EXPECT_FALSE(map.rotationEnabled[t]);
            continue;
        }
        ASSERT_TRUE(map.avatarJoint[t].has_value()) << "target " << t;
        const std::string& name = rig.skeleton.joints[rig.targets[t].jointIndex].name;
        EXPECT_EQ(avatar.joints[*map.avatarJoint[t]].name, name);
        EXPECT_TRUE(map.enabled[t]) << "target " << t << " should start enabled";
        EXPECT_TRUE(map.rotationEnabled[t]) << "target " << t << " should start rotation-enabled";
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
        correctDevicePoses(calib.calibration, map, avatar, calib.devices);
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
        correctDevicePoses(calib.calibration, map, avatar, calib.devices);
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
        correctDevicePoses(calib.calibration, map, avatar, calib.devices);

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

    const std::vector<CorrectedPose> corrected = correctDevicePoses(calibration, map, avatar, devices);

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
    EXPECT_TRUE(correctDevicePoses(uncalibrated, map, avatar, {}).empty());
}

// ---- correctionOffsets: delta reproduces the corrected pose ----------------

TEST(CorrectionOffsets, DeltaComposedWithRawReproducesCorrected)
{
    const Pose raw{glm::vec3(1.0f, 2.0f, 3.0f),
                   glm::angleAxis(0.5f, glm::vec3(0.0f, 1.0f, 0.0f))};
    const Pose corrected{glm::vec3(1.1f, 2.2f, 3.3f),
                         glm::angleAxis(0.6f, glm::vec3(0.0f, 1.0f, 0.0f))};

    std::vector<CorrectedPose> poses{{0, 42, corrected}};
    std::vector<TrackedDevice> devices{{42, TrackedDeviceKind::Tracker, raw}};

    const auto offsets = correctionOffsets(poses, devices);
    ASSERT_EQ(offsets.size(), 1u);
    EXPECT_EQ(offsets[0].deviceId, 42);

    const Pose reconstructed = compose(offsets[0].delta, raw);
    expectPoseNear(reconstructed, corrected);
}

TEST(CorrectionOffsets, DeviceMissingFromSnapshotIsSkipped)
{
    const Pose raw{glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    const Pose corrected{glm::vec3(0.1f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};

    std::vector<CorrectedPose> poses{{0, 99, corrected}};
    std::vector<TrackedDevice> devices{{42, TrackedDeviceKind::Tracker, raw}};

    EXPECT_TRUE(correctionOffsets(poses, devices).empty());
}

TEST(CorrectionOffsets, EmptyCorrectedYieldsEmptyOffsets)
{
    std::vector<CorrectedPose> poses;
    std::vector<TrackedDevice> devices{{1, TrackedDeviceKind::Tracker, {}}};
    EXPECT_TRUE(correctionOffsets(poses, devices).empty());
}

// ---- controllers: rotation is locked, only position is corrected ------------

TEST(CorrectDevicePoses, ControllerKeepsRawRotationTrackerTakesAvatarRotation)
{
    IkRig rig = makeRig();
    const RestCalibration calib = calibrateAtRest(rig);
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    retargetPose(rig.skeleton, avatar, buildRetargetMap(rig.skeleton, avatar));
    const CorrectionMap map = buildCorrectionMap(rig, avatar);

    // Give the raw devices a non-identity rotation so the lock is observable.
    const glm::quat controllerRot = glm::angleAxis(0.7f, glm::vec3(0.0f, 1.0f, 0.0f));
    std::vector<TrackedDevice> devices = calib.devices;
    for (TrackedDevice& d : devices)
        if (d.kind == TrackedDeviceKind::Controller)
            d.pose.rotation = controllerRot;

    const std::vector<CorrectedPose> corrected =
        correctDevicePoses(calib.calibration, map, avatar, devices);
    const WorldTransforms avatarWorld = computeWorldTransforms(avatar);

    const int leftHandTarget = findTargetIndex(rig, BoneNames::LeftHand);
    const int leftFootTarget = findTargetIndex(rig, BoneNames::LeftFoot);
    ASSERT_GE(leftHandTarget, 0);
    ASSERT_GE(leftFootTarget, 0);

    for (const CorrectedPose& c : corrected)
    {
        const int avatarJoint = *map.avatarJoint[c.targetIndex];
        if (static_cast<int>(c.targetIndex) == leftHandTarget)
        {
            // Controller: position corrected to the avatar joint, rotation kept
            // exactly as the raw device reported it.
            EXPECT_TRUE(c.rotationLocked);
            EXPECT_NEAR(glm::dot(c.pose.rotation, controllerRot), 1.0f, 1e-6f);
            EXPECT_TRUE(glm::all(glm::epsilonEqual(
                c.pose.position, avatarWorld.positions[static_cast<size_t>(avatarJoint)], 1e-4f)));
        }
        else if (static_cast<int>(c.targetIndex) == leftFootTarget)
        {
            // Tracker: full avatar joint world pose (position + rotation).
            EXPECT_FALSE(c.rotationLocked);
            const Pose expected{avatarWorld.positions[static_cast<size_t>(avatarJoint)],
                                avatarWorld.rotations[static_cast<size_t>(avatarJoint)]};
            expectPoseNear(c.pose, expected, 1e-4f);
        }
    }
}

TEST(CorrectDevicePoses, BoundDeviceMissingFromSnapshotIsSkipped)
{
    IkRig rig = makeRig();
    const RestCalibration calib = calibrateAtRest(rig);
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    retargetPose(rig.skeleton, avatar, buildRetargetMap(rig.skeleton, avatar));
    const CorrectionMap map = buildCorrectionMap(rig, avatar);

    // Drop one device from the snapshot entirely.
    std::vector<TrackedDevice> partial = calib.devices;
    const int leftFootTarget = findTargetIndex(rig, BoneNames::LeftFoot);
    ASSERT_GE(leftFootTarget, 0);
    const int droppedId = calib.devices[static_cast<size_t>(leftFootTarget)].id;
    partial.erase(std::remove_if(partial.begin(), partial.end(),
                                 [droppedId](const TrackedDevice& d) { return d.id == droppedId; }),
                  partial.end());

    const std::vector<CorrectedPose> corrected =
        correctDevicePoses(calib.calibration, map, avatar, partial);

    for (const CorrectedPose& c : corrected)
        EXPECT_NE(c.deviceId, droppedId);
}

TEST(CorrectDevicePoses, TrackerRotationDisabledKeepsRawRotation)
{
    IkRig rig = makeRig();
    const RestCalibration calib = calibrateAtRest(rig);
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    retargetPose(rig.skeleton, avatar, buildRetargetMap(rig.skeleton, avatar));
    CorrectionMap map = buildCorrectionMap(rig, avatar);

    // Give the foot tracker a non-identity rotation so the toggle is observable.
    const glm::quat trackerRot = glm::angleAxis(0.9f, glm::vec3(1.0f, 0.0f, 0.0f));
    std::vector<TrackedDevice> devices = calib.devices;
    const int leftFootTarget = findTargetIndex(rig, BoneNames::LeftFoot);
    ASSERT_GE(leftFootTarget, 0);
    for (TrackedDevice& d : devices)
        if (d.id == calib.devices[static_cast<size_t>(leftFootTarget)].id)
            d.pose.rotation = trackerRot;

    // Rotation disabled → tracker keeps raw rotation, position still corrected.
    map.rotationEnabled[static_cast<size_t>(leftFootTarget)] = false;

    const std::vector<CorrectedPose> corrected =
        correctDevicePoses(calib.calibration, map, avatar, devices);
    const WorldTransforms avatarWorld = computeWorldTransforms(avatar);

    bool found = false;
    for (const CorrectedPose& c : corrected)
    {
        if (static_cast<int>(c.targetIndex) != leftFootTarget)
            continue;
        found = true;
        EXPECT_TRUE(c.rotationLocked);
        EXPECT_NEAR(glm::dot(c.pose.rotation, trackerRot), 1.0f, 1e-6f);
        const int avatarJoint = *map.avatarJoint[c.targetIndex];
        EXPECT_TRUE(glm::all(glm::epsilonEqual(
            c.pose.position, avatarWorld.positions[static_cast<size_t>(avatarJoint)], 1e-4f)));
    }
    EXPECT_TRUE(found);
}

TEST(CorrectionOffsets, LockedControllerDeltaHasIdentityRotation)
{
    const glm::quat rawRot = glm::angleAxis(0.5f, glm::vec3(0.0f, 1.0f, 0.0f));
    const Pose raw{glm::vec3(1.0f, 2.0f, 3.0f), rawRot};
    const glm::vec3 correctedPos(1.5f, 2.5f, 3.5f);

    // rotationLocked = true: corrected pose carries rawRot but the delta must
    // come out with exactly identity rotation.
    std::vector<CorrectedPose> poses{{0, 42, Pose{correctedPos, rawRot}, true}};
    std::vector<TrackedDevice> devices{{42, TrackedDeviceKind::Controller, raw}};

    const auto offsets = correctionOffsets(poses, devices);
    ASSERT_EQ(offsets.size(), 1u);
    EXPECT_EQ(offsets[0].deviceId, 42);
    // Exactly identity — no float epsilon from compose(inverse(rawRot), rawRot).
    EXPECT_FLOAT_EQ(offsets[0].delta.rotation.w, 1.0f);
    EXPECT_FLOAT_EQ(offsets[0].delta.rotation.x, 0.0f);
    EXPECT_FLOAT_EQ(offsets[0].delta.rotation.y, 0.0f);
    EXPECT_FLOAT_EQ(offsets[0].delta.rotation.z, 0.0f);
    // Reconstructing: compose(delta, raw) keeps the raw rotation and lands on
    // the corrected position.
    const Pose reconstructed = compose(offsets[0].delta, raw);
    EXPECT_TRUE(glm::all(glm::epsilonEqual(reconstructed.position, correctedPos, 1e-5f)));
    EXPECT_NEAR(glm::dot(reconstructed.rotation, rawRot), 1.0f, 1e-5f);
}
