#include <gtest/gtest.h>

#include <glm/gtc/epsilon.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "model/BoneNames.h"
#include "model/IkRig.h"
#include "model/IkRigConfig.h"
#include "model/Pose.h"
#include "model/Retarget.h"
#include "model/Skeleton.h"
#include "model/TrackedDevice.h"
#include "model/TrackerCalibration.h"
#include "model/VirtualTrackers.h"

namespace
{
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

// Places a device on every target's rest bone pose and calibrates, so every
// target (Head included — it binds to the placed device) becomes bound.
TrackerCalibration calibrateAllBound(IkRig& rig)
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
    const CalibrationFrame frame = updateCalibrationFrame(rig, devices);
    TrackerCalibration calibration;
    captureOffsets(calibration, frame, devices);
    return calibration;
}

// Calibrates every target except the one whose bone name matches `skip`.
TrackerCalibration calibrateAllExcept(IkRig& rig, const char* skip)
{
    const WorldTransforms rest = computeWorldTransforms(rig.skeleton);
    std::vector<TrackedDevice> devices;
    for (size_t i = 0; i < rig.targets.size(); ++i)
    {
        const std::string& name = rig.skeleton.joints[rig.targets[i].jointIndex].name;
        if (name == skip)
            continue;
        devices.push_back({static_cast<int>(100 + i), TrackedDeviceKind::Tracker,
                           {rest.positions[rig.targets[i].jointIndex],
                            glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}});
    }
    const CalibrationFrame frame = updateCalibrationFrame(rig, devices);
    TrackerCalibration calibration;
    captureOffsets(calibration, frame, devices);
    return calibration;
}

bool contains(const std::vector<std::string>& names, std::string_view needle)
{
    return std::find(names.begin(), names.end(), needle) != names.end();
}
} // namespace

TEST(EligibleVirtualTrackers, AllSixBoundYieldsSixteenBones)
{
    IkRig rig = makeRig();
    const TrackerCalibration calib = calibrateAllBound(rig);
    const Skeleton avatar = Skeleton::makeDefaultHipRooted();

    const std::vector<std::string> bones = eligibleVirtualTrackerBones(rig, avatar, calib);

    const std::vector<std::string> expected = {
        BoneNames::Neck,         BoneNames::Chest,         BoneNames::Spine,
        BoneNames::Waist,        BoneNames::LeftHip,      BoneNames::RightHip,
        BoneNames::LeftUpperLeg, BoneNames::RightUpperLeg, BoneNames::LeftLowerLeg,
        BoneNames::RightLowerLeg, BoneNames::LeftShoulder, BoneNames::RightShoulder,
        BoneNames::LeftUpperArm,  BoneNames::RightUpperArm, BoneNames::LeftLowerArm,
        BoneNames::RightLowerArm,
    };
    EXPECT_EQ(bones.size(), expected.size());
    for (const std::string& name : expected)
        EXPECT_TRUE(contains(bones, name)) << "missing " << name;
    EXPECT_EQ(bones.size(), 16u);
}

TEST(EligibleVirtualTrackers, KindOneExcluded)
{
    IkRig rig = makeRig();
    const TrackerCalibration calib = calibrateAllBound(rig);
    const Skeleton avatar = Skeleton::makeDefaultHipRooted();

    const std::vector<std::string> bones = eligibleVirtualTrackerBones(rig, avatar, calib);

    EXPECT_FALSE(contains(bones, BoneNames::Head));
    EXPECT_FALSE(contains(bones, BoneNames::Hips));
    EXPECT_FALSE(contains(bones, BoneNames::LeftHand));
    EXPECT_FALSE(contains(bones, BoneNames::RightHand));
    EXPECT_FALSE(contains(bones, BoneNames::LeftFoot));
    EXPECT_FALSE(contains(bones, BoneNames::RightFoot));
}

TEST(EligibleVirtualTrackers, AvatarOnlyBoneAbsent)
{
    IkRig rig = makeRig();
    const TrackerCalibration calib = calibrateAllBound(rig);
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    avatar.joints.push_back({.name = "AvatarOnlyBone",
                             .parentIndex = std::nullopt,
                             .restOffset = glm::vec3(0.0f),
                             .localRot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)});

    const std::vector<std::string> bones = eligibleVirtualTrackerBones(rig, avatar, calib);

    EXPECT_FALSE(contains(bones, "AvatarOnlyBone"));
}

TEST(EligibleVirtualTrackers, SourceOnlyBoneAbsent)
{
    IkRig rig = makeRig();
    const TrackerCalibration calib = calibrateAllBound(rig);
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    avatar.joints[findJointIndex(avatar, BoneNames::Neck)].name = "NeckX";

    const std::vector<std::string> bones = eligibleVirtualTrackerBones(rig, avatar, calib);

    EXPECT_FALSE(contains(bones, BoneNames::Neck));
    EXPECT_FALSE(contains(bones, "NeckX"));
}

TEST(EligibleVirtualTrackers, OrderIsSourceParentBeforeChild)
{
    IkRig rig = makeRig();
    const TrackerCalibration calib = calibrateAllBound(rig);
    const Skeleton avatar = Skeleton::makeDefaultHipRooted();

    const std::vector<std::string> bones = eligibleVirtualTrackerBones(rig, avatar, calib);

    std::vector<std::string> sourceEligible;
    for (const Joint& joint : rig.skeleton.joints)
        if (joint.name != BoneNames::Head && joint.name != BoneNames::Hips &&
            joint.name != BoneNames::LeftHand && joint.name != BoneNames::RightHand &&
            joint.name != BoneNames::LeftFoot && joint.name != BoneNames::RightFoot)
            sourceEligible.push_back(joint.name);
    EXPECT_EQ(bones, sourceEligible);
}

// ---- step 2: computeVirtualTrackerPoses ------------------------------------

TEST(VirtualTrackerPoses, PositionIsMidpointOfAvatarBoneEndpoints)
{
    IkRig rig = makeRig();
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    retargetPose(rig.skeleton, avatar, buildRetargetMap(rig.skeleton, avatar));
    const WorldTransforms wt = computeWorldTransforms(avatar);

    const std::vector<std::string> bones = {
        BoneNames::LeftUpperLeg, BoneNames::RightUpperArm,
    };
    const std::vector<VirtualTrackerPose> poses = computeVirtualTrackerPoses(avatar, bones);

    ASSERT_EQ(poses.size(), 2u);
    for (const VirtualTrackerPose& p : poses)
    {
        const int joint = findJointIndex(avatar, p.name.c_str());
        ASSERT_GE(joint, 0);
        const int parent = *avatar.joints[static_cast<size_t>(joint)].parentIndex;
        const glm::vec3 expectedMid =
            (wt.positions[static_cast<size_t>(parent)] + wt.positions[static_cast<size_t>(joint)]) * 0.5f;
        EXPECT_TRUE(glm::all(glm::epsilonEqual(p.pose.position, expectedMid, 1e-6f)));
    }
}

TEST(VirtualTrackerPoses, ThighMidpointMatchesHandComputedValue)
{
    IkRig rig = makeRig();
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    retargetPose(rig.skeleton, avatar, buildRetargetMap(rig.skeleton, avatar));
    const WorldTransforms wt = computeWorldTransforms(avatar);
    const int hip = findJointIndex(avatar, BoneNames::LeftHip);
    const int thigh = findJointIndex(avatar, BoneNames::LeftUpperLeg);
    ASSERT_GE(hip, 0);
    ASSERT_GE(thigh, 0);
    const glm::vec3 handComputed =
        (wt.positions[static_cast<size_t>(hip)] + wt.positions[static_cast<size_t>(thigh)]) * 0.5f;

    const std::vector<VirtualTrackerPose> poses =
        computeVirtualTrackerPoses(avatar, {std::string(BoneNames::LeftUpperLeg)});
    ASSERT_EQ(poses.size(), 1u);
    EXPECT_TRUE(glm::all(glm::epsilonEqual(poses[0].pose.position, handComputed, 1e-6f)));
}

TEST(VirtualTrackerPoses, UpperArmMidpointMatchesHandComputedValue)
{
    IkRig rig = makeRig();
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    retargetPose(rig.skeleton, avatar, buildRetargetMap(rig.skeleton, avatar));
    const WorldTransforms wt = computeWorldTransforms(avatar);
    const int shoulder = findJointIndex(avatar, BoneNames::LeftShoulder);
    const int upperArm = findJointIndex(avatar, BoneNames::LeftUpperArm);
    ASSERT_GE(shoulder, 0);
    ASSERT_GE(upperArm, 0);
    const glm::vec3 handComputed =
        (wt.positions[static_cast<size_t>(shoulder)] + wt.positions[static_cast<size_t>(upperArm)]) * 0.5f;

    const std::vector<VirtualTrackerPose> poses =
        computeVirtualTrackerPoses(avatar, {std::string(BoneNames::LeftUpperArm)});
    ASSERT_EQ(poses.size(), 1u);
    EXPECT_TRUE(glm::all(glm::epsilonEqual(poses[0].pose.position, handComputed, 1e-6f)));
}

TEST(VirtualTrackerPoses, RotationEqualsAvatarJointWorldRotation)
{
    IkRig rig = makeRig();
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    retargetPose(rig.skeleton, avatar, buildRetargetMap(rig.skeleton, avatar));
    const WorldTransforms wt = computeWorldTransforms(avatar);

    const std::vector<std::string> bones = {
        BoneNames::Chest, BoneNames::LeftLowerLeg, BoneNames::RightLowerArm,
    };
    const std::vector<VirtualTrackerPose> poses = computeVirtualTrackerPoses(avatar, bones);

    ASSERT_EQ(poses.size(), bones.size());
    for (size_t i = 0; i < poses.size(); ++i)
    {
        const int joint = findJointIndex(avatar, bones[i].c_str());
        ASSERT_GE(joint, 0);
        EXPECT_NEAR(glm::dot(poses[i].pose.rotation, wt.rotations[static_cast<size_t>(joint)]), 1.0f, 1e-6f);
    }
}

TEST(VirtualTrackerPoses, PositionsChangeWhenAvatarPoseChanges)
{
    IkRig rig = makeRig();
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    retargetPose(rig.skeleton, avatar, buildRetargetMap(rig.skeleton, avatar));

    const std::vector<std::string> bones = {BoneNames::LeftUpperLeg, BoneNames::Chest};
    const std::vector<VirtualTrackerPose> rest = computeVirtualTrackerPoses(avatar, bones);

    const int hips = findJointIndex(avatar, BoneNames::Hips);
    ASSERT_GE(hips, 0);
    avatar.joints[static_cast<size_t>(hips)].localRot =
        glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.0f, 0.0f, 1.0f));

    const std::vector<VirtualTrackerPose> posed = computeVirtualTrackerPoses(avatar, bones);
    ASSERT_EQ(posed.size(), rest.size());
    bool anyChanged = false;
    for (size_t i = 0; i < posed.size(); ++i)
        if (!glm::all(glm::epsilonEqual(posed[i].pose.position, rest[i].pose.position, 1e-4f)))
            anyChanged = true;
    EXPECT_TRUE(anyChanged);
}

TEST(VirtualTrackerPoses, PositionsChangeWhenAvatarRescaledByMatchRestHeight)
{
    IkRig rig = makeRig();
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    for (Joint& joint : avatar.joints)
        if (joint.parentIndex)
            joint.restOffset *= 1.5f;

    const std::vector<std::string> bones = {BoneNames::LeftUpperLeg};
    const std::vector<VirtualTrackerPose> before = computeVirtualTrackerPoses(avatar, bones);

    matchRestHeight(rig.skeleton, avatar);

    const std::vector<VirtualTrackerPose> after = computeVirtualTrackerPoses(avatar, bones);
    ASSERT_EQ(after.size(), before.size());
    EXPECT_FALSE(glm::all(glm::epsilonEqual(after[0].pose.position, before[0].pose.position, 1e-4f)));
}

TEST(VirtualTrackerPoses, UnknownBoneIsSkipped)
{
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    const std::vector<VirtualTrackerPose> poses =
        computeVirtualTrackerPoses(avatar, {std::string("NoSuchBone")});
    EXPECT_TRUE(poses.empty());
}

TEST(VirtualTrackerPoses, OutputOrderMatchesInputOrder)
{
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    const std::vector<std::string> bones = {
        BoneNames::RightLowerArm, BoneNames::Neck, BoneNames::LeftHip,
    };
    const std::vector<VirtualTrackerPose> poses = computeVirtualTrackerPoses(avatar, bones);
    ASSERT_EQ(poses.size(), bones.size());
    for (size_t i = 0; i < poses.size(); ++i)
        EXPECT_EQ(poses[i].name, bones[i]);
}

TEST(VirtualTrackerPoses, EmptyInputYieldsEmptyOutput)
{
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    EXPECT_TRUE(computeVirtualTrackerPoses(avatar, {}).empty());
}

// ---- bug regression guard (added after step-4 user report) ---------------
//
// Bug 1: on a fresh start (no calibration), every IK target bone leaked into
// the eligible list because the step-4 rule excluded only targets with a
// bound device — pre-calibration every boundDevice() returns nullopt, so
// nothing is excluded. Head (the HMD anchor), Hips, hands and feet showed up
// as tickable virtual-tracker candidates, colliding with real trackers. This
// test asserts the defect: with an uncalibrated TrackerCalibration, no IK
// target bone is eligible.
//
// Bug 2 (no markers rendered in replay) is a GUI-wiring defect in main.cpp
// — the render loop never called computeVirtualTrackerPoses for the ticked
// selection. That cannot be caught by a model-layer unit test (the pipeline
// function, when called, works; the defect is that main.cpp forgot to call
// it). Bug 2 is verified by code inspection of main.cpp, not by a test.
TEST(EligibleVirtualTrackers, Bug1IkTargetBonesExcludedPreCalibration)
{
    IkRig rig = makeRig();
    const Skeleton avatar = Skeleton::makeDefaultHipRooted();
    const TrackerCalibration uncalibrated; // fresh start: nothing bound

    const std::vector<std::string> bones = eligibleVirtualTrackerBones(rig, avatar, uncalibrated);

    EXPECT_FALSE(contains(bones, BoneNames::Head))
        << "BUG 1: Head leaked into eligible list pre-calibration";
    EXPECT_FALSE(contains(bones, BoneNames::Hips))
        << "BUG 1: Hips leaked into eligible list pre-calibration";
    EXPECT_FALSE(contains(bones, BoneNames::LeftHand))
        << "BUG 1: LeftHand leaked into eligible list pre-calibration";
    EXPECT_FALSE(contains(bones, BoneNames::RightHand))
        << "BUG 1: RightHand leaked into eligible list pre-calibration";
    EXPECT_FALSE(contains(bones, BoneNames::LeftFoot))
        << "BUG 1: LeftFoot leaked into eligible list pre-calibration";
    EXPECT_FALSE(contains(bones, BoneNames::RightFoot))
        << "BUG 1: RightFoot leaked into eligible list pre-calibration";
}

// ---- bug 2: VirtualTrackerRenderer produces the marker poses ----------------
//
// The render loop draws what VirtualTrackerRenderer::poses() returns. If the
// class returns empty, no VT markers render — the app shows ticked bones that
// produce nothing visible (the reported defect: "don't see trackers anywhere
// in replay"). This test asserts the defect: a renderer constructed with a
// ticked eligible bone must return a non-empty pose vector. Against the stub
// (returns empty) the test fails — proving the visualization is missing.
TEST(VirtualTrackerRenderer, Bug2TickedEligibleBoneYieldsPoses)
{
    IkRig rig = makeRig();
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    retargetPose(rig.skeleton, avatar, buildRetargetMap(rig.skeleton, avatar));
    const TrackerCalibration uncalibrated;
    const std::vector<std::string> selected = {std::string(BoneNames::Neck)};

    const VirtualTrackerRenderer renderer(rig, avatar, uncalibrated, selected);
    const std::vector<Pose> poses = renderer.poses();

    ASSERT_FALSE(poses.empty())
        << "BUG 2: VirtualTrackerRenderer::poses() returned empty — "
           "visualization missing, ticked bones produce no markers";
    EXPECT_GT(glm::length(poses[0].position), 0.0f);
}

TEST(VirtualTrackerRenderer, EmptySelectionYieldsNoPoses)
{
    IkRig rig = makeRig();
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    const TrackerCalibration uncalibrated;
    const std::vector<std::string> empty;

    const VirtualTrackerRenderer renderer(rig, avatar, uncalibrated, empty);
    EXPECT_TRUE(renderer.poses().empty());
}
