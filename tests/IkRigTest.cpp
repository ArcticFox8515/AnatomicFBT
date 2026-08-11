#include <gtest/gtest.h>
#include "model/BoneNames.h"
#include "model/IkRig.h"
#include "model/IkRigConfig.h"
#include "model/Skeleton.h"

#include <cmath>

namespace
{
IkRig makeDefaultRig()
{
    IkRig rig(Skeleton::makeDefault());
    rig.loadConfig(IkRigConfig::makeDefault());
    return rig;
}
} // namespace

TEST(IkRigConfigSerialization, DefaultRoundTrip)
{
    const IkRigConfig original = IkRigConfig::makeDefault();
    const nlohmann::json j = original;
    const IkRigConfig parsed = j.get<IkRigConfig>();

    ASSERT_EQ(parsed.targets.size(), original.targets.size());
    for (size_t i = 0; i < original.targets.size(); ++i)
    {
        EXPECT_EQ(parsed.targets[i].bone, original.targets[i].bone);
        EXPECT_EQ(parsed.targets[i].solver, original.targets[i].solver);
    }
    ASSERT_EQ(parsed.limits.size(), original.limits.size());
    for (size_t i = 0; i < original.limits.size(); ++i)
    {
        EXPECT_EQ(parsed.limits[i].bone, original.limits[i].bone);
        EXPECT_FLOAT_EQ(parsed.limits[i].twistMinDeg, original.limits[i].twistMinDeg);
        EXPECT_FLOAT_EQ(parsed.limits[i].twistMaxDeg, original.limits[i].twistMaxDeg);
        EXPECT_FLOAT_EQ(parsed.limits[i].swingConeDeg, original.limits[i].swingConeDeg);
        ASSERT_EQ(parsed.limits[i].pole.has_value(), original.limits[i].pole.has_value());
        if (original.limits[i].pole)
            EXPECT_EQ(*parsed.limits[i].pole, *original.limits[i].pole);
    }
}

TEST(IkRigConfigDeserialization, DuplicateTargetsThrow)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "targets": [
            { "bone": "Head", "solver": "anchor" },
            { "bone": "Head", "solver": "chain" }
        ]
    })");
    EXPECT_THROW(j.get<IkRigConfig>(), std::runtime_error);
}

TEST(IkRigConfigDeserialization, LegacyStringTargetsThrow)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "targets": ["Head", "Hips"]
    })");
    EXPECT_THROW(j.get<IkRigConfig>(), nlohmann::json::exception);
}

TEST(IkRigConfigDeserialization, InvalidPoleThrows)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "limits": [ { "bone": "x", "pole": [0.0, 0.0] } ]
    })");
    EXPECT_THROW(j.get<IkRigConfig>(), nlohmann::json::exception);
}

TEST(IkRigConfigDeserialization, InvalidLimitThrows)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "limits": [ { "bone": "x", "twistMin": 10.0, "twistMax": -10.0 } ]
    })");
    EXPECT_THROW(j.get<IkRigConfig>(), std::runtime_error);
}

TEST(IkRigConfigDeserialization, MissingSectionsGetDefaults)
{
    const IkRigConfig config = nlohmann::json::parse("{}").get<IkRigConfig>();
    EXPECT_TRUE(config.targets.empty());
    EXPECT_TRUE(config.limits.empty());
}

TEST(IkRigConfigDeserialization, TargetsMustBeArray)
{
    const nlohmann::json j = nlohmann::json::parse(R"({ "targets": "Head" })");
    EXPECT_THROW(j.get<IkRigConfig>(), nlohmann::json::exception);
}

TEST(IkRigConfigDeserialization, TargetMissingBoneThrows)
{
    const nlohmann::json j = nlohmann::json::parse(R"({ "targets": [ { "solver": "anchor" } ] })");
    EXPECT_THROW(j.get<IkRigConfig>(), nlohmann::json::exception);
}

TEST(IkRigConstruction, TargetsBindToJointsAndInitAtRestPose)
{
    IkRig rig = makeDefaultRig();

    ASSERT_EQ(rig.targets.size(), rig.config.targets.size());
    const std::vector<glm::vec3> restPositions = computeWorldPositions(rig.skeleton);
    for (size_t i = 0; i < rig.targets.size(); ++i)
    {
        const IkTarget& target = rig.targets[i];
        EXPECT_EQ(rig.skeleton.joints[target.jointIndex].name, rig.config.targets[i].bone);
        EXPECT_EQ(target.position, restPositions[target.jointIndex]);
        EXPECT_EQ(target.rotation, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    }
}

TEST(IkRigLoadConfig, UnknownTargetBoneThrows)
{
    IkRigConfig config;
    config.targets = {{"does_not_exist", SolverType::Anchor}};
    IkRig rig(Skeleton::makeDefault());
    EXPECT_THROW(rig.loadConfig(config), std::runtime_error);
}

TEST(IkRigLoadConfig, UnknownLimitBoneThrows)
{
    IkRigConfig config;
    config.limits.push_back(JointLimits{"does_not_exist", 0.0f, 0.0f, 90.0f});
    IkRig rig(Skeleton::makeDefault());
    EXPECT_THROW(rig.loadConfig(config), std::runtime_error);
}

TEST(IkRigLoadConfig, AnchorOnNonRootThrows)
{
    IkRigConfig config;
    config.targets = {{"Hips", SolverType::Anchor}};
    IkRig rig(Skeleton::makeDefault());
    EXPECT_THROW(rig.loadConfig(config), std::runtime_error);
}

TEST(IkRigLoadConfig, ChainOnRootThrows)
{
    IkRigConfig config;
    config.targets = {{"Head", SolverType::Chain}};
    IkRig rig(Skeleton::makeDefault());
    EXPECT_THROW(rig.loadConfig(config), std::runtime_error);
}

TEST(IkRigLoadConfig, TwoBoneWithTooFewAncestorsThrows)
{
    IkRigConfig config;
    config.targets = {{"Chest", SolverType::TwoBone}};  // Head->Neck->Chest: only 2
    IkRig rig(Skeleton::makeDefault());
    EXPECT_THROW(rig.loadConfig(config), std::runtime_error);
}

TEST(IkRigLoadConfig, TwoBoneWithoutMiddleBonePoleThrows)
{
    IkRigConfig config;
    config.targets = {{"LeftFoot", SolverType::TwoBone}};  // no limits entry -> no pole
    IkRig rig(Skeleton::makeDefault());
    EXPECT_THROW(rig.loadConfig(config), std::runtime_error);
}

TEST(IkRigLoadConfig, FailedLoadKeepsPreviousConfig)
{
    IkRig rig = makeDefaultRig();
    const size_t targetCount = rig.targets.size();

    IkRigConfig bad;
    bad.targets = {{"does_not_exist", SolverType::Anchor}};
    EXPECT_THROW(rig.loadConfig(bad), std::runtime_error);

    EXPECT_EQ(rig.targets.size(), targetCount);
    EXPECT_EQ(rig.config.targets.size(), IkRigConfig::makeDefault().targets.size());
}

TEST(IkRig, ResetTargetsRestoresRestPose)
{
    IkRig rig = makeDefaultRig();
    rig.targets[0].position = glm::vec3(5.0f, 5.0f, 5.0f);
    rig.targets[0].rotation = glm::quat(0.0f, 1.0f, 0.0f, 0.0f);

    rig.resetTargets();

    const std::vector<glm::vec3> restPositions = computeWorldPositions(rig.skeleton);
    EXPECT_EQ(rig.targets[0].position, restPositions[rig.targets[0].jointIndex]);
    EXPECT_EQ(rig.targets[0].rotation, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
}

namespace
{
IkTarget* findTarget(IkRig& rig, const std::string& bone)
{
    for (IkTarget& target : rig.targets)
        if (rig.skeleton.joints[target.jointIndex].name == bone)
            return &target;
    return nullptr;
}

int findJoint(const IkRig& rig, const std::string& bone)
{
    for (size_t i = 0; i < rig.skeleton.joints.size(); ++i)
        if (rig.skeleton.joints[i].name == bone)
            return static_cast<int>(i);
    return -1;
}

void expectVecNear(const glm::vec3& actual, const glm::vec3& expected, float tol)
{
    EXPECT_NEAR(actual.x, expected.x, tol);
    EXPECT_NEAR(actual.y, expected.y, tol);
    EXPECT_NEAR(actual.z, expected.z, tol);
}

// Angular distance between two rotations, in radians, in [0, pi] (shortest path).
float quatAngleRad(const glm::quat& a, const glm::quat& b)
{
    const glm::quat na = glm::normalize(a);
    const glm::quat nb = glm::normalize(b);
    const float d = glm::clamp(glm::abs(glm::dot(na, nb)), 0.0f, 1.0f);
    return glm::acos(d);
}

// Solves the default rig (already configured) toward the LeftFoot goal, and
// returns the world rotations of the mid bone (thigh) and tracked end bone.
struct FootSolve
{
    glm::quat thighWorld;
    glm::quat footWorld;
};

FootSolve solveLeftFoot(const IkRigConfig& config, const glm::vec3& position, const glm::quat& rotation)
{
    IkRig rig(Skeleton::makeDefault());
    rig.loadConfig(config);
    for (IkTarget& target : rig.targets)
        if (rig.skeleton.joints[target.jointIndex].name == BoneNames::LeftFoot)
        {
            target.position = position;
            target.rotation = rotation;
        }
    rig.solve();
    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    FootSolve result{glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    for (size_t i = 0; i < rig.skeleton.joints.size(); ++i)
    {
        if (rig.skeleton.joints[i].name == BoneNames::LeftUpperLeg)
            result.thighWorld = wt.rotations[i];
        else if (rig.skeleton.joints[i].name == BoneNames::LeftFoot)
            result.footWorld = wt.rotations[i];
    }
    return result;
}

// Same for the LeftHand target: mid bone = upper arm, end bone = hand.
struct ArmSolve
{
    glm::quat upperArmWorld;
    glm::quat handWorld;
};

ArmSolve solveLeftHand(const IkRigConfig& config, const glm::vec3& position, const glm::quat& rotation)
{
    IkRig rig(Skeleton::makeDefault());
    rig.loadConfig(config);
    for (IkTarget& target : rig.targets)
        if (rig.skeleton.joints[target.jointIndex].name == BoneNames::LeftHand)
        {
            target.position = position;
            target.rotation = rotation;
        }
    rig.solve();
    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    ArmSolve result{glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    for (size_t i = 0; i < rig.skeleton.joints.size(); ++i)
    {
        if (rig.skeleton.joints[i].name == BoneNames::LeftUpperArm)
            result.upperArmWorld = wt.rotations[i];
        else if (rig.skeleton.joints[i].name == BoneNames::LeftHand)
            result.handWorld = wt.rotations[i];
    }
    return result;
}

// Poses the Hips (chain end) target; returns the world rotations of the mid
// spine bone and the chain end.
struct SpineSolve
{
    glm::quat spineWorld;
    glm::quat hipsWorld;
};

SpineSolve solveHips(const IkRigConfig& config, const glm::vec3& position, const glm::quat& rotation)
{
    IkRig rig(Skeleton::makeDefault());
    rig.loadConfig(config);
    for (IkTarget& target : rig.targets)
        if (rig.skeleton.joints[target.jointIndex].name == BoneNames::Hips)
        {
            target.position = position;
            target.rotation = rotation;
        }
    rig.solve();
    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    SpineSolve result{glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    for (size_t i = 0; i < rig.skeleton.joints.size(); ++i)
    {
        if (rig.skeleton.joints[i].name == BoneNames::Spine)
            result.spineWorld = wt.rotations[i];
        else if (rig.skeleton.joints[i].name == BoneNames::Hips)
            result.hipsWorld = wt.rotations[i];
    }
    return result;
}
} // namespace

TEST(IkRigSolve, RestTargetsReproduceRestPose)
{
    IkRig rig = makeDefaultRig();
    const std::vector<glm::vec3> rest = computeWorldPositions(rig.skeleton);

    rig.solve();

    const std::vector<glm::vec3> solved = computeWorldPositions(rig.skeleton);
    ASSERT_EQ(solved.size(), rest.size());
    for (size_t i = 0; i < rest.size(); ++i)
        expectVecNear(solved[i], rest[i], 1e-3f);
}

TEST(IkRigSolve, MovingAllTargetsTranslatesWholeSkeleton)
{
    IkRig rig = makeDefaultRig();
    const std::vector<glm::vec3> rest = computeWorldPositions(rig.skeleton);
    const glm::vec3 delta(0.3f, -0.2f, 0.1f);
    for (IkTarget& target : rig.targets)
        target.position += delta;

    rig.solve();

    const std::vector<glm::vec3> solved = computeWorldPositions(rig.skeleton);
    ASSERT_EQ(solved.size(), rest.size());
    for (size_t i = 0; i < rest.size(); ++i)
        expectVecNear(solved[i], rest[i] + delta, 1e-3f);
}

TEST(IkRigSolve, HeadTargetDrivesRootPose)
{
    IkRig rig = makeDefaultRig();
    IkTarget* head = findTarget(rig, "Head");
    ASSERT_NE(head, nullptr);
    head->position = glm::vec3(0.5f, 1.2f, -0.3f);
    head->rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    rig.solve();

    int root = -1;
    for (size_t i = 0; i < rig.skeleton.joints.size(); ++i)
        if (!rig.skeleton.joints[i].parentIndex)
            root = static_cast<int>(i);
    ASSERT_GE(root, 0);
    EXPECT_EQ(rig.skeleton.rootPosition, head->position);
    EXPECT_NEAR(glm::abs(glm::dot(rig.skeleton.joints[root].localRot, head->rotation)), 1.0f, 1e-4f);
}

TEST(IkRigSolve, SolveFromExplicitGoalsLeavesTargetsUntouched)
{
    IkRig rig = makeDefaultRig();
    IkTarget* head = findTarget(rig, "Head");
    ASSERT_NE(head, nullptr);
    const size_t headIndex = static_cast<size_t>(head - rig.targets.data());
    const glm::vec3 targetBefore = rig.targets[headIndex].position;

    std::vector<IkTarget> goals = rig.targets;
    goals[headIndex].position = glm::vec3(0.5f, 1.2f, -0.3f);

    rig.solve(goals);

    // The goal drove the solve...
    EXPECT_EQ(rig.skeleton.rootPosition, glm::vec3(0.5f, 1.2f, -0.3f));
    // ...while the stored target kept its pose.
    EXPECT_EQ(rig.targets[headIndex].position, targetBefore);
}

TEST(IkRigSolve, SolveFromGoalsWithWrongSizeThrows)
{
    IkRig rig = makeDefaultRig();

    EXPECT_THROW(rig.solve({}), std::runtime_error);
    EXPECT_THROW(rig.solve(std::vector<IkTarget>(rig.targets.size() + 1)), std::runtime_error);
}

TEST(IkRigSolve, FootTargetPlacesAnkleWithKneeForward)
{
    IkRig rig = makeDefaultRig();
    IkTarget* foot = findTarget(rig, "LeftFoot");
    ASSERT_NE(foot, nullptr);
    foot->position = glm::vec3(0.1f, 0.5f, -0.4f);

    rig.solve();

    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    const int ankle = findJoint(rig, "LeftLowerLeg");
    const int knee = findJoint(rig, "LeftUpperLeg");
    ASSERT_GE(ankle, 0);
    ASSERT_GE(knee, 0);

    // The ankle lands on the goal implied by the foot target.
    const glm::vec3 goal = foot->position - foot->rotation * rig.skeleton.joints[findJoint(rig, "LeftFoot")].restOffset;
    expectVecNear(wt.positions[ankle], goal, 1e-3f);
    // The knee bends forward (-Z), not backward.
    EXPECT_LT(wt.positions[knee].z, -0.1f);
}

TEST(IkRigSolve, HandTargetPlacesWrist)
{
    IkRig rig = makeDefaultRig();
    IkTarget* hand = findTarget(rig, "RightHand");
    ASSERT_NE(hand, nullptr);
    hand->position = glm::vec3(-0.5f, 1.3f, -0.3f);

    rig.solve();

    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    const int wrist = findJoint(rig, "RightLowerArm");
    ASSERT_GE(wrist, 0);
    const glm::vec3 goal = hand->position - hand->rotation * rig.skeleton.joints[findJoint(rig, "RightHand")].restOffset;
    expectVecNear(wt.positions[wrist], goal, 1e-3f);
}

TEST(IkRigSolve, HipTargetCrouchesSpine)
{
    IkRig rig = makeDefaultRig();
    IkTarget* hip = findTarget(rig, "Hips");
    ASSERT_NE(hip, nullptr);
    hip->position = glm::vec3(0.0f, 1.3f, 0.0f);  // 0.4m below the head: spine must curl

    rig.solve();

    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    const int hipJoint = findJoint(rig, "Hips");
    const int chest = findJoint(rig, "Spine");
    ASSERT_GE(hipJoint, 0);
    ASSERT_GE(chest, 0);
    expectVecNear(wt.positions[hipJoint], hip->position, 0.01f);
    // The chest stays on the way between head and hip.
    EXPECT_GT(wt.positions[chest].y, 1.3f);
    EXPECT_LT(wt.positions[chest].y, 1.7f);
}

TEST(IkRigSolve, OverreachedFootStretchesLegWithoutExploding)
{
    IkRig rig = makeDefaultRig();
    IkTarget* foot = findTarget(rig, "LeftFoot");
    ASSERT_NE(foot, nullptr);
    foot->position = glm::vec3(0.1f, 0.02f, -3.0f);  // far beyond leg length

    rig.solve();

    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    const int socket = findJoint(rig, "LeftHip");
    const int ankle = findJoint(rig, "LeftLowerLeg");
    // Leg fully extended: ankle at chain length from the socket, toward the goal.
    EXPECT_NEAR(glm::length(wt.positions[ankle] - wt.positions[socket]), 0.9f, 1e-3f);
    const glm::vec3 goal = foot->position - foot->rotation * rig.skeleton.joints[findJoint(rig, "LeftFoot")].restOffset;
    const glm::vec3 aim = glm::normalize(goal - wt.positions[socket]);
    const glm::vec3 ankleDir = glm::normalize(wt.positions[ankle] - wt.positions[socket]);
    EXPECT_NEAR(glm::dot(ankleDir, aim), 1.0f, 1e-3f);
}

TEST(IkRigSolve, HeadPitchKeepsHipAndLegsStable)
{
    // Regression: pitching the head anchor must not corrupt the hip's world
    // orientation (previously the chain solver over-rotated the chain end),
    // which otherwise sends the legs haywire.
    IkRig rig = makeDefaultRig();
    const std::vector<glm::vec3> rest = computeWorldPositions(rig.skeleton);
    IkTarget* head = findTarget(rig, "Head");
    ASSERT_NE(head, nullptr);
    head->rotation = glm::angleAxis(glm::radians(75.0f), glm::vec3(1.0f, 0.0f, 0.0f));

    rig.solve();

    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    const int hip = findJoint(rig, "Hips");
    ASSERT_GE(hip, 0);
    EXPECT_NEAR(glm::abs(glm::dot(wt.rotations[hip], glm::quat(1.0f, 0.0f, 0.0f, 0.0f))), 1.0f, 0.05f);

    IkTarget* leftFoot = findTarget(rig, "LeftFoot");
    IkTarget* rightFoot = findTarget(rig, "RightFoot");
    ASSERT_NE(leftFoot, nullptr);
    ASSERT_NE(rightFoot, nullptr);
    const int leftFootJoint = findJoint(rig, "LeftFoot");
    const int rightFootJoint = findJoint(rig, "RightFoot");
    expectVecNear(wt.positions[leftFootJoint], leftFoot->position, 0.02f);
    expectVecNear(wt.positions[rightFootJoint], rightFoot->position, 0.02f);

    // With all leg targets at rest, the legs stay at the rest pose.
    const int leftKnee = findJoint(rig, "LeftUpperLeg");
    expectVecNear(wt.positions[leftKnee], rest[leftKnee], 0.02f);
}

TEST(IkRigSolve, HeadYawKeepsHipLevel)
{
    IkRig rig = makeDefaultRig();
    IkTarget* head = findTarget(rig, "Head");
    ASSERT_NE(head, nullptr);
    head->rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    rig.solve();

    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    const int hip = findJoint(rig, "Hips");
    ASSERT_GE(hip, 0);
    EXPECT_NEAR(glm::abs(glm::dot(wt.rotations[hip], glm::quat(1.0f, 0.0f, 0.0f, 0.0f))), 1.0f, 0.05f);

    IkTarget* leftFoot = findTarget(rig, "LeftFoot");
    IkTarget* rightFoot = findTarget(rig, "RightFoot");
    ASSERT_NE(leftFoot, nullptr);
    ASSERT_NE(rightFoot, nullptr);
    const int leftFootJoint = findJoint(rig, "LeftFoot");
    const int rightFootJoint = findJoint(rig, "RightFoot");
    expectVecNear(wt.positions[leftFootJoint], leftFoot->position, 0.02f);
    expectVecNear(wt.positions[rightFootJoint], rightFoot->position, 0.02f);
}

TEST(IkRigSolve, HipRollKeepsFeetOnTargetsAndLegsDown)
{
    // Regression: rolling the hip target about Z (sway) previously displaced
    // the hip's solved position and rotated the leg sockets so far that the
    // legs pointed in the wrong direction entirely.
    // Head and hip drop slightly: at full rest extension a rolled hip is
    // unreachable for both the spine and the raised-side leg.
    IkRig rig = makeDefaultRig();
    IkTarget* head = findTarget(rig, "Head");
    IkTarget* hip = findTarget(rig, "Hips");
    ASSERT_NE(head, nullptr);
    ASSERT_NE(hip, nullptr);
    head->position.y -= 0.1f;
    hip->position.y -= 0.05f;
    hip->rotation = glm::angleAxis(glm::radians(30.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    rig.solve();

    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    const int hipJoint = findJoint(rig, "Hips");
    ASSERT_GE(hipJoint, 0);
    expectVecNear(wt.positions[hipJoint], hip->position, 0.01f);

    IkTarget* leftFoot = findTarget(rig, "LeftFoot");
    IkTarget* rightFoot = findTarget(rig, "RightFoot");
    ASSERT_NE(leftFoot, nullptr);
    ASSERT_NE(rightFoot, nullptr);
    const int leftFootJoint = findJoint(rig, "LeftFoot");
    const int rightFootJoint = findJoint(rig, "RightFoot");
    expectVecNear(wt.positions[leftFootJoint], leftFoot->position, 0.02f);
    expectVecNear(wt.positions[rightFootJoint], rightFoot->position, 0.02f);

    // Upper legs still point roughly downward from the hip (not sideways).
    const int leftUpperLeg = findJoint(rig, "LeftUpperLeg");
    const int rightUpperLeg = findJoint(rig, "RightUpperLeg");
    const int leftHipSocket = findJoint(rig, "LeftHip");
    const int rightHipSocket = findJoint(rig, "RightHip");
    const glm::vec3 leftDir = glm::normalize(wt.positions[leftUpperLeg] - wt.positions[leftHipSocket]);
    const glm::vec3 rightDir = glm::normalize(wt.positions[rightUpperLeg] - wt.positions[rightHipSocket]);
    EXPECT_LT(leftDir.y, -0.7f);
    EXPECT_LT(rightDir.y, -0.7f);
}

TEST(IkRigSolve, HipSwayKeepsFeetPlanted)
{
    IkRig rig = makeDefaultRig();
    IkTarget* head = findTarget(rig, "Head");
    IkTarget* hip = findTarget(rig, "Hips");
    ASSERT_NE(head, nullptr);
    ASSERT_NE(hip, nullptr);
    head->position.y -= 0.1f;  // slack: the rest pose is fully extended
    hip->position += glm::vec3(0.1f, -0.05f, 0.0f);
    hip->rotation = glm::angleAxis(glm::radians(20.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    rig.solve();

    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    const int hipJoint = findJoint(rig, "Hips");
    ASSERT_GE(hipJoint, 0);
    expectVecNear(wt.positions[hipJoint], hip->position, 0.01f);

    IkTarget* leftFoot = findTarget(rig, "LeftFoot");
    IkTarget* rightFoot = findTarget(rig, "RightFoot");
    ASSERT_NE(leftFoot, nullptr);
    ASSERT_NE(rightFoot, nullptr);
    const int leftFootJoint = findJoint(rig, "LeftFoot");
    const int rightFootJoint = findJoint(rig, "RightFoot");
    expectVecNear(wt.positions[leftFootJoint], leftFoot->position, 0.02f);
    expectVecNear(wt.positions[rightFootJoint], rightFoot->position, 0.02f);
}

TEST(IkRigSolve, ClampedHipKeepsFootRotationExact)
{
    // WP1: a mid-bone limit must bend the pose but never silently rotate a
    // tracked end bone (issue 6: feet rotate in/out). A near-stiff hip cone
    // makes the thigh (a mid bone of the foot's two-bone chain) clamp hard;
    // the foot's tracked world rotation must still win after solve.
    const glm::quat goalRot = glm::angleAxis(glm::radians(30.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::vec3 footPos(0.1f, 0.5f, -0.3f);  // ankle pulled up + forward of the hip

    IkRigConfig loose = IkRigConfig::makeDefault();
    IkRigConfig tight = loose;
    for (JointLimits& limit : tight.limits)
        if (limit.bone == BoneNames::LeftUpperLeg)
            limit.swingConeDeg = 1.0f;

    const FootSolve looseSolve = solveLeftFoot(loose, footPos, goalRot);
    const FootSolve tightSolve = solveLeftFoot(tight, footPos, goalRot);

    // The mid-bone clamp visibly engaged...
    EXPECT_GT(quatAngleRad(tightSolve.thighWorld, looseSolve.thighWorld), glm::radians(5.0f));
    // ...yet the tracked end-bone rotation wins in both cases.
    EXPECT_NEAR(quatAngleRad(looseSolve.footWorld, goalRot), 0.0f, 1e-2f);
    EXPECT_NEAR(quatAngleRad(tightSolve.footWorld, goalRot), 0.0f, 1e-2f);
}

TEST(IkRigSolve, ClampedShoulderKeepsHandRotationExact)
{
    const glm::quat goalRot = glm::angleAxis(glm::radians(-40.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    const glm::vec3 handPos(0.55f, 1.85f, -0.2f);  // hand raised well off the shoulder line

    IkRigConfig loose = IkRigConfig::makeDefault();
    IkRigConfig tight = loose;
    for (JointLimits& limit : tight.limits)
        if (limit.bone == BoneNames::LeftUpperArm)
            limit.swingConeDeg = 5.0f;

    const ArmSolve looseSolve = solveLeftHand(loose, handPos, goalRot);
    const ArmSolve tightSolve = solveLeftHand(tight, handPos, goalRot);

    EXPECT_GT(quatAngleRad(tightSolve.upperArmWorld, looseSolve.upperArmWorld), glm::radians(5.0f));
    EXPECT_NEAR(quatAngleRad(looseSolve.handWorld, goalRot), 0.0f, 1e-2f);
    EXPECT_NEAR(quatAngleRad(tightSolve.handWorld, goalRot), 0.0f, 1e-2f);
}

TEST(IkRigSolve, ClampedSpineKeepsHipsRotationExact)
{
    // Chain end: a mid-spine limit must bend the chain but never rotate the
    // chain end (Hips) away from its tracked rotation. The Hips goal is a
    // crouch — the chain must arc, which swings the spine segments for real (a
    // pure body roll would be carried as axial twist inside the twist
    // envelope, which a swing cone cannot clamp).
    const glm::quat goalRot = glm::angleAxis(glm::radians(45.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    const glm::vec3 goalPos(0.0f, 1.3f, 0.0f);

    IkRigConfig loose = IkRigConfig::makeDefault();
    IkRigConfig tight = loose;
    tight.limits.push_back(JointLimits{BoneNames::Spine, -90.0f, 90.0f, 5.0f});

    const SpineSolve looseSolve = solveHips(loose, goalPos, goalRot);
    const SpineSolve tightSolve = solveHips(tight, goalPos, goalRot);

    EXPECT_GT(quatAngleRad(tightSolve.spineWorld, looseSolve.spineWorld), glm::radians(3.0f));
    EXPECT_NEAR(quatAngleRad(looseSolve.hipsWorld, goalRot), 0.0f, 1e-2f);
    EXPECT_NEAR(quatAngleRad(tightSolve.hipsWorld, goalRot), 0.0f, 1e-2f);
}

TEST(IkRigSolve, TrackedAnchorRotationWinsOverRootLimit)
{
    // Anchor: even a fully stiff limit on the root (Head) must not defeat the
    // pinned tracked rotation (policy: tracked rotation wins on end bones).
    IkRigConfig config = IkRigConfig::makeDefault();
    config.limits.push_back(JointLimits{BoneNames::Head, 0.0f, 0.0f, 0.0f});
    IkRig rig(Skeleton::makeDefault());
    rig.loadConfig(config);

    IkTarget* head = findTarget(rig, "Head");
    ASSERT_NE(head, nullptr);
    head->rotation = glm::angleAxis(glm::radians(45.0f), glm::vec3(1.0f, 0.0f, 0.0f));

    rig.solve();

    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    const int headJoint = findJoint(rig, "Head");
    ASSERT_GE(headJoint, 0);
    EXPECT_NEAR(quatAngleRad(wt.rotations[headJoint], head->rotation), 0.0f, 1e-2f);
}

TEST(IkRigSolve, ClampedSpineKeepsFootRotationExactUnderHipsGoal)
{
    // WP1, ancestor case: the re-aim pass must also hold when the clamped mid
    // bone sits ABOVE a *different* target's end bone. Hips (chain end) is an
    // ancestor of both legs, so re-aiming Hips rotates the whole leg rigidly.
    // A single FK snapshot taken before the pass means the feet, re-aimed
    // earlier in config order, were computed against a parent frame that the
    // Hips write then invalidates — the feet end up rotated away from their
    // tracked rotation, which is exactly issue 6.
    // Same Hips goal as ClampedSpineKeepsHipsRotationExact — a pose where the
    // spine segments swing for real, so a 5 deg cone on Spine clamps.
    const glm::quat hipsRot = glm::angleAxis(glm::radians(45.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    const glm::vec3 hipsPos(0.0f, 1.3f, 0.0f);
    const glm::quat footRot = glm::angleAxis(glm::radians(20.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    IkRigConfig config = IkRigConfig::makeDefault();
    config.limits.push_back(JointLimits{BoneNames::Spine, -90.0f, 90.0f, 5.0f});

    IkRig rig(Skeleton::makeDefault());
    rig.loadConfig(config);
    IkTarget* hips = findTarget(rig, "Hips");
    IkTarget* leftFoot = findTarget(rig, "LeftFoot");
    ASSERT_NE(hips, nullptr);
    ASSERT_NE(leftFoot, nullptr);
    hips->position = hipsPos;
    hips->rotation = hipsRot;
    leftFoot->rotation = footRot;  // feet stay at their rest positions
    const glm::vec3 footPos = leftFoot->position;

    rig.solve();

    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    const int spineJoint = findJoint(rig, "Spine");
    const int hipsJoint = findJoint(rig, "Hips");
    const int footJoint = findJoint(rig, "LeftFoot");
    ASSERT_GE(spineJoint, 0);
    ASSERT_GE(hipsJoint, 0);
    ASSERT_GE(footJoint, 0);

    // Reference solve without the spine limit: proves the clamp engages here,
    // so the assertions below are about the clamp path and not a no-op config.
    IkRig unclamped(Skeleton::makeDefault());
    unclamped.loadConfig(IkRigConfig::makeDefault());
    findTarget(unclamped, "Hips")->position = hipsPos;
    findTarget(unclamped, "Hips")->rotation = hipsRot;
    findTarget(unclamped, "LeftFoot")->position = footPos;
    findTarget(unclamped, "LeftFoot")->rotation = footRot;
    unclamped.solve();
    const WorldTransforms unclampedWt = computeWorldTransforms(unclamped.skeleton);
    EXPECT_GT(quatAngleRad(wt.rotations[spineJoint], unclampedWt.rotations[spineJoint]),
              glm::radians(3.0f));

    EXPECT_NEAR(quatAngleRad(wt.rotations[hipsJoint], hipsRot), 0.0f, 1e-2f);
    EXPECT_NEAR(quatAngleRad(wt.rotations[footJoint], footRot), 0.0f, 1e-2f);
}

TEST(IkRigSolve, StiffRootLimitKeepsLimbRotationsExact)
{
    // WP1, root case: the anchor root (Head) is an ancestor of every joint, so
    // restoring its tracked rotation after the clamp moves every limb's parent
    // frame. Limb end bones must still land on their tracked rotation.
    const glm::quat headRot = glm::angleAxis(glm::radians(45.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    const glm::quat footRot = glm::angleAxis(glm::radians(15.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::quat handRot = glm::angleAxis(glm::radians(-25.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    IkRigConfig config = IkRigConfig::makeDefault();
    config.limits.push_back(JointLimits{BoneNames::Head, 0.0f, 0.0f, 0.0f});  // fully stiff root

    IkRig rig(Skeleton::makeDefault());
    rig.loadConfig(config);
    findTarget(rig, "Head")->rotation = headRot;
    findTarget(rig, "LeftFoot")->rotation = footRot;
    findTarget(rig, "LeftHand")->rotation = handRot;

    rig.solve();

    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    const int headJoint = findJoint(rig, "Head");
    const int footJoint = findJoint(rig, "LeftFoot");
    const int handJoint = findJoint(rig, "LeftHand");
    ASSERT_GE(headJoint, 0);
    ASSERT_GE(footJoint, 0);
    ASSERT_GE(handJoint, 0);

    EXPECT_NEAR(quatAngleRad(wt.rotations[headJoint], headRot), 0.0f, 1e-2f);
    EXPECT_NEAR(quatAngleRad(wt.rotations[footJoint], footRot), 0.0f, 1e-2f);
    EXPECT_NEAR(quatAngleRad(wt.rotations[handJoint], handRot), 0.0f, 1e-2f);
}

TEST(IkRigConfigValidation, DuplicateTargetsThrow)
{
    IkRigConfig config;
    config.targets = {{"Head", SolverType::Anchor}, {"Head", SolverType::Chain}};
    EXPECT_THROW(config.validate(), std::runtime_error);
}

TEST(IkRigConfigValidation, DuplicateLimitsThrow)
{
    IkRigConfig config;
    config.limits = {JointLimits{"x", 0.0f, 0.0f, 90.0f}, JointLimits{"x", 0.0f, 0.0f, 90.0f}};
    EXPECT_THROW(config.validate(), std::runtime_error);
}

TEST(IkRigConfigValidation, ZeroPoleThrows)
{
    IkRigConfig config;
    JointLimits limit;
    limit.bone = "x";
    limit.pole = glm::vec3(0.0f);
    config.limits = {limit};
    EXPECT_THROW(config.validate(), std::runtime_error);
}

TEST(IkRigConfigValidation, TwistRangeThrows)
{
    IkRigConfig config;
    config.limits = {JointLimits{"x", 10.0f, -10.0f, 90.0f}};
    EXPECT_THROW(config.validate(), std::runtime_error);
}

TEST(IkRigConfigValidation, SwingConeOutOfRangeThrows)
{
    IkRigConfig config;
    config.limits = {JointLimits{"x", 0.0f, 0.0f, 270.0f}};
    EXPECT_THROW(config.validate(), std::runtime_error);
}

TEST(IkRigConfigValidation, DefaultConfigPasses)
{
    EXPECT_NO_THROW(IkRigConfig::makeDefault().validate());
}

// ---------------------------------------------------------------------------
// WP2 — dynamic bend normals from the hinge axis (fixes issues 1, 2, 3b).
//
// The bend normal = flexSign * cross(foot/hand lateral axis, chain aim). The
// cross product is perpendicular to the aim by construction, so there is no
// pole‖aim degeneracy. The lateral axis is invariant under foot pitch (the
// hinge axis itself), so a shin-mounted tracker's pitch cannot tilt it.
// flexSign is derived once at bind time from the static pole.
// ---------------------------------------------------------------------------

namespace
{
// Perpendicular offset of `knee` from the hip->ankle line (the bulge).
glm::vec3 jointBulge(const glm::vec3& socket, const glm::vec3& mid, const glm::vec3& tip)
{
    const glm::vec3 line = tip - socket;
    const float denom = glm::dot(line, line);
    if (denom < 1e-8f)
        return glm::vec3(0.0f);
    const float t = glm::dot(mid - socket, line) / denom;
    return mid - (socket + line * t);
}

void expectAllFinite(const std::vector<glm::vec3>& positions)
{
    for (size_t i = 0; i < positions.size(); ++i)
    {
        EXPECT_TRUE(std::isfinite(positions[i].x)) << "joint " << i;
        EXPECT_TRUE(std::isfinite(positions[i].y)) << "joint " << i;
        EXPECT_TRUE(std::isfinite(positions[i].z)) << "joint " << i;
    }
}
} // namespace

// Cross-legged (issue 2): hips forward, left foot tucked medial-forward at
// floor, foot yawed ~60° outward. The static pole (socket -Z) is near-
// parallel to the aim here; the dynamic bend normal (cross of the foot's
// lateral axis and the aim) puts the knee outward (lateral, +X for the left
// side), the correct cross-legged bulge — not upward/forward (the static
// result, which the abandoned blend reproduced as a no-op).
TEST(IkRigSolve, CrossLeggedKneeBulgesOutward)
{
    IkRig rig = makeDefaultRig();
    if (IkTarget* foot = findTarget(rig, BoneNames::LeftFoot))
    {
        foot->position = glm::vec3(-0.05f, 0.0f, -0.15f);  // medial, floor, forward
        foot->rotation = glm::angleAxis(glm::radians(-60.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    }
    rig.solve();
    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    expectAllFinite(wt.positions);
    const glm::vec3 hip = wt.positions[findJoint(rig, BoneNames::LeftHip)];
    const glm::vec3 knee = wt.positions[findJoint(rig, BoneNames::LeftUpperLeg)];
    const glm::vec3 ankle = wt.positions[findJoint(rig, BoneNames::LeftLowerLeg)];
    const glm::vec3 bulge = jointBulge(hip, knee, ankle);
    // Left side outward = +X. The cross product of the (yawed) foot lateral
    // axis with the aim gives +X here; the abandoned static/blend gave ~+Y.
    EXPECT_GT(bulge.x, 0.03f);
}

// Knee-sit (issue 1): kneeling, foot forward at floor, foot pitched ~180°
// (dorsiflexed, toes back). The abandoned approach NaN'd here (footForward
// ≈ +Z ≈ -staticPole → normalize(0)); the lateral axis is invariant under
// foot pitch (rotation about X doesn't move X), so the pole stays forward
// and finite, knee bulges forward (-Z).
TEST(IkRigSolve, KneeSitKneeBulgesForward)
{
    IkRig rig = makeDefaultRig();
    if (IkTarget* foot = findTarget(rig, BoneNames::LeftFoot))
    {
        foot->position = glm::vec3(0.10f, 0.0f, -0.30f);  // forward, floor
        foot->rotation = glm::angleAxis(glm::radians(160.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    }
    rig.solve();
    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    expectAllFinite(wt.positions);
    const glm::vec3 hip = wt.positions[findJoint(rig, BoneNames::LeftHip)];
    const glm::vec3 knee = wt.positions[findJoint(rig, BoneNames::LeftUpperLeg)];
    const glm::vec3 ankle = wt.positions[findJoint(rig, BoneNames::LeftLowerLeg)];
    const glm::vec3 bulge = jointBulge(hip, knee, ankle);
    EXPECT_LT(bulge.z, -0.03f);  // forward = -Z
}

// Foot yawed 180° opposite the hips: the abandoned approach's footForward
// opposed the static pole → normalize(0) → NaN. The cross product uses the
// lateral axis, which flips sign (outward becomes inward) but the cross
// product stays finite (flexSign was derived to match at rest, so it
// compensates). Assert all positions finite (no NaN).
TEST(IkRigSolve, DynamicKneePoleFiniteForFootYawOppositeHip)
{
    IkRig rig = makeDefaultRig();
    if (IkTarget* foot = findTarget(rig, BoneNames::LeftFoot))
    {
        foot->position = glm::vec3(0.10f, 0.05f, -0.20f);
        foot->rotation = glm::angleAxis(glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    }
    rig.solve();
    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    expectAllFinite(wt.positions);
}

// Continuity (3b): a 1mm foot-target delta must move the knee by a bounded
// amount (no 180° bend-plane flip). The cross product has no degeneracy to
// fall back from, so the bend plane is continuous.
TEST(IkRigSolve, DynamicKneeContinuousUnderSmallTargetDelta)
{
    const IkRigConfig config = IkRigConfig::makeDefault();
    const glm::quat footRot = glm::angleAxis(glm::radians(-30.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const auto solveKnee = [&](const glm::vec3& footPos) -> glm::vec3
    {
        IkRig rig(Skeleton::makeDefault());
        rig.loadConfig(config);
        if (IkTarget* foot = findTarget(rig, BoneNames::LeftFoot))
        {
            foot->position = footPos;
            foot->rotation = footRot;
        }
        rig.solve();
        const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
        return wt.positions[findJoint(rig, BoneNames::LeftUpperLeg)];
    };
    const glm::vec3 footPos(0.05f, 0.05f, -0.25f);
    const glm::vec3 kneeA = solveKnee(footPos);
    const glm::vec3 kneeB = solveKnee(footPos + glm::vec3(0.001f, 0.0f, 0.0f));
    expectVecNear(kneeA, kneeB, 0.02f);
}

// Elbow: raised hand. The dynamic bend normal from the hand's lateral axis
// must produce a finite, stable elbow, no flip on a 1mm hand-target delta.
TEST(IkRigSolve, DynamicElbowStableForRaisedHand)
{
    const IkRigConfig config = IkRigConfig::makeDefault();
    const glm::quat handRot = glm::angleAxis(glm::radians(30.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const auto solveElbow = [&](const glm::vec3& handPos) -> glm::vec3
    {
        IkRig rig(Skeleton::makeDefault());
        rig.loadConfig(config);
        if (IkTarget* hand = findTarget(rig, BoneNames::LeftHand))
        {
            hand->position = handPos;
            hand->rotation = handRot;
        }
        rig.solve();
        const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
        return wt.positions[findJoint(rig, BoneNames::LeftUpperArm)];
    };
    const glm::vec3 handPos(0.5f, 1.7f, -0.2f);
    const glm::vec3 elbow = solveElbow(handPos);
    const glm::vec3 elbowNudge = solveElbow(handPos + glm::vec3(0.001f, 0.0f, 0.0f));
    EXPECT_TRUE(std::isfinite(elbow.x) && std::isfinite(elbow.y) && std::isfinite(elbow.z));
    expectVecNear(elbow, elbowNudge, 0.02f);
}

// Elbow tracks the hand rotation, not world axes (the abandoned approach's
// bug). Two solves with the *same* goal (so the chain aim is identical) but
// the hand target rotated 90° yaw. The abandoned world-axis heuristic kept
// the pole world-fixed, so the bulge would be identical. The dynamic bend
// normal uses `handRot * lateral`, so the bulge must differ.
TEST(IkRigSolve, DynamicElbowRotatesWithHandRotation)
{
    const IkRigConfig config = IkRigConfig::makeDefault();
    // Fixed goal (where the hand ends up), hand position derived per rotation
    // so goal = handPos - handRot * handRestOffset stays constant.
    const glm::vec3 goal(0.35f, 1.30f, -0.25f);
    const glm::vec3 handRestOffset(0.12f, 0.0f, 0.0f);
    const auto solveElbowBulge = [&](const glm::quat& handRot) -> glm::vec3
    {
        IkRig rig(Skeleton::makeDefault());
        rig.loadConfig(config);
        if (IkTarget* hand = findTarget(rig, BoneNames::LeftHand))
        {
            hand->rotation = handRot;
            hand->position = goal + handRot * handRestOffset;
        }
        rig.solve();
        const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
        const glm::vec3 shoulder = wt.positions[findJoint(rig, BoneNames::LeftShoulder)];
        const glm::vec3 elbow = wt.positions[findJoint(rig, BoneNames::LeftUpperArm)];
        const glm::vec3 hand = wt.positions[findJoint(rig, BoneNames::LeftHand)];
        return jointBulge(shoulder, elbow, hand);
    };
    const glm::quat restRot(1.0f, 0.0f, 0.0f, 0.0f);
    const glm::quat yawedRot = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::vec3 bulgeRest = solveElbowBulge(restRot);
    const glm::vec3 bulgeYawed = solveElbowBulge(yawedRot);
    expectAllFinite({bulgeRest, bulgeYawed});
    // The bulge directions must differ — a 90° hand rotation rotates the
    // hinge axis, so the cross product (and thus the bulge) rotates with it.
    const float d = glm::dot(glm::normalize(bulgeRest), glm::normalize(bulgeYawed));
    EXPECT_LT(d, 0.9f);  // not the same direction
}

// --- Config serialization (carried from the abandoned attempt — the config
// layer was correct; only the solver was wrong). ---

TEST(IkRigConfigSerialization, PoleModeRoundTrip)
{
    IkRigConfig config;
    JointLimits limit;
    limit.bone = "x";
    limit.pole = glm::vec3(0.0f, 0.0f, -1.0f);
    limit.poleMode = PoleMode::DynamicFoot;
    config.limits.push_back(limit);
    const nlohmann::json j = config;
    const IkRigConfig parsed = j.get<IkRigConfig>();
    ASSERT_EQ(parsed.limits.size(), 1u);
    EXPECT_EQ(parsed.limits[0].poleMode, PoleMode::DynamicFoot);
}

TEST(IkRigConfigSerialization, LegacyConfigDefaultsToStaticPoleMode)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "limits": [ { "bone": "x", "pole": [0.0, 0.0, -1.0] } ]
    })");
    const IkRigConfig config = j.get<IkRigConfig>();
    ASSERT_EQ(config.limits.size(), 1u);
    EXPECT_EQ(config.limits[0].poleMode, PoleMode::Static);
}

TEST(IkRigConfigSerialization, DefaultConfigKneesAndElbowsAreDynamic)
{
    const IkRigConfig config = IkRigConfig::makeDefault();
    for (const JointLimits& limit : config.limits)
    {
        if (limit.bone == BoneNames::LeftLowerLeg || limit.bone == BoneNames::RightLowerLeg)
            EXPECT_EQ(limit.poleMode, PoleMode::DynamicFoot) << limit.bone;
        else if (limit.bone == BoneNames::LeftLowerArm || limit.bone == BoneNames::RightLowerArm)
            EXPECT_EQ(limit.poleMode, PoleMode::DynamicHand) << limit.bone;
    }
}
