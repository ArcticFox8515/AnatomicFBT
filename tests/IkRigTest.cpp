#include <gtest/gtest.h>
#include "model/IkRig.h"
#include "model/IkRigConfig.h"
#include "model/Skeleton.h"

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
    hip->position = glm::vec3(0.0f, 0.95f, 0.0f);
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
