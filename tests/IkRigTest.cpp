#include <gtest/gtest.h>
#include "model/IkRig.h"
#include "model/IkRigConfig.h"
#include "model/Skeleton.h"

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
            { "bone": "head", "solver": "anchor" },
            { "bone": "head", "solver": "chain" }
        ]
    })");
    EXPECT_THROW(j.get<IkRigConfig>(), std::runtime_error);
}

TEST(IkRigConfigDeserialization, LegacyStringTargetsThrow)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "targets": ["head", "hip"]
    })");
    EXPECT_THROW(j.get<IkRigConfig>(), std::runtime_error);
}

TEST(IkRigConfigDeserialization, InvalidPoleThrows)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "limits": [ { "bone": "x", "pole": [0.0, 0.0] } ]
    })");
    EXPECT_THROW(j.get<IkRigConfig>(), std::runtime_error);
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

TEST(IkRigConstruction, TargetsBindToJointsAndInitAtRestPose)
{
    IkRig rig(Skeleton::makeDefault(), IkRigConfig::makeDefault());

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

TEST(IkRigConstruction, UnknownTargetBoneThrows)
{
    IkRigConfig config;
    config.targets = {{"does_not_exist", SolverType::Anchor}};
    EXPECT_THROW(IkRig(Skeleton::makeDefault(), config), std::runtime_error);
}

TEST(IkRigConstruction, UnknownLimitBoneThrows)
{
    IkRigConfig config;
    config.limits.push_back(JointLimits{"does_not_exist", 0.0f, 0.0f, 90.0f});
    EXPECT_THROW(IkRig(Skeleton::makeDefault(), config), std::runtime_error);
}

TEST(IkRigConstruction, AnchorOnNonRootThrows)
{
    IkRigConfig config;
    config.targets = {{"hip", SolverType::Anchor}};
    EXPECT_THROW(IkRig(Skeleton::makeDefault(), config), std::runtime_error);
}

TEST(IkRigConstruction, ChainOnRootThrows)
{
    IkRigConfig config;
    config.targets = {{"head", SolverType::Chain}};
    EXPECT_THROW(IkRig(Skeleton::makeDefault(), config), std::runtime_error);
}

TEST(IkRigConstruction, TwoBoneWithTooFewAncestorsThrows)
{
    IkRigConfig config;
    config.targets = {{"upper_chest", SolverType::TwoBone}};  // head->neck->upper_chest: only 2
    EXPECT_THROW(IkRig(Skeleton::makeDefault(), config), std::runtime_error);
}

TEST(IkRigConstruction, TwoBoneWithoutMiddleBonePoleThrows)
{
    IkRigConfig config;
    config.targets = {{"left_foot", SolverType::TwoBone}};  // no limits entry -> no pole
    EXPECT_THROW(IkRig(Skeleton::makeDefault(), config), std::runtime_error);
}

TEST(IkRig, ResetTargetsRestoresRestPose)
{
    IkRig rig(Skeleton::makeDefault(), IkRigConfig::makeDefault());
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
    IkRig rig(Skeleton::makeDefault(), IkRigConfig::makeDefault());
    const std::vector<glm::vec3> rest = computeWorldPositions(rig.skeleton);

    rig.solve();

    const std::vector<glm::vec3> solved = computeWorldPositions(rig.skeleton);
    ASSERT_EQ(solved.size(), rest.size());
    for (size_t i = 0; i < rest.size(); ++i)
        expectVecNear(solved[i], rest[i], 1e-3f);
}

TEST(IkRigSolve, MovingAllTargetsTranslatesWholeSkeleton)
{
    IkRig rig(Skeleton::makeDefault(), IkRigConfig::makeDefault());
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
    IkRig rig(Skeleton::makeDefault(), IkRigConfig::makeDefault());
    IkTarget* head = findTarget(rig, "head");
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

TEST(IkRigSolve, FootTargetPlacesAnkleWithKneeForward)
{
    IkRig rig(Skeleton::makeDefault(), IkRigConfig::makeDefault());
    IkTarget* foot = findTarget(rig, "left_foot");
    ASSERT_NE(foot, nullptr);
    foot->position = glm::vec3(0.1f, 0.5f, -0.4f);

    rig.solve();

    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    const int ankle = findJoint(rig, "left_lower_leg");
    const int knee = findJoint(rig, "left_upper_leg");
    ASSERT_GE(ankle, 0);
    ASSERT_GE(knee, 0);

    // The ankle lands on the goal implied by the foot target.
    const glm::vec3 goal = foot->position - foot->rotation * rig.skeleton.joints[findJoint(rig, "left_foot")].restOffset;
    expectVecNear(wt.positions[ankle], goal, 1e-3f);
    // The knee bends forward (-Z), not backward.
    EXPECT_LT(wt.positions[knee].z, -0.1f);
}

TEST(IkRigSolve, HandTargetPlacesWrist)
{
    IkRig rig(Skeleton::makeDefault(), IkRigConfig::makeDefault());
    IkTarget* hand = findTarget(rig, "right_hand");
    ASSERT_NE(hand, nullptr);
    hand->position = glm::vec3(-0.5f, 1.3f, -0.3f);

    rig.solve();

    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    const int wrist = findJoint(rig, "right_lower_arm");
    ASSERT_GE(wrist, 0);
    const glm::vec3 goal = hand->position - hand->rotation * rig.skeleton.joints[findJoint(rig, "right_hand")].restOffset;
    expectVecNear(wt.positions[wrist], goal, 1e-3f);
}

TEST(IkRigSolve, HipTargetCrouchesSpine)
{
    IkRig rig(Skeleton::makeDefault(), IkRigConfig::makeDefault());
    IkTarget* hip = findTarget(rig, "hip");
    ASSERT_NE(hip, nullptr);
    hip->position = glm::vec3(0.0f, 1.3f, 0.0f);  // 0.4m below the head: spine must curl

    rig.solve();

    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    const int hipJoint = findJoint(rig, "hip");
    const int chest = findJoint(rig, "chest");
    ASSERT_GE(hipJoint, 0);
    ASSERT_GE(chest, 0);
    expectVecNear(wt.positions[hipJoint], hip->position, 0.01f);
    // The chest stays on the way between head and hip.
    EXPECT_GT(wt.positions[chest].y, 1.3f);
    EXPECT_LT(wt.positions[chest].y, 1.7f);
}

TEST(IkRigSolve, OverreachedFootStretchesLegWithoutExploding)
{
    IkRig rig(Skeleton::makeDefault(), IkRigConfig::makeDefault());
    IkTarget* foot = findTarget(rig, "left_foot");
    ASSERT_NE(foot, nullptr);
    foot->position = glm::vec3(0.1f, 0.02f, -3.0f);  // far beyond leg length

    rig.solve();

    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    const int socket = findJoint(rig, "left_hip");
    const int ankle = findJoint(rig, "left_lower_leg");
    // Leg fully extended: ankle at chain length from the socket, toward the goal.
    EXPECT_NEAR(glm::length(wt.positions[ankle] - wt.positions[socket]), 0.9f, 1e-3f);
    const glm::vec3 goal = foot->position - foot->rotation * rig.skeleton.joints[findJoint(rig, "left_foot")].restOffset;
    const glm::vec3 aim = glm::normalize(goal - wt.positions[socket]);
    const glm::vec3 ankleDir = glm::normalize(wt.positions[ankle] - wt.positions[socket]);
    EXPECT_NEAR(glm::dot(ankleDir, aim), 1.0f, 1e-3f);
}
