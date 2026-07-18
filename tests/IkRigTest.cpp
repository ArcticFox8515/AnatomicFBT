#include <gtest/gtest.h>
#include "model/IkRig.h"
#include "model/IkRigConfig.h"
#include "model/Skeleton.h"

TEST(IkRigConfigSerialization, DefaultRoundTrip)
{
    const IkRigConfig original = IkRigConfig::makeDefault();
    const nlohmann::json j = original;
    const IkRigConfig parsed = j.get<IkRigConfig>();

    EXPECT_EQ(parsed.targetBones, original.targetBones);
    ASSERT_EQ(parsed.limits.size(), original.limits.size());
    for (size_t i = 0; i < original.limits.size(); ++i)
    {
        EXPECT_EQ(parsed.limits[i].bone, original.limits[i].bone);
        EXPECT_FLOAT_EQ(parsed.limits[i].twistMinDeg, original.limits[i].twistMinDeg);
        EXPECT_FLOAT_EQ(parsed.limits[i].twistMaxDeg, original.limits[i].twistMaxDeg);
        EXPECT_FLOAT_EQ(parsed.limits[i].swingConeDeg, original.limits[i].swingConeDeg);
    }
}

TEST(IkRigConfigDeserialization, DuplicateTargetsThrow)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "targets": ["head", "head"]
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
    EXPECT_TRUE(config.targetBones.empty());
    EXPECT_TRUE(config.limits.empty());
}

TEST(IkRigConstruction, TargetsBindToJointsAndInitAtRestPose)
{
    IkRig rig(Skeleton::makeDefault(), IkRigConfig::makeDefault());

    ASSERT_EQ(rig.targets.size(), rig.config.targetBones.size());
    const std::vector<glm::vec3> restPositions = computeWorldPositions(rig.skeleton);
    for (size_t i = 0; i < rig.targets.size(); ++i)
    {
        const IkTarget& target = rig.targets[i];
        EXPECT_EQ(rig.skeleton.joints[target.jointIndex].name, rig.config.targetBones[i]);
        EXPECT_EQ(target.position, restPositions[target.jointIndex]);
        EXPECT_EQ(target.rotation, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    }
}

TEST(IkRigConstruction, UnknownTargetBoneThrows)
{
    IkRigConfig config;
    config.targetBones = {"does_not_exist"};
    EXPECT_THROW(IkRig(Skeleton::makeDefault(), config), std::runtime_error);
}

TEST(IkRigConstruction, UnknownLimitBoneThrows)
{
    IkRigConfig config;
    config.limits.push_back(JointLimits{"does_not_exist", 0.0f, 0.0f, 90.0f});
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
