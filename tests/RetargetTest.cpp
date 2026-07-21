#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "model/Retarget.h"
#include "model/Skeleton.h"

namespace
{
int indexOf(const Skeleton& skeleton, const std::string& name)
{
    for (size_t i = 0; i < skeleton.joints.size(); ++i)
        if (skeleton.joints[i].name == name)
            return static_cast<int>(i);
    return -1;
}

void expectNearVec3(const glm::vec3& a, const glm::vec3& b, float eps)
{
    EXPECT_NEAR(a.x, b.x, eps);
    EXPECT_NEAR(a.y, b.y, eps);
    EXPECT_NEAR(a.z, b.z, eps);
}

void expectSameRotation(const glm::quat& a, const glm::quat& b, float eps)
{
    EXPECT_NEAR(glm::abs(glm::dot(a, b)), 1.0f, eps);
}
} // namespace

TEST(HipRootedDefaultSkeleton, SameRestPoseAsDefault)
{
    const Skeleton headRooted = Skeleton::makeDefault();
    const Skeleton hipRooted = Skeleton::makeDefaultHipRooted();

    ASSERT_EQ(hipRooted.joints.size(), headRooted.joints.size());

    int rootCount = 0;
    for (size_t i = 0; i < hipRooted.joints.size(); ++i)
    {
        if (!hipRooted.joints[i].parentIndex)
        {
            ++rootCount;
            EXPECT_EQ(hipRooted.joints[i].name, "Hips");
        }
        else
        {
            EXPECT_LT(*hipRooted.joints[i].parentIndex, static_cast<int>(i))
                << "parent must come before child";
        }
    }
    EXPECT_EQ(rootCount, 1);

    // The spine chain now runs upward: head's parent is the neck.
    const int headIndex = indexOf(hipRooted, "Head");
    ASSERT_GE(headIndex, 0);
    ASSERT_TRUE(hipRooted.joints[headIndex].parentIndex.has_value());
    EXPECT_EQ(*hipRooted.joints[headIndex].parentIndex, indexOf(hipRooted, "Neck"));

    // Rest world positions are unchanged by the rerooting.
    const std::vector<glm::vec3> restPositions = computeWorldPositions(headRooted);
    const std::vector<glm::vec3> hipPositions = computeWorldPositions(hipRooted);
    for (size_t i = 0; i < hipRooted.joints.size(); ++i)
    {
        const int srcIndex = indexOf(headRooted, hipRooted.joints[i].name);
        ASSERT_GE(srcIndex, 0);
        expectNearVec3(hipPositions[i], restPositions[static_cast<size_t>(srcIndex)], 1e-6f);
    }

    // The hip-rooted root sits where the head-rooted skeleton's Hips joint rests.
    expectNearVec3(hipRooted.rootPosition,
                   restPositions[static_cast<size_t>(indexOf(headRooted, "Hips"))], 1e-6f);
}

TEST(RetargetPose, ReproducesWorldPoseAcrossDifferentRoots)
{
    Skeleton src = Skeleton::makeDefault();
    src.rootPosition = glm::vec3(0.3f, 1.9f, -0.2f);
    const glm::quat yaw = glm::angleAxis(glm::radians(35.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::quat pitch = glm::angleAxis(glm::radians(-20.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    src.joints[indexOf(src, "Head")].localRot = yaw * pitch;
    // Twist about the bone axis on the spine: must survive the chain reversal.
    src.joints[indexOf(src, "Neck")].localRot =
        glm::angleAxis(glm::radians(40.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    src.joints[indexOf(src, "Waist")].localRot =
        glm::angleAxis(glm::radians(15.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    src.joints[indexOf(src, "LeftUpperLeg")].localRot =
        glm::angleAxis(glm::radians(50.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    src.joints[indexOf(src, "LeftLowerLeg")].localRot =
        glm::angleAxis(glm::radians(-60.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    src.joints[indexOf(src, "RightUpperArm")].localRot =
        glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    Skeleton dst = Skeleton::makeDefaultHipRooted();
    const RetargetMap map = buildRetargetMap(src, dst);
    EXPECT_TRUE(unmatchedBones(dst, map).empty());

    retargetPose(src, dst, map);

    // Every joint must land on the same world position, and every matched bone
    // must have the same world rotation, despite the opposite hierarchy.
    const WorldTransforms srcWorld = computeWorldTransforms(src);
    const WorldTransforms dstWorld = computeWorldTransforms(dst);
    for (size_t i = 0; i < dst.joints.size(); ++i)
    {
        SCOPED_TRACE(dst.joints[i].name);
        const int srcIndex = indexOf(src, dst.joints[i].name);
        ASSERT_GE(srcIndex, 0);
        expectNearVec3(dstWorld.positions[i], srcWorld.positions[static_cast<size_t>(srcIndex)], 1e-4f);
        if (map.dstToSrc[i])
            expectSameRotation(dstWorld.rotations[i],
                srcWorld.rotations[static_cast<size_t>(*map.dstToSrc[i])], 1e-4f);
    }
}

TEST(RetargetPose, UnmatchedBonesStayAtRest)
{
    Skeleton src = Skeleton::makeDefault();
    Skeleton dst = Skeleton::makeDefaultHipRooted();

    // Extra bones on both sides; appended last, so parent-before-child holds.
    Joint cape;
    cape.name = "cape";
    cape.parentIndex = indexOf(dst, "Hips");
    cape.restOffset = glm::vec3(0.0f, 0.0f, -0.3f);
    dst.joints.push_back(cape);

    Joint antenna;
    antenna.name = "antenna";
    antenna.parentIndex = indexOf(src, "Head");
    antenna.restOffset = glm::vec3(0.0f, 0.2f, 0.0f);
    src.joints.push_back(antenna);

    const RetargetMap map = buildRetargetMap(src, dst);
    const std::vector<std::string> unmatched = unmatchedBones(dst, map);
    ASSERT_EQ(unmatched.size(), 1u);
    EXPECT_EQ(unmatched[0], "cape");

    src.joints[indexOf(src, "Waist")].localRot =
        glm::angleAxis(glm::radians(30.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    retargetPose(src, dst, map);

    constexpr glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);
    expectSameRotation(dst.joints[indexOf(dst, "cape")].localRot, identity, 1e-6f);
}

TEST(RetargetPose, AnchorsHeadWithDifferentProportions)
{
    Skeleton src = Skeleton::makeDefault();
    src.joints[indexOf(src, "Waist")].localRot =
        glm::angleAxis(glm::radians(20.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    src.joints[indexOf(src, "LeftUpperLeg")].localRot =
        glm::angleAxis(glm::radians(35.0f), glm::vec3(1.0f, 0.0f, 0.0f));

    // Same topology, 20% longer bones.
    Skeleton dst = Skeleton::makeDefaultHipRooted();
    for (Joint& joint : dst.joints)
        if (joint.parentIndex)
            joint.restOffset *= 1.2f;

    const RetargetMap map = buildRetargetMap(src, dst);
    retargetPose(src, dst, map);

    const WorldTransforms srcWorld = computeWorldTransforms(src);
    const WorldTransforms dstWorld = computeWorldTransforms(dst);

    // The head (HMD anchor) lands exactly...
    expectNearVec3(dstWorld.positions[indexOf(dst, "Head")],
        srcWorld.positions[indexOf(src, "Head")], 1e-4f);
    // ...rotations still transfer by name...
    for (size_t i = 0; i < dst.joints.size(); ++i)
    {
        SCOPED_TRACE(dst.joints[i].name);
        if (map.dstToSrc[i])
            expectSameRotation(dstWorld.rotations[i],
                srcWorld.rotations[static_cast<size_t>(*map.dstToSrc[i])], 1e-4f);
    }
    // ...but the proportion difference shows up at the feet.
    const float footDistance = glm::distance(dstWorld.positions[indexOf(dst, "LeftFoot")],
        srcWorld.positions[indexOf(src, "LeftFoot")]);
    EXPECT_GT(footDistance, 0.01f);
}

TEST(RetargetPose, NoMatchingNamesLeavesDstUntouched)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "bones": [
            { "name": "pelvis", "parent": null, "offset": [0.0, 1.0, 0.0] },
            { "name": "spine", "parent": "pelvis", "offset": [0.0, 0.2, 0.0] }
        ]
    })");
    Skeleton dst = j.get<Skeleton>();
    Skeleton src = Skeleton::makeDefault();
    src.joints[indexOf(src, "Waist")].localRot =
        glm::angleAxis(glm::radians(30.0f), glm::vec3(1.0f, 0.0f, 0.0f));

    const RetargetMap map = buildRetargetMap(src, dst);
    EXPECT_EQ(unmatchedBones(dst, map).size(), 2u);

    const glm::vec3 rootBefore = dst.rootPosition;
    retargetPose(src, dst, map);

    EXPECT_EQ(dst.rootPosition, rootBefore);
    constexpr glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);
    for (const Joint& joint : dst.joints)
        expectSameRotation(joint.localRot, identity, 1e-6f);
}
