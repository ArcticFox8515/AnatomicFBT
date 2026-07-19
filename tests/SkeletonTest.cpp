#include <gtest/gtest.h>
#include "model/Skeleton.h"

TEST(SkeletonSerialization, DefaultRoundTrip)
{
    const Skeleton original = Skeleton::makeDefault();
    const nlohmann::json j = original;
    const Skeleton parsed = j.get<Skeleton>();

    ASSERT_EQ(parsed.joints.size(), original.joints.size());
    for (size_t i = 0; i < original.joints.size(); ++i)
    {
        EXPECT_EQ(parsed.joints[i].name, original.joints[i].name);
        EXPECT_EQ(parsed.joints[i].parentIndex, original.joints[i].parentIndex);
        EXPECT_EQ(parsed.joints[i].restOffset, original.joints[i].restOffset);
        EXPECT_EQ(parsed.joints[i].localRot, original.joints[i].localRot);
    }
}

TEST(SkeletonSerialization, StringRoundTrip)
{
    const Skeleton original = Skeleton::makeDefault();
    const std::string text = nlohmann::json(original).dump();
    const Skeleton parsed = nlohmann::json::parse(text).get<Skeleton>();

    ASSERT_EQ(parsed.joints.size(), original.joints.size());
    EXPECT_EQ(parsed.joints.front().name, original.joints.front().name);
    EXPECT_EQ(parsed.joints.back().name, original.joints.back().name);
}

TEST(SkeletonDeserialization, SortsParentBeforeChild)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "bones": [
            { "name": "grandchild", "parent": "child", "offset": [1.0, 0.0, 0.0] },
            { "name": "child", "parent": "root", "offset": [0.0, 1.0, 0.0] },
            { "name": "root", "parent": null, "offset": [0.0, 0.0, 0.0] }
        ]
    })");

    const Skeleton skeleton = j.get<Skeleton>();
    ASSERT_EQ(skeleton.joints.size(), 3u);
    EXPECT_EQ(skeleton.joints[0].name, "root");
    EXPECT_EQ(skeleton.joints[0].parentIndex, std::nullopt);
    EXPECT_EQ(skeleton.joints[1].name, "child");
    EXPECT_EQ(skeleton.joints[1].parentIndex, 0);
    EXPECT_EQ(skeleton.joints[2].name, "grandchild");
    EXPECT_EQ(skeleton.joints[2].parentIndex, 1);
    EXPECT_EQ(skeleton.joints[1].restOffset, glm::vec3(0.0f, 1.0f, 0.0f));
    EXPECT_EQ(skeleton.joints[2].restOffset, glm::vec3(1.0f, 0.0f, 0.0f));
}

TEST(SkeletonDeserialization, MissingFieldsGetDefaults)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "bones": [
            { "name": "root" },
            { "name": "child", "parent": "root" }
        ]
    })");

    const Skeleton skeleton = j.get<Skeleton>();
    ASSERT_EQ(skeleton.joints.size(), 2u);
    EXPECT_EQ(skeleton.joints[0].parentIndex, std::nullopt);
    EXPECT_EQ(skeleton.joints[0].restOffset, glm::vec3(0.0f));
    EXPECT_EQ(skeleton.joints[1].restOffset, glm::vec3(0.0f));
}

TEST(SkeletonDeserialization, RejectsUnknownParent)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "bones": [
            { "name": "root", "parent": null },
            { "name": "child", "parent": "ghost" }
        ]
    })");
    EXPECT_THROW(j.get<Skeleton>(), std::runtime_error);
}

TEST(SkeletonDeserialization, RejectsDuplicateNames)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "bones": [
            { "name": "root", "parent": null },
            { "name": "root", "parent": "root" }
        ]
    })");
    EXPECT_THROW(j.get<Skeleton>(), std::runtime_error);
}

TEST(SkeletonDeserialization, RejectsMultipleRoots)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "bones": [
            { "name": "a", "parent": null },
            { "name": "b", "parent": null }
        ]
    })");
    EXPECT_THROW(j.get<Skeleton>(), std::runtime_error);
}

TEST(SkeletonDeserialization, RejectsCycles)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "bones": [
            { "name": "root", "parent": null },
            { "name": "a", "parent": "b" },
            { "name": "b", "parent": "a" }
        ]
    })");
    EXPECT_THROW(j.get<Skeleton>(), std::runtime_error);
}

TEST(SkeletonDeserialization, RejectsEmptySkeleton)
{
    const nlohmann::json j = nlohmann::json::parse(R"({ "bones": [] })");
    EXPECT_THROW(j.get<Skeleton>(), std::runtime_error);
}

TEST(SkeletonDeserialization, MissingBonesKeyThrows)
{
    const nlohmann::json j = nlohmann::json::parse("{}");
    EXPECT_THROW(j.get<Skeleton>(), nlohmann::json::exception);
}

TEST(SkeletonDeserialization, BonesMustBeArray)
{
    const nlohmann::json j = nlohmann::json::parse(R"({ "bones": {} })");
    EXPECT_THROW(j.get<Skeleton>(), nlohmann::json::exception);
}

TEST(SkeletonDeserialization, OffsetMustBeThreeNumbers)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "bones": [ { "name": "root", "offset": [0.0, 1.0] } ]
    })");
    EXPECT_THROW(j.get<Skeleton>(), nlohmann::json::exception);
}

TEST(SkeletonDeserialization, MissingNameThrows)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "bones": [ { "parent": null } ]
    })");
    EXPECT_THROW(j.get<Skeleton>(), nlohmann::json::exception);
}

TEST(DefaultSkeleton, IsValidAndOrdered)
{
    const Skeleton skeleton = Skeleton::makeDefault();
    ASSERT_FALSE(skeleton.joints.empty());

    int rootCount = 0;
    for (size_t i = 0; i < skeleton.joints.size(); ++i)
    {
        if (skeleton.joints[i].parentIndex < 0)
            ++rootCount;
        else
            EXPECT_LT(skeleton.joints[i].parentIndex, static_cast<int>(i)) << "parent must come before child";
    }
    EXPECT_EQ(rootCount, 1);
    EXPECT_EQ(skeleton.joints.front().name, "head");
}

TEST(ComputeWorldPositions, AccumulatesOffsets)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "bones": [
            { "name": "root", "parent": null, "offset": [0.0, 1.0, 0.0] },
            { "name": "child", "parent": "root", "offset": [0.0, -0.5, 0.0] },
            { "name": "grandchild", "parent": "child", "offset": [0.25, 0.0, 0.0] }
        ]
    })");
    const Skeleton skeleton = j.get<Skeleton>();
    const std::vector<glm::vec3> positions = computeWorldPositions(skeleton);

    ASSERT_EQ(positions.size(), 3u);
    EXPECT_EQ(positions[0], glm::vec3(0.0f, 1.0f, 0.0f));
    EXPECT_EQ(positions[1], glm::vec3(0.0f, 0.5f, 0.0f));
    EXPECT_EQ(positions[2], glm::vec3(0.25f, 0.5f, 0.0f));
}

TEST(ComputeWorldTransforms, RootSitsAtRootPosition)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "bones": [
            { "name": "root", "parent": null, "offset": [0.0, 1.0, 0.0] },
            { "name": "child", "parent": "root", "offset": [0.0, -0.5, 0.0] }
        ]
    })");
    Skeleton skeleton = j.get<Skeleton>();

    // rootPosition is seeded from the root's restOffset.
    EXPECT_EQ(skeleton.rootPosition, glm::vec3(0.0f, 1.0f, 0.0f));

    // ...but FK places the root at rootPosition and ignores the restOffset.
    skeleton.rootPosition = glm::vec3(5.0f, 6.0f, 7.0f);
    const WorldTransforms wt = computeWorldTransforms(skeleton);
    EXPECT_EQ(wt.positions[0], glm::vec3(5.0f, 6.0f, 7.0f));
    EXPECT_EQ(wt.positions[1], glm::vec3(5.0f, 5.5f, 7.0f));
}

TEST(ComputeWorldTransforms, RotationsAccumulateHierarchically)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "bones": [
            { "name": "root", "parent": null, "offset": [0.0, 1.0, 0.0] },
            { "name": "child", "parent": "root", "offset": [0.0, -0.5, 0.0] },
            { "name": "grandchild", "parent": "child", "offset": [0.25, 0.0, 0.0] }
        ]
    })");
    Skeleton skeleton = j.get<Skeleton>();

    const glm::quat rootRot = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    skeleton.joints[0].localRot = rootRot;

    const WorldTransforms wt = computeWorldTransforms(skeleton);

    // The child's offset is rotated by the root; the grandchild inherits both.
    EXPECT_NEAR(wt.positions[1].x, 0.5f, 1e-5f);
    EXPECT_NEAR(wt.positions[1].y, 1.0f, 1e-5f);
    EXPECT_NEAR(wt.positions[2].x, 0.5f, 1e-5f);
    EXPECT_NEAR(wt.positions[2].y, 1.25f, 1e-5f);
    EXPECT_NEAR(glm::abs(glm::dot(wt.rotations[1], rootRot)), 1.0f, 1e-5f);
    EXPECT_NEAR(glm::abs(glm::dot(wt.rotations[2], rootRot)), 1.0f, 1e-5f);
}
