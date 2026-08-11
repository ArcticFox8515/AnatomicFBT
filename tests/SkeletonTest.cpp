#include <gtest/gtest.h>
#include "model/Skeleton.h"
#include "model/BodyProportions.h"
#include "model/BoneNames.h"
#include "model/IkMath.h"

#include <glm/gtc/quaternion.hpp>
#include <cmath>

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
    EXPECT_EQ(skeleton.joints.front().name, "Head");
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

// ---- rest height / scaling -------------------------------------------------

TEST(RestHeight, EqualsHeadToFeetSpan)
{
    BodyProportions p;
    p.shoulderHeight = 1.40f;
    p.neckLength = 0.30f;
    p.upperLegLength = 0.50f;
    p.lowerLegLength = 0.50f;
    p.navelHeight = 1.10f;
    const Skeleton skeleton = Skeleton::makeDefault(p);

    // Head (the root) sits at the top; the span down to the feet includes the
    // Head->Neck bone (0.15, an internal constant) on top of shoulderHeight +
    // neckLength, since the root is the Head joint itself.
    constexpr float kHeadBoneLength = 0.15f;
    EXPECT_NEAR(restHeight(skeleton), p.shoulderHeight + p.neckLength + kHeadBoneLength, 1e-5f);
}

TEST(RestHeight, HeadRootedAndHipRootedDefaultAgree)
{
    const Skeleton headRooted = Skeleton::makeDefault();
    const Skeleton hipRooted = Skeleton::makeDefaultHipRooted();

    EXPECT_NEAR(restHeight(headRooted), restHeight(hipRooted), 1e-5f);
}

TEST(RestHeight, IsPoseIndependent)
{
    Skeleton skeleton = Skeleton::makeDefault();
    // Pose the skeleton arbitrarily — restHeight must not move.
    const float before = restHeight(skeleton);
    for (Joint& joint : skeleton.joints)
        joint.localRot = glm::angleAxis(0.5f, glm::vec3(0.0f, 1.0f, 0.0f));
    skeleton.rootPosition = glm::vec3(10.0f, 20.0f, 30.0f);
    EXPECT_NEAR(restHeight(skeleton), before, 1e-5f);
}

TEST(RestHeight, MissingHeadReturnsZero)
{
    Skeleton skeleton = Skeleton::makeDefaultHipRooted();
    const int head = [&]
    {
        for (size_t i = 0; i < skeleton.joints.size(); ++i)
            if (skeleton.joints[i].name == BoneNames::Head)
                return static_cast<int>(i);
        return -1;
    }();
    ASSERT_GE(head, 0);
    skeleton.joints[static_cast<size_t>(head)].name = "HeadX";
    EXPECT_EQ(restHeight(skeleton), 0.0f);
}

TEST(ScaleSkeleton, ScalesRestOffsetsAndRootPosition)
{
    Skeleton skeleton = Skeleton::makeDefault();
    const Skeleton original = skeleton;
    const float scale = 0.9f;
    scaleSkeleton(skeleton, scale);

    for (size_t i = 0; i < skeleton.joints.size(); ++i)
        EXPECT_EQ(skeleton.joints[i].restOffset, original.joints[i].restOffset * scale);
    EXPECT_EQ(skeleton.rootPosition, original.rootPosition * scale);
}

TEST(ScaleSkeleton, ScalesRestHeight)
{
    Skeleton skeleton = Skeleton::makeDefault();
    const float before = restHeight(skeleton);
    scaleSkeleton(skeleton, 1.2f);
    EXPECT_NEAR(restHeight(skeleton), before * 1.2f, 1e-5f);
}

TEST(MatchRestHeight, ScalesAvatarToReference)
{
    BodyProportions ref;
    ref.shoulderHeight = 1.60f;
    ref.neckLength = 0.25f;
    ref.upperLegLength = 0.50f;
    ref.lowerLegLength = 0.50f;
    ref.navelHeight = 1.10f;
    const Skeleton reference = Skeleton::makeDefault(ref);

    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    const float refHeight = restHeight(reference);
    const float avatarHeightBefore = restHeight(avatar);

    const float scale = matchRestHeight(reference, avatar);

    EXPECT_NEAR(scale, refHeight / avatarHeightBefore, 1e-5f);
    EXPECT_NEAR(restHeight(avatar), refHeight, 1e-5f);
}

TEST(MatchRestHeight, MissingHeadLeavesAvatarUntouched)
{
    const Skeleton reference = Skeleton::makeDefault();
    Skeleton avatar = Skeleton::makeDefaultHipRooted();
    const Skeleton avatarCopy = avatar;
    // Remove Head from the reference so its restHeight is 0.
    Skeleton badRef = reference;
    for (Joint& joint : badRef.joints)
        if (joint.name == BoneNames::Head)
            joint.name = "HeadX";

    const float scale = matchRestHeight(badRef, avatar);

    EXPECT_EQ(scale, 1.0f);
    ASSERT_EQ(avatar.joints.size(), avatarCopy.joints.size());
    for (size_t i = 0; i < avatar.joints.size(); ++i)
        EXPECT_EQ(avatar.joints[i].restOffset, avatarCopy.joints[i].restOffset);
    EXPECT_EQ(avatar.rootPosition, avatarCopy.rootPosition);
}

// ---- computeBoneFrames ------------------------------------------------------

namespace
{
Skeleton downwardChainSkeleton()
{
    // root -> child, child offset straight down (-Y). The downward bone is the
    // degenerate case for a minimal-rotation-from-+Y renderer.
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "bones": [
            { "name": "root", "parent": null, "offset": [0.0, 0.0, 0.0] },
            { "name": "child", "parent": "root", "offset": [0.0, -1.0, 0.0] }
        ]
    })");
    return j.get<Skeleton>();
}

float angleBetween(const glm::quat& a, const glm::quat& b)
{
    return 2.0f * std::acos(std::clamp(std::abs(glm::dot(a, b)), 0.0f, 1.0f));
}
} // namespace

TEST(ComputeBoneFrames, OmitsRootAndZeroLengthBones)
{
    const nlohmann::json j = nlohmann::json::parse(R"(
    {
        "bones": [
            { "name": "root", "parent": null, "offset": [0.0, 0.0, 0.0] },
            { "name": "zero", "parent": "root", "offset": [0.0, 0.0, 0.0] },
            { "name": "child", "parent": "root", "offset": [0.0, -1.0, 0.0] }
        ]
    })");
    const Skeleton skeleton = j.get<Skeleton>();
    const std::vector<BoneFrame> frames = computeBoneFrames(skeleton);
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0].joint, 2);
    EXPECT_FLOAT_EQ(frames[0].length, 1.0f);
    EXPECT_EQ(frames[0].base, glm::vec3(0.0f));
}

TEST(ComputeBoneFrames, RestPoseYAxisMapsOntoBoneDirection)
{
    const Skeleton skeleton = Skeleton::makeDefault();
    const std::vector<BoneFrame> frames = computeBoneFrames(skeleton);
    const WorldTransforms wt = computeWorldTransforms(skeleton);
    for (const BoneFrame& f : frames)
    {
        const glm::vec3 mapped = f.rotation * glm::vec3(0.0f, 1.0f, 0.0f);
        const glm::vec3 expected = glm::normalize(wt.positions[static_cast<size_t>(f.joint)] - f.base);
        EXPECT_NEAR(glm::dot(mapped, expected), 1.0f, 1e-5f) << "bone joint " << f.joint;
        EXPECT_NEAR(glm::length(mapped), 1.0f, 1e-5f);
    }
}

TEST(ComputeBoneFrames, ReflectsJointTwist)
{
    // A pure twist of the joint's localRot about the bone axis must rotate the
    // bone frame by the same angle about that axis — the renderer now shows
    // real twist, which is the point of the fix.
    Skeleton skeleton = downwardChainSkeleton();
    const std::vector<BoneFrame> restFrames = computeBoneFrames(skeleton);
    ASSERT_EQ(restFrames.size(), 1u);
    const glm::quat restFrame = restFrames[0].rotation;

    constexpr float kTwistDeg = 30.0f;
    const glm::vec3 boneAxisLocal = glm::normalize(skeleton.joints[1].restOffset); // (0,-1,0)
    skeleton.joints[1].localRot = glm::angleAxis(glm::radians(kTwistDeg), boneAxisLocal);
    const std::vector<BoneFrame> twistedFrames = computeBoneFrames(skeleton);
    ASSERT_EQ(twistedFrames.size(), 1u);

    // Direction unchanged (pure twist): base and +Y mapping stay the same.
    EXPECT_NEAR(glm::length(twistedFrames[0].rotation * glm::vec3(0.0f, 1.0f, 0.0f)
                             - restFrame * glm::vec3(0.0f, 1.0f, 0.0f)), 0.0f, 1e-5f);

    // Frame rotation differs from rest by the twist angle about the bone axis.
    const glm::quat delta = glm::normalize(twistedFrames[0].rotation * glm::inverse(restFrame));
    EXPECT_NEAR(glm::degrees(angleBetween(delta, glm::quat(1.0f, 0.0f, 0.0f, 0.0f))), kTwistDeg, 1e-2f);
}

TEST(ComputeBoneFrames, RollStaysContinuousNearVerticalDownBone)
{
    // Regression guard: sweeping the bone direction's azimuth at ~1 deg off the
    // -Y antipode must produce bounded frame-rotation deltas. The previous
    // renderer (minimal rotation from +Y) gave ~90 deg of roll per 45 deg of
    // azimuth here; computeBoneFrames composes with a constant per-bone factor,
    // so the frame follows the joint rotation continuously.
    for (int az = 0; az < 360; az += 45)
    {
        Skeleton skeleton = downwardChainSkeleton();
        const float azRad = glm::radians(static_cast<float>(az));
        const float tiltRad = glm::radians(1.0f);
        const glm::vec3 dir(std::sin(tiltRad) * std::cos(azRad),
                            -std::cos(tiltRad),
                            std::sin(tiltRad) * std::sin(azRad));
        const glm::vec3 restDir = glm::normalize(skeleton.joints[1].restOffset);
        skeleton.joints[1].localRot = quatFromTo(restDir, dir);

        const int nextAz = (az + 45) % 360;
        Skeleton next = downwardChainSkeleton();
        const float nextAzRad = glm::radians(static_cast<float>(nextAz));
        const glm::vec3 nextDir(std::sin(tiltRad) * std::cos(nextAzRad),
                                -std::cos(tiltRad),
                                std::sin(tiltRad) * std::sin(nextAzRad));
        next.joints[1].localRot = quatFromTo(restDir, nextDir);

        const glm::quat q0 = computeBoneFrames(skeleton)[0].rotation;
        const glm::quat q1 = computeBoneFrames(next)[0].rotation;
        // 45 deg of azimuth must not flip the frame by ~90 deg. Allow the
        // legitimate ~0.7 deg direction change plus a small margin.
        EXPECT_LT(glm::degrees(angleBetween(q0, q1)), 5.0f) << "az " << az << "->" << nextAz;
    }
}
