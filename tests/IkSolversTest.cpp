#include <gtest/gtest.h>
#include "model/IkSolvers.h"
#include "model/Skeleton.h"

#include <cmath>
#include <glm/gtc/quaternion.hpp>

namespace
{
int findJoint(const Skeleton& skeleton, const std::string& bone)
{
    for (size_t i = 0; i < skeleton.joints.size(); ++i)
        if (skeleton.joints[i].name == bone)
            return static_cast<int>(i);
    return -1;
}

void expectVecNear(const glm::vec3& actual, const glm::vec3& expected, float tol)
{
    EXPECT_NEAR(actual.x, expected.x, tol);
    EXPECT_NEAR(actual.y, expected.y, tol);
    EXPECT_NEAR(actual.z, expected.z, tol);
}

// Vertical 3-link chain hanging from a root at y = 1: root -> a -> b -> c,
// each link 0.3 m long.
Skeleton makeChainSkeleton()
{
    Skeleton skeleton;
    auto add = [&skeleton](std::string name, std::optional<int> parent, glm::vec3 offset)
    {
        Joint joint;
        joint.name = std::move(name);
        joint.parentIndex = parent;
        joint.restOffset = offset;
        skeleton.joints.push_back(std::move(joint));
    };
    add("root", std::nullopt, {0.0f, 1.0f, 0.0f});
    add("a", 0, {0.0f, -0.3f, 0.0f});
    add("b", 1, {0.0f, -0.3f, 0.0f});
    add("c", 2, {0.0f, -0.3f, 0.0f});
    skeleton.rootPosition = skeleton.joints[0].restOffset;
    return skeleton;
}
} // namespace

TEST(SolveAnchor, PinsRootPositionAndRotation)
{
    Skeleton skeleton = Skeleton::makeDefault();
    IkTarget target;
    target.jointIndex = findJoint(skeleton, "Head");
    target.position = glm::vec3(0.5f, 1.2f, -0.3f);
    target.rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    solveAnchor(skeleton, target.jointIndex, target);

    const WorldTransforms wt = computeWorldTransforms(skeleton);
    EXPECT_EQ(skeleton.rootPosition, target.position);
    expectVecNear(wt.positions[target.jointIndex], target.position, 1e-6f);
    EXPECT_NEAR(glm::abs(glm::dot(wt.rotations[target.jointIndex], target.rotation)), 1.0f, 1e-5f);
}

TEST(SolveChain, RestTargetReproducesRestPose)
{
    Skeleton skeleton = makeChainSkeleton();
    const std::vector<glm::vec3> rest = computeWorldPositions(skeleton);

    IkTarget target;
    target.jointIndex = 3;
    target.position = rest[3];

    const WorldTransforms wt = computeWorldTransforms(skeleton);
    solveChain(skeleton, wt, 0, {1, 2, 3}, target);

    const std::vector<glm::vec3> solved = computeWorldPositions(skeleton);
    for (size_t i = 0; i < rest.size(); ++i)
        expectVecNear(solved[i], rest[i], 1e-3f);
}

TEST(SolveChain, ShortenedTargetCurlsChainEndOntoTarget)
{
    Skeleton skeleton = makeChainSkeleton();

    IkTarget target;
    target.jointIndex = 3;
    target.position = glm::vec3(0.0f, 0.4f, 0.0f);  // 0.6 m below root: must curl

    const WorldTransforms wt = computeWorldTransforms(skeleton);
    solveChain(skeleton, wt, 0, {1, 2, 3}, target);

    const std::vector<glm::vec3> solved = computeWorldPositions(skeleton);
    expectVecNear(solved[3], target.position, 0.01f);
    // Middle joints stay between root and target heights, bulging sideways.
    EXPECT_GT(solved[1].y, 0.4f);
    EXPECT_LT(solved[1].y, 1.0f);
    // Root does not move.
    expectVecNear(solved[0], glm::vec3(0.0f, 1.0f, 0.0f), 1e-6f);
}

TEST(SolveChain, EndBoneBlendsTowardTargetRotation)
{
    Skeleton skeleton = makeChainSkeleton();
    const std::vector<glm::vec3> rest = computeWorldPositions(skeleton);

    IkTarget target;
    target.jointIndex = 3;
    target.position = rest[3];
    target.rotation = glm::angleAxis(glm::radians(60.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    const WorldTransforms wt = computeWorldTransforms(skeleton);
    solveChain(skeleton, wt, 0, {1, 2, 3}, target);

    // The chain end takes the full target rotation (blend weight 1 at the end).
    const WorldTransforms solved = computeWorldTransforms(skeleton);
    EXPECT_NEAR(glm::abs(glm::dot(solved.rotations[3], target.rotation)), 1.0f, 1e-4f);
}

TEST(SolveChain, RotatedRootKeepsEndOnIdentityTarget)
{
    // Regression: rotating the root (e.g. the head anchor pitching) must not
    // corrupt the end bone's orientation when its own target is unrotated.
    Skeleton skeleton = makeChainSkeleton();
    const std::vector<glm::vec3> rest = computeWorldPositions(skeleton);

    skeleton.joints[0].localRot = glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

    IkTarget target;
    target.jointIndex = 3;
    target.position = rest[3];  // end target stays put in world space
    target.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    const WorldTransforms wt = computeWorldTransforms(skeleton);
    solveChain(skeleton, wt, 0, {1, 2, 3}, target);

    const WorldTransforms solved = computeWorldTransforms(skeleton);
    expectVecNear(solved.positions[3], target.position, 1e-3f);
    EXPECT_NEAR(glm::abs(glm::dot(solved.rotations[3], target.rotation)), 1.0f, 1e-3f);
}

TEST(SolveChain, RotatedRootWithRollKeepsEndOnIdentityTarget)
{
    Skeleton skeleton = makeChainSkeleton();
    const std::vector<glm::vec3> rest = computeWorldPositions(skeleton);

    skeleton.joints[0].localRot = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    IkTarget target;
    target.jointIndex = 3;
    target.position = rest[3];
    target.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    const WorldTransforms wt = computeWorldTransforms(skeleton);
    solveChain(skeleton, wt, 0, {1, 2, 3}, target);

    const WorldTransforms solved = computeWorldTransforms(skeleton);
    expectVecNear(solved.positions[3], target.position, 1e-3f);
    EXPECT_NEAR(glm::abs(glm::dot(solved.rotations[3], target.rotation)), 1.0f, 1e-3f);
}

TEST(SolveChain, SwingTargetRotationKeepsPositionExact)
{
    // Regression: a target rotation perpendicular to the chain axis (a
    // "swing", e.g. hip roll) must not displace the end bone's position.
    // The target sits above the rest pose so the tilted end bone stays
    // reachable (at full extension any end-bone tilt overreaches).
    Skeleton skeleton = makeChainSkeleton();

    IkTarget target;
    target.jointIndex = 3;
    target.position = glm::vec3(0.0f, 0.55f, 0.0f);
    target.rotation = glm::angleAxis(glm::radians(30.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    const WorldTransforms wt = computeWorldTransforms(skeleton);
    solveChain(skeleton, wt, 0, {1, 2, 3}, target);

    const WorldTransforms solved = computeWorldTransforms(skeleton);
    expectVecNear(solved.positions[3], target.position, 1e-3f);
    EXPECT_NEAR(glm::abs(glm::dot(solved.rotations[3], target.rotation)), 1.0f, 1e-3f);
}

TEST(SolveChain, TwistDistributesMonotonicallyAlongChain)
{
    Skeleton skeleton = makeChainSkeleton();
    const std::vector<glm::vec3> rest = computeWorldPositions(skeleton);

    IkTarget target;
    target.jointIndex = 3;
    target.position = rest[3];
    // Rotation about the chain's own axis (Y): pure twist.
    target.rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    const WorldTransforms wt = computeWorldTransforms(skeleton);
    solveChain(skeleton, wt, 0, {1, 2, 3}, target);

    const WorldTransforms solved = computeWorldTransforms(skeleton);
    expectVecNear(solved.positions[3], target.position, 1e-3f);
    EXPECT_NEAR(glm::abs(glm::dot(solved.rotations[3], target.rotation)), 1.0f, 1e-3f);

    // Twist angle about Y increases monotonically from root to end.
    const auto twistDeg = [](const glm::quat& q)
    {
        return glm::degrees(2.0f * std::atan2(q.y, q.w));
    };
    const float twistA = twistDeg(solved.rotations[1]);
    const float twistB = twistDeg(solved.rotations[2]);
    const float twistC = twistDeg(solved.rotations[3]);
    EXPECT_GT(twistB, twistA + 1.0f);
    EXPECT_GT(twistC, twistB + 1.0f);
    EXPECT_NEAR(twistC, 90.0f, 1.0f);
}

TEST(SolveChain, WorksOnDefaultSkeletonSpine)
{
    Skeleton skeleton = Skeleton::makeDefault();
    const std::vector<int> chain = {
        findJoint(skeleton, "Neck"), findJoint(skeleton, "Chest"),
        findJoint(skeleton, "Spine"), findJoint(skeleton, "Waist"),
        findJoint(skeleton, "Hips")};

    IkTarget target;
    target.jointIndex = chain.back();
    target.position = glm::vec3(0.0f, 1.3f, 0.0f);  // 0.4 m below head: spine curls

    const WorldTransforms wt = computeWorldTransforms(skeleton);
    solveChain(skeleton, wt, findJoint(skeleton, "Head"), chain, target);

    const std::vector<glm::vec3> solved = computeWorldPositions(skeleton);
    expectVecNear(solved[chain.back()], target.position, 0.01f);
}

TEST(SolveTwoBone, PlacesMiddleJointOnGoalBendingTowardPole)
{
    Skeleton skeleton = Skeleton::makeDefault();
    const int socket = findJoint(skeleton, "LeftHip");
    const int j1 = findJoint(skeleton, "LeftUpperLeg");
    const int j2 = findJoint(skeleton, "LeftLowerLeg");
    const int tip = findJoint(skeleton, "LeftFoot");

    IkTarget target;
    target.jointIndex = tip;
    target.position = glm::vec3(0.1f, 0.5f, -0.4f);

    const WorldTransforms rest = computeWorldTransforms(skeleton);
    solveTwoBone(skeleton, rest, socket, j1, j2, tip, target, glm::vec3(0.0f, 0.0f, -1.0f));

    const WorldTransforms wt = computeWorldTransforms(skeleton);
    const glm::vec3 goal = target.position - target.rotation * skeleton.joints[tip].restOffset;
    expectVecNear(wt.positions[j2], goal, 1e-3f);
    expectVecNear(wt.positions[tip], target.position, 1e-3f);
    // The knee bends toward the pole (-Z), not backward.
    EXPECT_LT(wt.positions[j1].z, -0.1f);
}

TEST(SolveTwoBone, OverreachStretchesTowardGoal)
{
    Skeleton skeleton = Skeleton::makeDefault();
    const int socket = findJoint(skeleton, "LeftHip");
    const int j1 = findJoint(skeleton, "LeftUpperLeg");
    const int j2 = findJoint(skeleton, "LeftLowerLeg");
    const int tip = findJoint(skeleton, "LeftFoot");

    IkTarget target;
    target.jointIndex = tip;
    target.position = glm::vec3(0.1f, 0.02f, -3.0f);  // far beyond leg length

    const WorldTransforms rest = computeWorldTransforms(skeleton);
    solveTwoBone(skeleton, rest, socket, j1, j2, tip, target, glm::vec3(0.0f, 0.0f, -1.0f));

    const WorldTransforms wt = computeWorldTransforms(skeleton);
    EXPECT_NEAR(glm::length(wt.positions[j2] - wt.positions[socket]), 0.9f, 1e-3f);
    const glm::vec3 goal = target.position - target.rotation * skeleton.joints[tip].restOffset;
    const glm::vec3 aim = glm::normalize(goal - wt.positions[socket]);
    const glm::vec3 dir = glm::normalize(wt.positions[j2] - wt.positions[socket]);
    EXPECT_NEAR(glm::dot(dir, aim), 1.0f, 1e-3f);
}

TEST(SolveTwoBone, RotatedSocketStillReachesTarget)
{
    // Regression: solveTwoBoneIk returns rest-relative world rotations; with a
    // rotated socket (e.g. a rolled hip) they must be composed on top of the
    // socket frame, or the whole limb points off by the socket rotation.
    Skeleton skeleton = Skeleton::makeDefault();
    const int hip = findJoint(skeleton, "Hips");
    skeleton.joints[hip].localRot = glm::angleAxis(glm::radians(30.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    const int socket = findJoint(skeleton, "LeftHip");
    const int j1 = findJoint(skeleton, "LeftUpperLeg");
    const int j2 = findJoint(skeleton, "LeftLowerLeg");
    const int tip = findJoint(skeleton, "LeftFoot");

    IkTarget target;
    target.jointIndex = tip;
    target.position = glm::vec3(0.1f, 0.4f, -0.2f);  // within reach of the rolled socket

    const WorldTransforms rest = computeWorldTransforms(skeleton);
    solveTwoBone(skeleton, rest, socket, j1, j2, tip, target, glm::vec3(0.0f, 0.0f, -1.0f));

    const WorldTransforms wt = computeWorldTransforms(skeleton);
    const glm::vec3 goal = target.position - target.rotation * skeleton.joints[tip].restOffset;
    expectVecNear(wt.positions[j2], goal, 1e-3f);
    expectVecNear(wt.positions[tip], target.position, 1e-3f);
    // The knee still bends toward the (socket-frame) pole, roughly forward.
    EXPECT_LT(wt.positions[j1].z, -0.05f);
}

TEST(SolveTwoBone, TipBoneTakesTargetRotation)
{
    Skeleton skeleton = Skeleton::makeDefault();
    const int socket = findJoint(skeleton, "RightShoulder");
    const int j1 = findJoint(skeleton, "RightUpperArm");
    const int j2 = findJoint(skeleton, "RightLowerArm");
    const int tip = findJoint(skeleton, "RightHand");

    IkTarget target;
    target.jointIndex = tip;
    target.position = glm::vec3(-0.5f, 1.3f, -0.3f);
    target.rotation = glm::angleAxis(glm::radians(45.0f), glm::vec3(1.0f, 0.0f, 0.0f));

    const WorldTransforms rest = computeWorldTransforms(skeleton);
    solveTwoBone(skeleton, rest, socket, j1, j2, tip, target,
                 glm::normalize(glm::vec3(0.0f, -1.0f, 1.0f)));

    const WorldTransforms wt = computeWorldTransforms(skeleton);
    EXPECT_NEAR(glm::abs(glm::dot(wt.rotations[tip], target.rotation)), 1.0f, 1e-4f);
    expectVecNear(wt.positions[tip], target.position, 1e-3f);
}
