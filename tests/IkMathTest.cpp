#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include "model/IkMath.h"

namespace
{
void expectQuatNear(const glm::quat& actual, const glm::quat& expected, float tol = 1e-4f)
{
    EXPECT_NEAR(glm::abs(glm::dot(actual, expected)), 1.0f, tol);
}

void expectVecNear(const glm::vec3& actual, const glm::vec3& expected, float tol = 1e-4f)
{
    EXPECT_NEAR(actual.x, expected.x, tol);
    EXPECT_NEAR(actual.y, expected.y, tol);
    EXPECT_NEAR(actual.z, expected.z, tol);
}

// End position of the solved chain (root -> middle -> end) for straight-at-rest
// bones pointing along restDir.
glm::vec3 chainEnd(const TwoBoneIkResult& result, const glm::vec3& rootPos,
    const glm::vec3& restDir, float len1, float len2)
{
    const glm::vec3 mid = rootPos + result.rot1 * restDir * len1;
    return mid + result.rot2 * restDir * len2;
}
} // namespace

TEST(QuatFromTo, RotatesFromTo)
{
    const glm::vec3 from = glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f));
    const glm::vec3 to = glm::normalize(glm::vec3(-2.0f, 1.0f, 0.5f));
    const glm::quat q = quatFromTo(from, to);
    expectVecNear(q * from, to, 1e-5f);
}

TEST(QuatFromTo, HandlesAntiparallel)
{
    const glm::vec3 from(0.0f, -1.0f, 0.0f);
    const glm::vec3 to(0.0f, 1.0f, 0.0f);
    const glm::quat q = quatFromTo(from, to);
    expectVecNear(q * from, to, 1e-5f);
}

TEST(SolveTwoBoneIk, ReachesTargetAndBendsTowardPole)
{
    const glm::vec3 root(0.0f, 1.0f, 0.0f);
    const glm::vec3 restDir(0.0f, -1.0f, 0.0f);
    const glm::vec3 target(0.0f, 0.4f, -0.3f);

    const TwoBoneIkResult result = solveTwoBoneIk(root, target, glm::vec3(0.0f, 0.0f, -1.0f), 0.5f, 0.4f, restDir);

    expectVecNear(chainEnd(result, root, restDir, 0.5f, 0.4f), target);
    // Middle joint must be on the pole side (-Z) of the root->target line.
    const glm::vec3 mid = root + result.rot1 * restDir * 0.5f;
    EXPECT_LT(mid.z, root.z);
}

TEST(SolveTwoBoneIk, OverreachStretchesStraightTowardTarget)
{
    const glm::vec3 root(0.0f, 0.0f, 0.0f);
    const glm::vec3 restDir(0.0f, -1.0f, 0.0f);
    const glm::vec3 target(0.0f, -2.0f, -1.0f);

    const TwoBoneIkResult result = solveTwoBoneIk(root, target, glm::vec3(0.0f, 0.0f, -1.0f), 0.45f, 0.45f, restDir);

    const glm::vec3 aim = glm::normalize(target - root);
    expectVecNear(chainEnd(result, root, restDir, 0.45f, 0.45f), aim * 0.9f, 1e-3f);
    // Both bones point straight at the target: no bend.
    expectVecNear(result.rot1 * restDir, aim, 1e-3f);
    expectVecNear(result.rot2 * restDir, aim, 1e-3f);
}

TEST(SolveTwoBoneIk, StraightChainAtRestIsIdentity)
{
    const glm::vec3 root(0.0f, 1.0f, 0.0f);
    const glm::vec3 restDir(0.0f, -1.0f, 0.0f);
    const glm::vec3 target = root + restDir * 0.9f;  // exactly at full reach

    const TwoBoneIkResult result = solveTwoBoneIk(root, target, glm::vec3(0.0f, 0.0f, -1.0f), 0.45f, 0.45f, restDir);

    expectQuatNear(result.rot1, glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 1e-3f);
    expectQuatNear(result.rot2, glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 1e-3f);
}

TEST(SolveTwoBoneIk, PoleSelectsBendSide)
{
    const glm::vec3 root(0.0f, 1.0f, 0.0f);
    const glm::vec3 restDir(0.0f, -1.0f, 0.0f);
    const glm::vec3 target(0.0f, 0.4f, 0.0f);  // straight down: bend direction fully pole-driven

    const TwoBoneIkResult forward = solveTwoBoneIk(root, target, glm::vec3(0.0f, 0.0f, -1.0f), 0.45f, 0.45f, restDir);
    const TwoBoneIkResult backward = solveTwoBoneIk(root, target, glm::vec3(0.0f, 0.0f, 1.0f), 0.45f, 0.45f, restDir);

    const glm::vec3 midF = root + forward.rot1 * restDir * 0.45f;
    const glm::vec3 midB = root + backward.rot1 * restDir * 0.45f;
    EXPECT_LT(midF.z, -0.05f);
    EXPECT_GT(midB.z, 0.05f);
}

TEST(SolveTwoBoneIk, MirroredChainReachesTarget)
{
    const glm::vec3 root(0.2f, 1.5f, 0.0f);
    const glm::vec3 restDir(1.0f, 0.0f, 0.0f);  // arm along +X
    const glm::vec3 pole = glm::normalize(glm::vec3(0.0f, -1.0f, 1.0f));
    const glm::vec3 target(0.55f, 1.35f, 0.2f);

    const TwoBoneIkResult result = solveTwoBoneIk(root, target, pole, 0.28f, 0.26f, restDir);

    expectVecNear(chainEnd(result, root, restDir, 0.28f, 0.26f), target);
    // Elbow drops below the shoulder line and goes back.
    const glm::vec3 mid = root + result.rot1 * restDir * 0.28f;
    EXPECT_LT(mid.y, root.y);
    EXPECT_GT(mid.z, root.z);
}

TEST(SolveTwoBoneIk, DegenerateTargetAtRootStaysFinite)
{
    const glm::vec3 root(0.0f, 1.0f, 0.0f);
    const glm::vec3 restDir(0.0f, -1.0f, 0.0f);

    const TwoBoneIkResult result = solveTwoBoneIk(root, root, glm::vec3(0.0f, 0.0f, -1.0f), 0.45f, 0.45f, restDir);

    EXPECT_NEAR(glm::length(result.rot1), 1.0f, 1e-4f);
    EXPECT_NEAR(glm::length(result.rot2), 1.0f, 1e-4f);
    const glm::vec3 end = chainEnd(result, root, restDir, 0.45f, 0.45f);
    EXPECT_TRUE(std::isfinite(end.x) && std::isfinite(end.y) && std::isfinite(end.z));
}

TEST(ClampSwingTwist, IdentityPassesThrough)
{
    const glm::quat clamped = clampSwingTwist(glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, -1.0f, 0.0f), -5.0f, 5.0f, 30.0f);
    expectQuatNear(clamped, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
}

TEST(ClampSwingTwist, ClampsTwistToRange)
{
    const glm::quat q = glm::angleAxis(glm::radians(30.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::quat clamped = clampSwingTwist(q, glm::vec3(0.0f, 1.0f, 0.0f), -10.0f, 10.0f, 180.0f);
    expectQuatNear(clamped, glm::angleAxis(glm::radians(10.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
}

TEST(ClampSwingTwist, ClampsSwingToCone)
{
    const glm::quat q = glm::angleAxis(glm::radians(60.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    const glm::quat clamped = clampSwingTwist(q, glm::vec3(0.0f, 1.0f, 0.0f), -180.0f, 180.0f, 30.0f);
    expectQuatNear(clamped, glm::angleAxis(glm::radians(30.0f), glm::vec3(1.0f, 0.0f, 0.0f)), 1e-3f);
}

TEST(ClampSwingTwist, ClampsTwistAndSwingIndependently)
{
    const glm::quat swing = glm::angleAxis(glm::radians(45.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    const glm::quat twist = glm::angleAxis(glm::radians(30.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::quat clamped = clampSwingTwist(swing * twist, glm::vec3(0.0f, 1.0f, 0.0f), -10.0f, 10.0f, 20.0f);

    const glm::quat expected = glm::angleAxis(glm::radians(20.0f), glm::vec3(1.0f, 0.0f, 0.0f))
        * glm::angleAxis(glm::radians(10.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    expectQuatNear(clamped, expected, 1e-3f);
}

// ---------------------------------------------------------------------------
// WP3 — clavicle swing. armRest is the T-pose arm (+X for the left side);
// elevation is the rotation toward +Y (upward only), reach the rotation toward
// -Z (both ways).
// ---------------------------------------------------------------------------

namespace
{
const glm::quat kIdentity(1.0f, 0.0f, 0.0f, 0.0f);
const glm::vec3 kLeftArmRest(1.0f, 0.0f, 0.0f);
} // namespace

TEST(ClavicleSwing, AimAlongRestArmIsIdentity)
{
    expectQuatNear(clavicleSwing(kLeftArmRest, kLeftArmRest, 1.0f, 1.0f, 90.0f), kIdentity);
}

TEST(ClavicleSwing, RaisedAimTakesAFractionOfTheElevation)
{
    // Aim 90 deg above the rest arm, half weight: 45 deg about +Z (X toward Y).
    const glm::quat swing = clavicleSwing(kLeftArmRest, glm::vec3(0.0f, 1.0f, 0.0f),
        0.5f, 0.0f, 180.0f);
    expectQuatNear(swing, glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f)), 1e-3f);
}

TEST(ClavicleSwing, LoweredAimIsIdentity)
{
    // The clavicle has (almost) no depression range: a hanging arm must not
    // drop the shoulder, even at full elevation weight.
    expectQuatNear(clavicleSwing(kLeftArmRest, glm::vec3(0.0f, -1.0f, 0.0f),
        1.0f, 0.0f, 180.0f), kIdentity);
}

TEST(ClavicleSwing, ForwardAimProtractsAndBackwardAimRetracts)
{
    const glm::quat forward = clavicleSwing(kLeftArmRest, glm::vec3(0.0f, 0.0f, -1.0f),
        0.0f, 0.5f, 180.0f);
    const glm::quat backward = clavicleSwing(kLeftArmRest, glm::vec3(0.0f, 0.0f, 1.0f),
        0.0f, 0.5f, 180.0f);
    // Forward = +Y (X toward -Z), backward the opposite sense.
    expectVecNear(forward * kLeftArmRest,
        glm::vec3(glm::cos(glm::radians(45.0f)), 0.0f, -glm::sin(glm::radians(45.0f))), 1e-3f);
    expectVecNear(backward * kLeftArmRest,
        glm::vec3(glm::cos(glm::radians(45.0f)), 0.0f, glm::sin(glm::radians(45.0f))), 1e-3f);
}

TEST(ClavicleSwing, MirroredArmMirrorsTheSwing)
{
    // The axes are cross(armRest, target axis), so the right arm (-X) elevates
    // by the mirrored rotation — same world displacement, opposite axis sign.
    const glm::vec3 rightArmRest(-1.0f, 0.0f, 0.0f);
    const glm::quat left = clavicleSwing(kLeftArmRest, glm::normalize(glm::vec3(1.0f, 1.0f, 0.0f)),
        0.5f, 0.0f, 180.0f);
    const glm::quat right = clavicleSwing(rightArmRest, glm::normalize(glm::vec3(-1.0f, 1.0f, 0.0f)),
        0.5f, 0.0f, 180.0f);
    const glm::vec3 leftEnd = left * kLeftArmRest;
    const glm::vec3 rightEnd = right * rightArmRest;
    EXPECT_GT(leftEnd.y, 0.1f);
    expectVecNear(rightEnd, glm::vec3(-leftEnd.x, leftEnd.y, leftEnd.z), 1e-4f);
}

TEST(ClavicleSwing, ClampsToMaxAngle)
{
    const glm::quat swing = clavicleSwing(kLeftArmRest,
        glm::normalize(glm::vec3(0.0f, 1.0f, -1.0f)), 1.0f, 1.0f, 10.0f);
    const float angleDeg = glm::degrees(glm::angle(glm::normalize(swing)));
    EXPECT_NEAR(angleDeg, 10.0f, 1e-2f);
}

TEST(ClavicleSwing, ZeroWeightsAreIdentity)
{
    expectQuatNear(clavicleSwing(kLeftArmRest, glm::normalize(glm::vec3(0.0f, 1.0f, -1.0f)),
        0.0f, 0.0f, 180.0f), kIdentity);
}

TEST(ClavicleSwing, AntiparallelAimIsIdentity)
{
    // Degenerate cross product: no follow direction is defined, so the clavicle
    // holds still instead of flipping.
    expectQuatNear(clavicleSwing(kLeftArmRest, -kLeftArmRest, 1.0f, 1.0f, 180.0f), kIdentity);
}
