#include "IkMath.h"

#include <algorithm>
#include <cmath>
#include <glm/gtc/constants.hpp>

namespace
{
constexpr float kEpsilon = 1e-6f;

// Component of v perpendicular to unit vector n, normalized. Falls back to a
// deterministic perpendicular when v is (near-)parallel to n.
glm::vec3 projectPerpendicular(const glm::vec3& v, const glm::vec3& n)
{
    const glm::vec3 projected = v - n * glm::dot(v, n);
    const float len = glm::length(projected);
    return (len > kEpsilon) ? projected / len : anyPerpendicular(n);
}

// Orthonormal rotation mapping rest basis (restDir, restPole) onto (aim, pole),
// where aim/pole must be unit and perpendicular. Used to give the IK chain a
// well-defined bone roll relative to the pole vector.
glm::quat basisRotation(const glm::vec3& restDir, const glm::vec3& restPole,
    const glm::vec3& aim, const glm::vec3& pole)
{
    const glm::vec3 restNormal = glm::normalize(glm::cross(restDir, restPole));
    const glm::vec3 normal = glm::normalize(glm::cross(aim, pole));
    const glm::mat3 from(restDir, restPole, restNormal);
    const glm::mat3 to(aim, pole, normal);
    return glm::quat_cast(to * glm::transpose(from));
}
} // namespace

glm::vec3 anyPerpendicular(const glm::vec3& v)
{
    const glm::vec3 other = (glm::abs(v.y) < 0.9f) ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    return glm::normalize(glm::cross(v, other));
}

glm::quat quatFromTo(const glm::vec3& from, const glm::vec3& to)
{
    const float d = glm::dot(from, to);
    if (d > 1.0f - kEpsilon)
        return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    if (d < -1.0f + kEpsilon)
        return glm::angleAxis(glm::pi<float>(), anyPerpendicular(from));
    const glm::vec3 axis = glm::cross(from, to);
    return glm::normalize(glm::quat(1.0f + d, axis.x, axis.y, axis.z));
}

TwoBoneIkResult solveTwoBoneIk(const glm::vec3& rootPos, const glm::vec3& targetPos,
    const glm::vec3& poleDir, float len1, float len2, const glm::vec3& restDir)
{
    const glm::vec3 toTarget = targetPos - rootPos;
    const float dist = glm::length(toTarget);

    // Aim direction; degenerate zero-length targets aim along the rest direction.
    const glm::vec3 aim = (dist > kEpsilon) ? toTarget / dist : glm::normalize(restDir);
    const float reach = std::clamp(dist, kEpsilon, len1 + len2);

    // Law of cosines: angle at the root between root->target and root->middle,
    // and the interior angle at the middle joint.
    const float cosRoot = std::clamp((len1 * len1 + reach * reach - len2 * len2) / (2.0f * len1 * reach), -1.0f, 1.0f);
    const float cosMid = std::clamp((len1 * len1 + len2 * len2 - reach * reach) / (2.0f * len1 * len2), -1.0f, 1.0f);
    const float rootAngle = std::acos(cosRoot);
    const float midAngle = std::acos(cosMid);

    // Bend plane from the pole vector, shared by the whole chain.
    const glm::vec3 pole = projectPerpendicular(poleDir, aim);
    const glm::vec3 normal = glm::normalize(glm::cross(aim, pole));

    // Rest basis: same pole relative to the rest direction, so the result is
    // identity when the chain is straight along restDir with a matching pole.
    const glm::vec3 restPole = projectPerpendicular(poleDir, glm::normalize(restDir));
    const glm::quat basis = basisRotation(glm::normalize(restDir), restPole, aim, pole);

    TwoBoneIkResult result;
    result.rot1 = glm::angleAxis(rootAngle, normal) * basis;
    // At the middle joint the chain turns back by the exterior angle (pi -
    // midAngle) in the opposite rotational sense: the middle bulges toward the
    // pole, the second bone comes back toward the target.
    result.rot2 = glm::angleAxis(rootAngle - (glm::pi<float>() - midAngle), normal) * basis;
    return result;
}

glm::quat clampSwingTwist(const glm::quat& localRot, const glm::vec3& twistAxis,
    float twistMinDeg, float twistMaxDeg, float swingConeDeg)
{
    const glm::vec3 axis = glm::normalize(twistAxis);

    glm::quat q = glm::normalize(localRot);
    if (q.w < 0.0f)
        q = -q;  // shortest-path representative, so twist lands in (-180, 180]

    // Twist: component of q around the axis. Signed angle about +axis.
    const float twistVecLen = glm::dot(glm::vec3(q.x, q.y, q.z), axis);
    const float twistDeg = glm::degrees(2.0f * std::atan2(twistVecLen, q.w));
    const float clampedTwistDeg = std::clamp(twistDeg, twistMinDeg, twistMaxDeg);
    const glm::quat twist = glm::angleAxis(glm::radians(clampedTwistDeg), axis);
    const glm::quat unclampedTwist = glm::angleAxis(glm::radians(twistDeg), axis);

    // Swing: remaining rotation, twist-free about the axis by construction.
    glm::quat swing = q * glm::inverse(unclampedTwist);
    if (swing.w < 0.0f)
        swing = -swing;
    const float swingDeg = glm::degrees(2.0f * std::acos(std::clamp(swing.w, -1.0f, 1.0f)));
    glm::quat clampedSwing(1.0f, 0.0f, 0.0f, 0.0f);
    if (swingDeg > kEpsilon)
    {
        const glm::vec3 swingAxis = glm::normalize(glm::vec3(swing.x, swing.y, swing.z));
        const float clampedSwingDeg = std::min(swingDeg, swingConeDeg);
        clampedSwing = glm::angleAxis(glm::radians(clampedSwingDeg), swingAxis);
    }

    return glm::normalize(clampedSwing * twist);
}

glm::quat clavicleSwing(const glm::vec3& armRest, const glm::vec3& aim,
    float elevationWeight, float reachWeight, float maxAngleDeg)
{
    const glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);

    // Full rotation armRest -> aim as a rotation vector (axis * angle).
    const glm::vec3 fullAxis = glm::cross(armRest, aim);
    const float fullAxisLen = glm::length(fullAxis);
    if (fullAxisLen < kEpsilon)
        return identity;  // aim parallel (or antiparallel) to the rest arm
    const float angle = std::acos(std::clamp(glm::dot(armRest, aim), -1.0f, 1.0f));
    const glm::vec3 rotVec = (fullAxis / fullAxisLen) * angle;

    // Component of rotVec about cross(armRest, toward) — the axis whose
    // positive rotation swings armRest toward `toward`.
    const auto component = [&rotVec, &armRest](const glm::vec3& toward, float weight,
        bool upwardOnly)
    {
        const glm::vec3 axis = glm::cross(armRest, toward);
        const float len = glm::length(axis);
        if (len < kEpsilon)
            return glm::vec3(0.0f);  // rest arm along `toward`: no such component
        const glm::vec3 unit = axis / len;
        const float amount = glm::dot(rotVec, unit) * weight;
        if (upwardOnly && amount < 0.0f)
            return glm::vec3(0.0f);
        return unit * amount;
    };

    const glm::vec3 total = component(glm::vec3(0.0f, 1.0f, 0.0f), elevationWeight, true)
        + component(glm::vec3(0.0f, 0.0f, -1.0f), reachWeight, false);
    const float mag = glm::length(total);
    if (mag < kEpsilon)
        return identity;
    const float maxRad = glm::radians(std::max(maxAngleDeg, 0.0f));
    return glm::angleAxis(std::min(mag, maxRad), total / mag);
}
