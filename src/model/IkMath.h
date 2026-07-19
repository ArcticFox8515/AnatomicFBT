#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// Pure math helpers for the IK solver. All inputs/outputs in world space.

// Returns a unit vector perpendicular to v (v must be unit length), deterministically.
glm::vec3 anyPerpendicular(const glm::vec3& v);

// Shortest-arc rotation taking unit vector `from` to unit vector `to`.
// Handles (near-)parallel and (near-)antiparallel inputs.
glm::quat quatFromTo(const glm::vec3& from, const glm::vec3& to);

// Closed-form two-bone IK result: world-space delta rotations from the rest
// pose for the two bones of a chain (root -> middle -> end), e.g. thigh + shin.
// Each is identity when the solved bone stays along restDir; compose with the
// bone's rest-frame orientation (rot * restFrame) to get a world orientation.
struct TwoBoneIkResult
{
    glm::quat rot1{1.0f, 0.0f, 0.0f, 0.0f};  // bone from rootPos to the middle joint
    glm::quat rot2{1.0f, 0.0f, 0.0f, 0.0f};  // bone from the middle joint to the end
};

// Positions the end joint at targetPos, or as close as possible when out of reach
// (chain stretches straight toward the target). poleDir biases the bend direction
// of the middle joint (knee/elbow) and fixes bone roll; it need not be normalized
// or perpendicular to the chain. Both bones are assumed to lie straight along
// restDir at rest with lengths len1/len2. All inputs in world space.
TwoBoneIkResult solveTwoBoneIk(const glm::vec3& rootPos, const glm::vec3& targetPos,
    const glm::vec3& poleDir, float len1, float len2, const glm::vec3& restDir);

// Clamps a parent-relative rotation to a twist range around twistAxis plus a swing
// cone half-angle, via swing-twist decomposition. Angles in degrees.
glm::quat clampSwingTwist(const glm::quat& localRot, const glm::vec3& twistAxis,
    float twistMinDeg, float twistMaxDeg, float swingConeDeg);
