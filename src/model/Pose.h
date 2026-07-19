#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>

// Rigid pose in world space (position + orientation), used for tracked devices
// and calibration math. Separate from IkTarget (which is a solver handle bound
// to a joint) and from WorldTransforms (full skeleton FK output).

struct Pose
{
    glm::vec3 position{0.0f, 0.0f, 0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
};

// Rigid composition: applying b first, then a (like matrix a * b).
inline Pose compose(const Pose& a, const Pose& b)
{
    return {a.position + a.rotation * b.position, a.rotation * b.rotation};
}

// Rigid inverse: compose(inverse(p), p) == identity.
inline Pose inverse(const Pose& p)
{
    const glm::quat invRot = glm::inverse(p.rotation);
    return {invRot * (-p.position), invRot};
}

// Rotation with pitch and roll removed, keeping only the heading (yaw around
// +Y). Identity input yields identity; a downward-facing rotation yields the
// yaw of its horizontal heading.
inline glm::quat yawOnly(const glm::quat& rotation)
{
    const glm::vec3 forward = rotation * glm::vec3(0.0f, 0.0f, -1.0f);
    if (forward.x * forward.x + forward.z * forward.z < 1e-12f)
        return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    return glm::angleAxis(std::atan2(-forward.x, -forward.z), glm::vec3(0.0f, 1.0f, 0.0f));
}
