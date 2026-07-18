#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <vector>

#include "IkRigConfig.h"
#include "Skeleton.h"

// World-space manipulation handle bound to a joint.
struct IkTarget
{
    int jointIndex = -1;
    glm::vec3 position{0.0f, 0.0f, 0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
};

// Skeleton + IK data (config + current target poses). Pure data; the IK solver
// is a separate class that reads targets and writes joint localRot values.
class IkRig
{
public:
    Skeleton skeleton;
    IkRigConfig config;
    std::vector<IkTarget> targets;

    // Validates that every target/limit bone exists in the skeleton and
    // initializes targets at the rest pose. Throws std::runtime_error otherwise.
    IkRig(Skeleton s, IkRigConfig c);

    // Returns target joint positions/rotations to the rest pose.
    void resetTargets();

    // Bone name for a target, for UI labels.
    const std::string& targetName(size_t targetIndex) const
    {
        return skeleton.joints[targets[targetIndex].jointIndex].name;
    }
};
