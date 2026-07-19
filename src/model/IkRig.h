#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <unordered_map>
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

// Skeleton + IK data (config + current target poses) + the IK solver.
class IkRig
{
public:
    Skeleton skeleton;
    IkRigConfig config;
    std::vector<IkTarget> targets;

    // Validates that every target/limit bone exists in the skeleton and
    // initializes targets at the rest pose. Throws std::runtime_error otherwise.
    IkRig(Skeleton s, IkRigConfig c);

    // Solves joint localRot values (and rootPosition) from the current target
    // poses: head pin -> spine interpolation -> two-bone IK for legs/arms ->
    // joint limits. Stateless: every call re-derives the full pose. Stages whose
    // target or bones are missing from the skeleton/config are skipped.
    void solve();

    // Resets the skeleton to the rest pose and targets to match.
    void resetTargets();

    // Bone name for a target, for UI labels.
    const std::string& targetName(size_t targetIndex) const
    {
        return skeleton.joints[targets[targetIndex].jointIndex].name;
    }

private:
    std::unordered_map<std::string, int> jointIndexOf_;
    int rootIndex_ = 0;

    const IkTarget* findTarget(const std::string& bone) const;
    int findJoint(const std::string& bone) const;
};
