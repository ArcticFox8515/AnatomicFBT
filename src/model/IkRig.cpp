#include "IkRig.h"

#include <stdexcept>
#include <unordered_map>

IkRig::IkRig(Skeleton s, IkRigConfig c)
    : skeleton(std::move(s)), config(std::move(c))
{
    std::unordered_map<std::string, int> indexOf;
    for (size_t i = 0; i < skeleton.joints.size(); ++i)
        indexOf[skeleton.joints[i].name] = static_cast<int>(i);

    for (const std::string& bone : config.targetBones)
    {
        const auto it = indexOf.find(bone);
        if (it == indexOf.end())
            throw std::runtime_error("ikrig: target bone '" + bone + "' not in skeleton");
        IkTarget target;
        target.jointIndex = it->second;
        targets.push_back(target);
    }

    for (const JointLimits& limit : config.limits)
        if (!indexOf.contains(limit.bone))
            throw std::runtime_error("ikrig: limits bone '" + limit.bone + "' not in skeleton");

    resetTargets();
}

void IkRig::resetTargets()
{
    const std::vector<glm::vec3> positions = computeWorldPositions(skeleton);
    for (IkTarget& target : targets)
    {
        target.position = positions[target.jointIndex];
        target.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    }
}
