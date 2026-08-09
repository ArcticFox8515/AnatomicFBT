#include "VirtualTrackers.h"

#include "IkRig.h"
#include "Skeleton.h"
#include "TrackerCalibration.h"

#include <unordered_map>
#include <unordered_set>

#include <algorithm>

std::vector<std::string> eligibleVirtualTrackerBones(
    const IkRig& rig, const Skeleton& avatar, const TrackerCalibration& /*calibration*/)
{
    // Joints that are IK targets — "mapped to trackers" (Head is the HMD
    // anchor, the rest are tracker targets). Excluded whether or not a
    // device is currently bound, so the list is stable across calibration.
    std::unordered_set<int> targetJoints;
    targetJoints.reserve(rig.targets.size());
    for (const IkTarget& target : rig.targets)
        targetJoints.insert(target.jointIndex);

    // Avatar joint names — only presence matters (kind-3 exclusion both ways).
    std::unordered_set<std::string> avatarNames;
    avatarNames.reserve(avatar.joints.size());
    for (const Joint& joint : avatar.joints)
        avatarNames.insert(joint.name);

    std::vector<std::string> result;
    for (size_t i = 0; i < rig.skeleton.joints.size(); ++i)
    {
        if (targetJoints.count(static_cast<int>(i)))
            continue;
        const std::string& name = rig.skeleton.joints[i].name;
        if (avatarNames.count(name))
            result.push_back(name);
    }
    return result;
}

std::vector<VirtualTrackerPose> computeVirtualTrackerPoses(
    const Skeleton& avatar, const std::vector<std::string>& boneNames)
{
    // Index avatar joints by name once.
    std::unordered_map<std::string, int> avatarIndexOf;
    avatarIndexOf.reserve(avatar.joints.size());
    for (size_t i = 0; i < avatar.joints.size(); ++i)
        avatarIndexOf.emplace(avatar.joints[i].name, static_cast<int>(i));

    const WorldTransforms wt = computeWorldTransforms(avatar);

    std::vector<VirtualTrackerPose> result;
    result.reserve(boneNames.size());
    for (const std::string& name : boneNames)
    {
        const auto it = avatarIndexOf.find(name);
        if (it == avatarIndexOf.end())
            continue;
        const int joint = it->second;
        const Joint& j = avatar.joints[static_cast<size_t>(joint)];
        if (!j.parentIndex)
            continue; // root has no parent endpoint -> no bone midpoint
        const int parent = *j.parentIndex;
        const glm::vec3 midpoint =
            (wt.positions[static_cast<size_t>(parent)] + wt.positions[static_cast<size_t>(joint)]) * 0.5f;
        result.push_back({name, {midpoint, wt.rotations[static_cast<size_t>(joint)]}});
    }
    return result;
}

VirtualTrackerRenderer::VirtualTrackerRenderer(
    const IkRig& rig, const Skeleton& avatar, const TrackerCalibration& calibration,
    const std::vector<std::string>& selectedBones)
    : rig_(rig), avatar_(avatar), calibration_(calibration), selectedBones_(selectedBones) {}

std::vector<Pose> VirtualTrackerRenderer::poses() const
{
    const std::vector<std::string> eligible =
        eligibleVirtualTrackerBones(rig_, avatar_, calibration_);
    std::vector<std::string> tickedEligible;
    for (const std::string& bone : selectedBones_)
        if (std::find(eligible.begin(), eligible.end(), bone) != eligible.end())
            tickedEligible.push_back(bone);
    const std::vector<VirtualTrackerPose> vt =
        computeVirtualTrackerPoses(avatar_, tickedEligible);
    std::vector<Pose> result;
    result.reserve(vt.size());
    for (const VirtualTrackerPose& p : vt)
        result.push_back(p.pose);
    return result;
}
