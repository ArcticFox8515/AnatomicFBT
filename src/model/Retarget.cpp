#include "Retarget.h"

#include "Skeleton.h"

#include <algorithm>
#include <glm/gtc/quaternion.hpp>
#include <map>
#include <unordered_map>
#include <utility>

RetargetMap buildRetargetMap(const Skeleton& src, const Skeleton& dst)
{
    std::unordered_map<std::string, int> srcIndexOf;
    for (size_t i = 0; i < src.joints.size(); ++i)
        srcIndexOf.emplace(src.joints[i].name, static_cast<int>(i));

    // Src bones keyed by the unordered pair of joint names they connect, so a
    // dst bone matches no matter which direction the src chain runs.
    std::map<std::pair<std::string, std::string>, int> srcBoneByPair;
    for (size_t i = 0; i < src.joints.size(); ++i)
    {
        const Joint& joint = src.joints[i];
        if (!joint.parentIndex)
            continue;
        const std::string& parentName = src.joints[static_cast<size_t>(*joint.parentIndex)].name;
        srcBoneByPair[std::minmax(parentName, joint.name)] = static_cast<int>(i);
    }

    RetargetMap map;
    map.dstToSrc.resize(dst.joints.size());
    for (size_t i = 0; i < dst.joints.size(); ++i)
    {
        const Joint& joint = dst.joints[i];
        if (!joint.parentIndex)
        {
            // The root has no bone; match it by plain name (e.g. the pelvis
            // orientation transfers onto the avatar's "hip" root joint).
            if (const auto it = srcIndexOf.find(joint.name); it != srcIndexOf.end())
                map.dstToSrc[i] = it->second;
            continue;
        }
        const std::string& parentName = dst.joints[static_cast<size_t>(*joint.parentIndex)].name;
        if (const auto it = srcBoneByPair.find(std::minmax(parentName, joint.name));
            it != srcBoneByPair.end())
            map.dstToSrc[i] = it->second;
    }

    // Anchor on the joint sharing the src root's name (e.g. "head"); fall back
    // to the dst root's name match.
    for (size_t i = 0; i < src.joints.size(); ++i)
        if (!src.joints[i].parentIndex)
        {
            if (const auto it = std::find_if(dst.joints.begin(), dst.joints.end(),
                    [&](const Joint& j) { return j.name == src.joints[i].name; });
                it != dst.joints.end())
            {
                map.anchorDst = static_cast<int>(std::distance(dst.joints.begin(), it));
                map.anchorSrc = static_cast<int>(i);
            }
            break;
        }
    if (!map.anchorDst)
        for (size_t i = 0; i < dst.joints.size(); ++i)
            if (!dst.joints[i].parentIndex && map.dstToSrc[i])
            {
                map.anchorDst = static_cast<int>(i);
                map.anchorSrc = map.dstToSrc[i];
                break;
            }

    return map;
}

void retargetPose(const Skeleton& src, Skeleton& dst, const RetargetMap& map)
{
    const WorldTransforms srcWorld = computeWorldTransforms(src);

    // dst joints are sorted parent-before-child, so parent world rotations are
    // always final when consumed here.
    constexpr glm::quat kIdentity(1.0f, 0.0f, 0.0f, 0.0f);
    std::vector<glm::quat> dstWorld(dst.joints.size());
    for (size_t i = 0; i < dst.joints.size(); ++i)
    {
        Joint& joint = dst.joints[i];
        const glm::quat parentWorld =
            joint.parentIndex ? dstWorld[static_cast<size_t>(*joint.parentIndex)] : kIdentity;
        if (map.dstToSrc[i])
        {
            dstWorld[i] = srcWorld.rotations[static_cast<size_t>(*map.dstToSrc[i])];
            joint.localRot = glm::inverse(parentWorld) * dstWorld[i];
        }
        else
        {
            dstWorld[i] = parentWorld * joint.localRot;
        }
    }

    if (map.anchorDst && map.anchorSrc)
    {
        const WorldTransforms dstNow = computeWorldTransforms(dst);
        dst.rootPosition += srcWorld.positions[static_cast<size_t>(*map.anchorSrc)]
            - dstNow.positions[static_cast<size_t>(*map.anchorDst)];
    }
}

std::vector<std::string> unmatchedBones(const Skeleton& dst, const RetargetMap& map)
{
    std::vector<std::string> names;
    for (size_t i = 0; i < dst.joints.size(); ++i)
        if (!map.dstToSrc[i])
            names.push_back(dst.joints[i].name);
    return names;
}
