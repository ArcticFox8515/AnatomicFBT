#include "Skeleton.h"

#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace
{
struct RawBone
{
    std::string name;
    std::string parent;
    bool hasParent = false;
    glm::vec3 offset{0.0f, 0.0f, 0.0f};
};

RawBone parseBone(const nlohmann::json& bone)
{
    RawBone raw;
    raw.name = bone.at("name").get<std::string>();

    if (bone.contains("parent") && !bone.at("parent").is_null())
    {
        raw.hasParent = true;
        raw.parent = bone.at("parent").get<std::string>();
    }

    if (bone.contains("offset"))
    {
        const nlohmann::json& arr = bone.at("offset");
        if (!arr.is_array() || arr.size() != 3)
            throw std::runtime_error("bone '" + raw.name + "': offset must be an array of 3 numbers");
        raw.offset = glm::vec3(arr[0].get<float>(), arr[1].get<float>(), arr[2].get<float>());
    }

    return raw;
}
} // namespace

void to_json(nlohmann::json& j, const Skeleton& skeleton)
{
    j = nlohmann::json{{"bones", nlohmann::json::array()}};
    for (const Joint& joint : skeleton.joints)
    {
        nlohmann::json bone;
        bone["name"] = joint.name;
        if (joint.parentIndex >= 0)
            bone["parent"] = skeleton.joints[joint.parentIndex].name;
        else
            bone["parent"] = nullptr;
        bone["offset"] = {joint.restOffset.x, joint.restOffset.y, joint.restOffset.z};
        j["bones"].push_back(std::move(bone));
    }
}

void from_json(const nlohmann::json& j, Skeleton& skeleton)
{
    const nlohmann::json& bones = j.at("bones");
    if (!bones.is_array())
        throw std::runtime_error("skeleton: 'bones' must be an array");

    std::vector<RawBone> raw;
    raw.reserve(bones.size());
    for (const nlohmann::json& bone : bones)
        raw.push_back(parseBone(bone));

    if (raw.empty())
        throw std::runtime_error("skeleton: no bones");

    std::unordered_set<std::string> names;
    for (const RawBone& bone : raw)
        if (!names.insert(bone.name).second)
            throw std::runtime_error("skeleton: duplicate bone name '" + bone.name + "'");

    int rootCount = 0;
    for (const RawBone& bone : raw)
        if (!bone.hasParent)
            ++rootCount;
    if (rootCount != 1)
        throw std::runtime_error("skeleton: expected exactly 1 root bone, got " + std::to_string(rootCount));

    for (const RawBone& bone : raw)
        if (bone.hasParent && names.find(bone.parent) == names.end())
            throw std::runtime_error("skeleton: bone '" + bone.name + "' has unknown parent '" + bone.parent + "'");

    // Sort parent-before-child.
    std::vector<Joint> joints;
    joints.reserve(raw.size());
    std::unordered_map<std::string, int> indexOf;
    std::vector<bool> placed(raw.size(), false);
    size_t remaining = raw.size();
    while (remaining > 0)
    {
        bool progress = false;
        for (size_t i = 0; i < raw.size(); ++i)
        {
            if (placed[i])
                continue;

            int parentIndex = -1;
            if (raw[i].hasParent)
            {
                const auto it = indexOf.find(raw[i].parent);
                if (it == indexOf.end())
                    continue; // parent not placed yet
                parentIndex = it->second;
            }

            Joint joint;
            joint.name = raw[i].name;
            joint.parentIndex = parentIndex;
            joint.restOffset = raw[i].offset;
            indexOf[joint.name] = static_cast<int>(joints.size());
            joints.push_back(std::move(joint));
            placed[i] = true;
            --remaining;
            progress = true;
        }
        if (!progress)
            throw std::runtime_error("skeleton: bones contain a cycle");
    }

    skeleton.joints = std::move(joints);
}

Skeleton Skeleton::makeDefault()
{
    Skeleton skeleton;
    auto add = [&skeleton](std::string name, const std::string& parent, glm::vec3 offset)
    {
        Joint joint;
        joint.name = std::move(name);
        joint.parentIndex = -1;
        if (!parent.empty())
        {
            for (size_t i = 0; i < skeleton.joints.size(); ++i)
                if (skeleton.joints[i].name == parent)
                    joint.parentIndex = static_cast<int>(i);
        }
        joint.restOffset = offset;
        skeleton.joints.push_back(std::move(joint));
    };

    // Head-rooted spine (SlimeVR-style), left side at +X.
    add("head", "", {0.0f, 1.70f, 0.0f});
    add("neck", "head", {0.0f, -0.10f, 0.0f});
    add("upper_chest", "neck", {0.0f, -0.10f, 0.0f});
    add("chest", "upper_chest", {0.0f, -0.15f, 0.0f});
    add("waist", "chest", {0.0f, -0.15f, 0.0f});
    add("hip", "waist", {0.0f, -0.20f, 0.0f});

    add("left_hip", "hip", {0.10f, 0.0f, 0.0f});
    add("left_upper_leg", "left_hip", {0.0f, -0.45f, 0.0f});
    add("left_lower_leg", "left_upper_leg", {0.0f, -0.45f, 0.0f});
    add("left_foot", "left_lower_leg", {0.0f, -0.08f, 0.0f});

    add("right_hip", "hip", {-0.10f, 0.0f, 0.0f});
    add("right_upper_leg", "right_hip", {0.0f, -0.45f, 0.0f});
    add("right_lower_leg", "right_upper_leg", {0.0f, -0.45f, 0.0f});
    add("right_foot", "right_lower_leg", {0.0f, -0.08f, 0.0f});

    add("left_shoulder", "upper_chest", {0.20f, 0.0f, 0.0f});
    add("left_upper_arm", "left_shoulder", {0.28f, 0.0f, 0.0f});
    add("left_lower_arm", "left_upper_arm", {0.26f, 0.0f, 0.0f});
    add("left_hand", "left_lower_arm", {0.18f, 0.0f, 0.0f});

    add("right_shoulder", "upper_chest", {-0.20f, 0.0f, 0.0f});
    add("right_upper_arm", "right_shoulder", {-0.28f, 0.0f, 0.0f});
    add("right_lower_arm", "right_upper_arm", {-0.26f, 0.0f, 0.0f});
    add("right_hand", "right_lower_arm", {-0.18f, 0.0f, 0.0f});

    return skeleton;
}

std::vector<glm::vec3> computeWorldPositions(const Skeleton& skeleton)
{
    std::vector<glm::vec3> positions(skeleton.joints.size());
    for (size_t i = 0; i < skeleton.joints.size(); ++i)
    {
        const Joint& joint = skeleton.joints[i];
        const glm::vec3 offset = joint.localRot * joint.restOffset;
        positions[i] = (joint.parentIndex >= 0 ? positions[joint.parentIndex] : glm::vec3(0.0f)) + offset;
    }
    return positions;
}
