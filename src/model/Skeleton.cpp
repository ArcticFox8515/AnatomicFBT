#include "Skeleton.h"

#include "Error.h"
#include "GlmJson.h"

#include <unordered_map>
#include <unordered_set>

namespace
{
struct RawBone
{
    std::string name;
    std::optional<std::string> parent;
    glm::vec3 offset{0.0f, 0.0f, 0.0f};
};

void from_json(const nlohmann::json& j, RawBone& bone)
{
    j.at("name").get_to(bone.name);
    if (const auto it = j.find("parent"); it != j.end() && !it->is_null())
        bone.parent = it->get<std::string>();
    bone.offset = j.value("offset", glm::vec3{0.0f});
}
} // namespace

void to_json(nlohmann::json& j, const Skeleton& skeleton)
{
    j = nlohmann::json{{"bones", nlohmann::json::array()}};
    for (const Joint& joint : skeleton.joints)
    {
        nlohmann::json bone;
        bone["name"] = joint.name;
        if (joint.parentIndex)
            bone["parent"] = skeleton.joints[*joint.parentIndex].name;
        else
            bone["parent"] = nullptr;
        bone["offset"] = joint.restOffset;
        j["bones"].push_back(std::move(bone));
    }
}

void from_json(const nlohmann::json& j, Skeleton& skeleton)
{
    const std::vector<RawBone> raw = j.at("bones").get<std::vector<RawBone>>();

    std::unordered_set<std::string> names;
    for (const RawBone& bone : raw)
        names.insert(bone.name);
    for (const RawBone& bone : raw)
        if (bone.parent && !names.contains(*bone.parent))
            throw Error("skeleton: bone '" + bone.name + "' has unknown parent '" + *bone.parent + "'");

    // Sort parent-before-child, resolving parent names to indices.
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

            std::optional<int> parentIndex = std::nullopt;
            if (raw[i].parent)
            {
                const auto it = indexOf.find(*raw[i].parent);
                if (it == indexOf.end())
                    continue; // parent not placed yet
                parentIndex = it->second;
            }

            Joint joint;
            joint.name = raw[i].name;
            joint.parentIndex = parentIndex;
            joint.restOffset = raw[i].offset;
            if (!indexOf.emplace(joint.name, static_cast<int>(joints.size())).second)
                throw Error("skeleton: duplicate bone name '" + joint.name + "'");
            joints.push_back(std::move(joint));
            placed[i] = true;
            --remaining;
            progress = true;
        }
        if (!progress)
            throw Error("skeleton: bones contain a cycle");
    }

    Skeleton result;
    result.joints = std::move(joints);
    int rootCount = 0;
    for (const Joint& joint : result.joints)
        if (!joint.parentIndex)
        {
            ++rootCount;
            result.rootPosition = joint.restOffset;
        }
    if (rootCount != 1)
        throw Error("skeleton: expected exactly 1 root bone, got " + std::to_string(rootCount));

    skeleton = std::move(result);
}

Skeleton Skeleton::makeDefault()
{
    Skeleton skeleton;
    auto add = [&skeleton](std::string name, const std::string& parent, glm::vec3 offset)
    {
        Joint joint;
        joint.name = std::move(name);
        joint.parentIndex = std::nullopt;
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
    add("left_foot", "left_lower_leg", {0.0f, 0.0f, -0.08f});

    add("right_hip", "hip", {-0.10f, 0.0f, 0.0f});
    add("right_upper_leg", "right_hip", {0.0f, -0.45f, 0.0f});
    add("right_lower_leg", "right_upper_leg", {0.0f, -0.45f, 0.0f});
    add("right_foot", "right_lower_leg", {0.0f, 0.0f, -0.08f});

    add("left_shoulder", "upper_chest", {0.20f, 0.0f, 0.0f});
    add("left_upper_arm", "left_shoulder", {0.28f, 0.0f, 0.0f});
    add("left_lower_arm", "left_upper_arm", {0.26f, 0.0f, 0.0f});
    add("left_hand", "left_lower_arm", {0.18f, 0.0f, 0.0f});

    add("right_shoulder", "upper_chest", {-0.20f, 0.0f, 0.0f});
    add("right_upper_arm", "right_shoulder", {-0.28f, 0.0f, 0.0f});
    add("right_lower_arm", "right_upper_arm", {-0.26f, 0.0f, 0.0f});
    add("right_hand", "right_lower_arm", {-0.18f, 0.0f, 0.0f});

    for (const Joint& joint : skeleton.joints)
        if (!joint.parentIndex)
            skeleton.rootPosition = joint.restOffset;

    return skeleton;
}

WorldTransforms computeWorldTransforms(const Skeleton& skeleton)
{
    WorldTransforms result;
    const size_t count = skeleton.joints.size();
    result.positions.resize(count);
    result.rotations.resize(count);
    for (size_t i = 0; i < count; ++i)
    {
        const Joint& joint = skeleton.joints[i];
        if (joint.parentIndex)
        {
            const size_t parent = static_cast<size_t>(*joint.parentIndex);
            const glm::quat worldRot = result.rotations[parent] * joint.localRot;
            result.rotations[i] = worldRot;
            result.positions[i] = result.positions[parent] + worldRot * joint.restOffset;
        }
        else
        {
            result.rotations[i] = joint.localRot;
            result.positions[i] = skeleton.rootPosition;
        }
    }
    return result;
}

std::vector<glm::vec3> computeWorldPositions(const Skeleton& skeleton)
{
    return computeWorldTransforms(skeleton).positions;
}
