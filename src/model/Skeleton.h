#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

struct Joint
{
    std::string name;
    std::optional<int> parentIndex = std::nullopt;
    glm::vec3 restOffset{0.0f, 0.0f, 0.0f};      // translation from parent joint, meters
    glm::quat localRot{1.0f, 0.0f, 0.0f, 0.0f};  // identity at rest; not serialized
};

// Pure data class: joints are public and kept sorted parent-before-child.
// The only logic here is JSON serialization.
class Skeleton
{
public:
    std::vector<Joint> joints;

    // World position of the root joint. Runtime-only (not serialized); initialized
    // from the root's restOffset at load time. The root's restOffset is NOT applied
    // by forward kinematics — it only seeds this value.
    glm::vec3 rootPosition{0.0f, 0.0f, 0.0f};

    // Builds the default SlimeVR-style head-rooted skeleton.
    static Skeleton makeDefault();

    // Builds the same skeleton as makeDefault() but rooted at "Hips" with the
    // spine chain reversed (Hips -> Waist -> ... -> Head), like
    // VRChat/Unity avatars. Rest world positions are identical.
    static Skeleton makeDefaultHipRooted();
};

void to_json(nlohmann::json& j, const Skeleton& skeleton);
void from_json(const nlohmann::json& j, Skeleton& skeleton);

// FK result for the whole skeleton.
struct WorldTransforms
{
    std::vector<glm::vec3> positions;
    std::vector<glm::quat> rotations;  // world orientation of the bone ending at each joint
};

// Hierarchical forward kinematics, one linear pass over the (parent-before-child
// sorted) joints. The root sits at skeleton.rootPosition with orientation
// localRot. Children: worldRot = parentWorldRot * localRot,
// pos = parentPos + worldRot * restOffset.
WorldTransforms computeWorldTransforms(const Skeleton& skeleton);

// Convenience wrapper: positions only.
std::vector<glm::vec3> computeWorldPositions(const Skeleton& skeleton);
