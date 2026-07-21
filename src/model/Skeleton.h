#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "BodyProportions.h"

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

    // Builds the default SlimeVR-style head-rooted skeleton (22 joints, fixed
    // hierarchy and bone names) scaled to the given body proportions. Bone
    // lengths equal the proportions; landmark heights hold relative to the
    // skeleton's own ankles: Chest (arm attachment) sits shoulderHeight above
    // them, Waist navelHeight, Hips upperLeg + lowerLeg. The root's rest Y
    // (shoulderHeight + neckLength) only seeds rootPosition — a rigid
    // translation of the rest pose, overwritten by calibration in VR modes.
    // The fictional mid-spine joint splits its span at the midpoint;
    // head-height (0.15, Head->Neck) and hand/foot bone lengths (0.12/0.08)
    // are internal constants — hand/foot lengths provably cancel out of the
    // capture-mode IK (the calibration offset gains exactly the term the
    // two-bone solver subtracts back).
    static Skeleton makeDefault(const BodyProportions& proportions = BodyProportions());

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
