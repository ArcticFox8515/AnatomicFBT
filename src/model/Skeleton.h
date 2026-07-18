#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

struct Joint
{
    std::string name;
    int parentIndex = -1;                        // -1 = root
    glm::vec3 restOffset{0.0f, 0.0f, 0.0f};      // translation from parent joint, meters
    glm::quat localRot{1.0f, 0.0f, 0.0f, 0.0f};  // identity at rest; not serialized
};

// Pure data class: joints are public and kept sorted parent-before-child.
// The only logic here is JSON serialization.
class Skeleton
{
public:
    std::vector<Joint> joints;

    // Builds the default SlimeVR-style head-rooted skeleton.
    static Skeleton makeDefault();
};

void to_json(nlohmann::json& j, const Skeleton& skeleton);
void from_json(const nlohmann::json& j, Skeleton& skeleton);

// One linear pass over the (parent-before-child sorted) joints.
std::vector<glm::vec3> computeWorldPositions(const Skeleton& skeleton);
