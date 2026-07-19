#pragma once

#include <glm/glm.hpp>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

// How a target's IK stage solves the skeleton toward it.
enum class SolverType
{
    Anchor,   // rigidly pin the root joint to the target
    Chain,    // interpolate the root->joint chain (spine-style)
    TwoBone,  // closed-form two-bone limb IK, tip bone takes target rotation
};

// A joint that accepts a tracker target, plus the solver stage it drives.
struct TargetConfig
{
    std::string bone;
    SolverType solver = SolverType::TwoBone;
};

// Per-joint rotation limits (degrees) and optional bend preference, consumed by
// the IK solver. `pole` is the bend direction of this bone's hinge, expressed
// in the limb socket's frame; required on the middle bone of two-bone chains.
struct JointLimits
{
    std::string bone;
    float twistMinDeg = -180.0f;  // twist range around the bone axis
    float twistMaxDeg = 180.0f;
    float swingConeDeg = 180.0f;  // cone half-angle for swing
    std::optional<glm::vec3> pole;
};

// Which joints accept tracker targets (and how they are solved), plus per-joint
// limits. Kept separate from the skeleton config: the milestone-3 avatar
// skeleton has no IK.
class IkRigConfig
{
public:
    std::vector<TargetConfig> targets;
    std::vector<JointLimits> limits;

    // Anchor on head, chain on hip, two-bone on hands/feet; hinge limits with
    // poles for knees/elbows, cones for hips/shoulders.
    static IkRigConfig makeDefault();
};

void to_json(nlohmann::json& j, const IkRigConfig& config);
void from_json(const nlohmann::json& j, IkRigConfig& config);
