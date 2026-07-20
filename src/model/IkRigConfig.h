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
    Chain,    // swing/curl the root->joint chain (spine-style), end bone takes target rotation
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

    // Anchor on Head, chain on Hips, two-bone on hands/feet; hinge limits with
    // poles for knees/elbows, cones for hips/shoulders.
    static IkRigConfig makeDefault();

    // Semantic checks: duplicate target/limit bones, pole non-zero,
    // twistMin <= twistMax, swingCone in [0, 180]. Throws Error on the first
    // violation found. Called by from_json; IkRig::loadConfig does the
    // skeleton-dependent checks.
    void validate() const;
};

void to_json(nlohmann::json& j, const IkRigConfig& config);

// JSON -> config, declaratively (missing sections -> empty vectors), then
// config.validate(). nlohmann exceptions on malformed JSON shapes.
void from_json(const nlohmann::json& j, IkRigConfig& config);

void to_json(nlohmann::json& j, const TargetConfig& target);
void from_json(const nlohmann::json& j, TargetConfig& target);
void to_json(nlohmann::json& j, const JointLimits& limit);
void from_json(const nlohmann::json& j, JointLimits& limit);
