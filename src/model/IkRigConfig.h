#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

// Per-joint rotation limits, degrees. Consumed by the IK solver (not yet implemented).
struct JointLimits
{
    std::string bone;
    float twistMinDeg = -180.0f;  // twist range around the bone axis
    float twistMaxDeg = 180.0f;
    float swingConeDeg = 180.0f;  // cone half-angle for swing
};

// Which joints accept tracker targets, plus per-joint limits.
// Kept separate from the skeleton config: the milestone-3 avatar skeleton has no IK.
class IkRigConfig
{
public:
    std::vector<std::string> targetBones;
    std::vector<JointLimits> limits;

    // Head/hands/feet/hip targets; hinge limits for knees/elbows, cones for hips/shoulders.
    static IkRigConfig makeDefault();
};

void to_json(nlohmann::json& j, const IkRigConfig& config);
void from_json(const nlohmann::json& j, IkRigConfig& config);
