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

// How the bend normal of a two-bone limb's middle bone is derived per frame
// (WP2). The bend normal is the direction the middle joint (knee/elbow) is
// pushed off the socket->tip line.
// - Static:      the configured `pole` vector, in the socket's frame (rotates
//                rigidly with the hip/shoulder). The legacy behaviour.
// - DynamicFoot: `cross(aim, footTarget.rot * lateralAxis)` — the bend normal
//                is perpendicular to the chain aim by construction, so there is
//                no pole||aim degeneracy and no fallback. The lateral axis is
//                invariant under foot pitch (the hinge axis itself), so a
//                shin-mounted tracker's pitch cannot tilt it; foot yaw/roll
//                (crossed-leg splay) move it correctly.
// - DynamicHand: same construction from the hand target's lateral axis.
// The static `pole` is retained only as the singularity guard (hinge axis
// parallel to aim — anatomically impossible for a bent limb) and as the
// reference from which the per-joint flex sign is derived at bind time.
enum class PoleMode
{
    Static,
    DynamicFoot,
    DynamicHand,
};

// WP3 clavicle stage: how much the bone ending at a two-bone limb socket (the
// clavicle, e.g. Chest->LeftShoulder) follows the limb's goal before the limb
// itself is solved. Fractions of the socket->goal aim rotation, split into an
// elevation and a reach (protraction/retraction) component; see clavicleSwing
// in IkMath.h for the construction. Lives on the *socket* bone's JointLimits
// entry; absent means the socket stays at rest (pre-WP3 behaviour).
struct ClavicleConfig
{
    float elevationWeight = 0.4f;   // fraction of the upward elevation angle
    float reachWeight = 0.35f;      // fraction of the forward/backward reach angle
    float reachThreshold = 0.8f;    // reach engages past this fraction of limb reach
    float maxAngleDeg = 30.0f;      // clamp on the resulting clavicle rotation
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
// `poleMode` selects how the bend normal is derived per frame (Static by
// default for backward compatibility with configs written before WP2); only
// meaningful on the middle bone of a two-bone chain — ignored elsewhere.
// `clavicle` enables the WP3 clavicle stage and is only meaningful on the
// *socket* bone of a two-bone chain (the shoulder) — ignored elsewhere; absent
// (the default) leaves the socket at rest as before WP3.
struct JointLimits
{
    std::string bone;
    float twistMinDeg = -180.0f;  // twist range around the bone axis
    float twistMaxDeg = 180.0f;
    float swingConeDeg = 180.0f;  // cone half-angle for swing
    std::optional<glm::vec3> pole;
    PoleMode poleMode = PoleMode::Static;
    std::optional<ClavicleConfig> clavicle;
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
    // poles for knees/elbows, cones for hips/shoulders, clavicle stage on the
    // shoulders.
    static IkRigConfig makeDefault();

    // Semantic checks: duplicate target/limit bones, pole non-zero,
    // twistMin <= twistMax, swingCone in [0, 180], clavicle weights in [0, 1],
    // clavicle reachThreshold in [0, 1], clavicle maxAngle in [0, 180]. Throws
    // Error on the first violation found. Called by from_json; IkRig::loadConfig
    // does the skeleton-dependent checks.
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
void to_json(nlohmann::json& j, const ClavicleConfig& clavicle);
void from_json(const nlohmann::json& j, ClavicleConfig& clavicle);
