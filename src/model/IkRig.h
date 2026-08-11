#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "IkRigConfig.h"
#include "IkSolvers.h"
#include "Skeleton.h"

// Skeleton + IK data (config + current target poses) + the IK solver.
class IkRig
{
public:
    Skeleton skeleton;
    IkRigConfig config;
    std::vector<IkTarget> targets;

    // Non-throwing: stores the skeleton and indexes its joints. The rig
    // starts config-less (no targets; solve() then just keeps the rest
    // pose) until loadConfig() is called.
    explicit IkRig(Skeleton s);

    // Validates the config against the skeleton, derives solver stages from
    // config + topology, and places targets at the rest pose. Throws Error
    // when a target/limit bone is missing, an anchor target is not the root
    // joint, a two_bone target has fewer than 3 ancestors, or a two_bone
    // chain's middle bone has no pole in the limits. On failure the
    // previous config/targets stay active.
    void loadConfig(IkRigConfig c);

    // Solves joint localRot values (and rootPosition) from the current target
    // poses. Stages, in config-declared solver types: anchors -> chains ->
    // clavicles -> two-bone limbs -> joint limits -> end-effector re-aim (see
    // below). Stateless: every call re-derives the full pose. Targets absent
    // from the config produce no stage. Policy: tracked rotation wins over
    // limits on end-effector bones (anchor root, chain end, two-bone tip) — the
    // final re-aim pass re-applies the goal rotation to them, so a limit bends
    // the mid-bones without silently rotating the tracked feature; limits only
    // constrain mid-bones.
    void solve();

    // Same as solve(), but consumes the given goals instead of the stored
    // targets — targets are left untouched. goals must parallel targets
    // (same size); intended for goals derived from the targets (e.g.
    // calibration offsets applied on a copy). Throws Error on size mismatch.
    void solve(const std::vector<IkTarget>& goals);

    // Resets the skeleton to the rest pose and targets to match.
    void resetTargets();

    // Bone name for a target, for UI labels.
    const std::string& targetName(size_t targetIndex) const
    {
        return skeleton.joints[targets[targetIndex].jointIndex].name;
    }

private:
    // Solver stage derived from config + skeleton topology, parallel to targets.
    struct SolverBinding
    {
        SolverType solver = SolverType::TwoBone;
        std::vector<int> chain;  // Chain: root->target path (root excluded);
                                 // TwoBone: {socket, j1, j2, tip}
        glm::vec3 pole{0.0f, 0.0f, 0.0f};  // TwoBone only, in the socket's frame
        PoleMode poleMode = PoleMode::Static;  // TwoBone only
        int sideSign = 1;   // DynamicFoot/Hand: +1 left side (+X), -1 right (-X)
        float flexSign = 1.0f;  // DynamicFoot/Hand: +1/-1, flips cross() to the
                                // anatomically correct bulge side (knee vs elbow
                                // flex in opposite directions); derived once at
                                // bind time from the static pole so no per-frame
                                // sign test can flip at the singularity.
        // WP3: set when the socket bone's limits entry carries a clavicle
        // config (TwoBone only); absent leaves the socket at rest.
        std::optional<ClavicleConfig> clavicle;
    };

    std::unordered_map<std::string, int> jointIndexOf_;
    std::vector<SolverBinding> bindings_;
    int rootIndex_ = 0;

    int findJoint(const std::string& bone) const;
};
