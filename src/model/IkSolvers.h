#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

#include "Skeleton.h"

// World-space manipulation handle bound to a joint.
struct IkTarget
{
    int jointIndex = -1;
    glm::vec3 position{0.0f, 0.0f, 0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
};

// Per-target solver stages. All write joint localRot values (and rootPosition)
// into the skeleton; callers recompute WorldTransforms between stages.

// Rigidly pins the root joint to the target (position + rotation).
// jointIndex must be the skeleton's root joint.
void solveAnchor(Skeleton& skeleton, int jointIndex, const IkTarget& target);

// Chain interpolation (SlimeVR-style): swings/curls the chain hanging off
// rootIndex so its end joint lands on the target, blending bone orientations
// from identity (world) toward the target rotation along the chain. Exact in
// position when the target is within chain length; otherwise the end bone
// orientation wins over exact position (documented interpolation trade-off).
// chain lists joint indices root-side first; each must be the parent of the next.
void solveChain(Skeleton& skeleton, const WorldTransforms& wt, int rootIndex,
                const std::vector<int>& chain, const IkTarget& target);

// Two-bone analytic IK for one limb: chain socket -> j1 -> j2 places j2 on the
// goal implied by the tip joint's target; the tip bone then takes the target
// rotation. The socket joint itself is left untouched (no clavicle/hip-socket
// solving). Pole is expressed in the socket's frame.
void solveTwoBone(Skeleton& skeleton, const WorldTransforms& wt, int socket,
                  int j1, int j2, int tip, const IkTarget& target,
                  const glm::vec3& poleInSocketFrame);
