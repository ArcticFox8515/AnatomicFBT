#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

#include "Pose.h"
#include "Skeleton.h"

// World-space manipulation handle bound to a joint.
struct IkTarget
{
    int jointIndex = -1;
    glm::vec3 position{0.0f, 0.0f, 0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
};

// Extracts the world-space poses of a target list — the markers the renderer
// draws. Mirrors devicePosePairs (TrackedDevice.h) for the same reason: the
// renderer takes a flat vector, not an IkRig.
inline std::vector<Pose> targetPoses(const std::vector<IkTarget>& targets)
{
    std::vector<Pose> poses;
    poses.reserve(targets.size());
    for (const IkTarget& t : targets)
        poses.push_back({t.position, t.rotation});
    return poses;
}

// Per-target solver stages. All write joint localRot values (and rootPosition)
// into the skeleton; callers recompute WorldTransforms between stages.

// Rigidly pins the root joint to the target (position + rotation).
// jointIndex must be the skeleton's root joint.
void solveAnchor(Skeleton& skeleton, int jointIndex, const IkTarget& target);

// Chain IK (spine-style): the end bone rigidly takes the target rotation, and
// the remaining segments swing/curl (arc model) so the end bone's base lands
// on the implied goal. Segment orientations use the minimal swing from the
// root frame onto their solved directions, plus the leftover twist about the
// chain distributed by length (so e.g. head yaw rolls down the spine
// gradually). End position and rotation are exact when the goal is within the
// segment chain's reach; otherwise the chain stretches straight toward the
// goal and the end rotation still wins.
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

// Clavicle stage (WP3): rotates the bone *ending at* a two-bone limb's socket
// (the clavicle, e.g. Chest->LeftShoulder) partway toward the limb's goal, so
// the shoulder elevates with a raised hand and protracts on a long reach
// instead of staying rigid. chain is the two-bone {socket, j1, j2, tip}.
//
// The rotation is a weighted fraction of the socket->goal aim rotation
// (clavicleSwing, IkMath.h): the elevation component scaled by
// elevationWeight, the reach component scaled by reachWeight times a gate that
// ramps from 0 to 1 as the goal distance goes from reachThreshold to 1.0 of the
// limb's reach (|j1| + |j2|), the sum clamped to maxAngleDeg.
//
// One-shot and stateless: the aim is measured from the socket's pre-rotation
// position (the displacement the clavicle itself causes is not iterated), and
// the socket joint is expected to still be at rest — call once per solve,
// before the limb stage, which then solves from the moved socket. Does nothing
// when the socket is the skeleton root (no clavicle bone exists), a limb bone
// has zero length, or the goal coincides with the socket.
void solveClavicle(Skeleton& skeleton, const WorldTransforms& wt,
                   const std::vector<int>& chain, const IkTarget& target,
                   float elevationWeight, float reachWeight, float reachThreshold,
                   float maxAngleDeg);
