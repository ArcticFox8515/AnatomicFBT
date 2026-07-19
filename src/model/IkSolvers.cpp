#include "IkSolvers.h"

#include "IkMath.h"

#include <glm/gtc/constants.hpp>

void solveAnchor(Skeleton& skeleton, int jointIndex, const IkTarget& target)
{
    skeleton.rootPosition = target.position;
    skeleton.joints[jointIndex].localRot = target.rotation;
}

void solveChain(Skeleton& skeleton, const WorldTransforms& wt, int rootIndex,
                const std::vector<int>& chain, const IkTarget& target)
{
    const size_t count = chain.size();
    if (count == 0)
        return;

    const glm::quat rootRot = wt.rotations[rootIndex];
    const glm::vec3 rootPos = wt.positions[rootIndex];

    std::vector<float> len(count);
    float totalLen = 0.0f;
    glm::vec3 netOffset{0.0f};
    for (size_t i = 0; i < count; ++i)
    {
        len[i] = glm::length(skeleton.joints[chain[i]].restOffset);
        totalLen += len[i];
        netOffset += skeleton.joints[chain[i]].restOffset;
    }
    if (totalLen < 1e-6f)
        return;

    const glm::vec3 toTarget = target.position - rootPos;
    const float dist = glm::length(toTarget);
    if (dist < 1e-4f)
        return;

    // Work in the root frame: s0 = rest chain direction, dir = desired direction.
    const glm::quat invRoot = glm::inverse(rootRot);
    const glm::vec3 s0 = glm::normalize(netOffset);
    const glm::vec3 dir = invRoot * (toTarget / dist);

    // Curl axis (root frame): bend toward the component of dir perpendicular to
    // the chain; fall back to the target's forward, then any perpendicular.
    glm::vec3 bend = dir - s0 * glm::dot(dir, s0);
    if (glm::length(bend) < 1e-3f)
    {
        const glm::vec3 forward = invRoot * target.rotation * glm::vec3(0.0f, 0.0f, -1.0f);
        bend = forward - s0 * glm::dot(forward, s0);
        if (glm::length(bend) < 1e-3f)
            bend = anyPerpendicular(s0);
    }
    bend = glm::normalize(bend);
    const glm::vec3 axis = glm::normalize(glm::cross(s0, bend));

    // Arc model: segment i direction rotates by psi * midW[i] around the axis.
    // Chain end for curl angle psi; |endPos| decreases monotonically with psi.
    std::vector<float> midW(count), endW(count);
    float accum = 0.0f;
    for (size_t i = 0; i < count; ++i)
    {
        midW[i] = (accum + 0.5f * len[i]) / totalLen;
        accum += len[i];
        endW[i] = accum / totalLen;
    }
    const auto endPos = [&](float psi)
    {
        glm::vec3 p{0.0f};
        for (size_t i = 0; i < count; ++i)
            p += glm::angleAxis(psi * midW[i], axis) * (s0 * len[i]);
        return p;
    };

    // Solve the curl angle so the chain end lands at the target distance.
    float psi = 0.0f;
    if (dist < totalLen)
    {
        float lo = 0.0f, hi = glm::two_pi<float>();
        if (glm::length(endPos(hi)) < dist)
        {
            for (int it = 0; it < 16; ++it)
            {
                const float mid = 0.5f * (lo + hi);
                if (glm::length(endPos(mid)) > dist)
                    lo = mid;
                else
                    hi = mid;
            }
        }
        psi = 0.5f * (lo + hi);
    }

    // Swing aligns the (possibly curled) chain end with the target direction.
    const glm::quat swing = quatFromTo(glm::normalize(endPos(psi)), dir);
    const glm::quat targetRel = invRoot * target.rotation;  // target rotation in root frame
    const glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);

    glm::quat prevWorld = rootRot;
    for (size_t i = 0; i < count; ++i)
    {
        const glm::quat rootFrameRot = swing * glm::angleAxis(psi * midW[i], axis)
            * glm::slerp(identity, targetRel, endW[i]);
        const glm::quat worldRot = glm::normalize(rootRot * rootFrameRot);
        Joint& joint = skeleton.joints[chain[i]];
        joint.localRot = glm::normalize(glm::inverse(prevWorld) * worldRot);
        prevWorld = worldRot;
    }
}

void solveTwoBone(Skeleton& skeleton, const WorldTransforms& wt, int socket,
                  int j1, int j2, int tip, const IkTarget& target,
                  const glm::vec3& poleInSocketFrame)
{
    const float len1 = glm::length(skeleton.joints[j1].restOffset);
    const float len2 = glm::length(skeleton.joints[j2].restOffset);
    if (len1 < 1e-6f || len2 < 1e-6f)
        return;

    const glm::quat socketRot = wt.rotations[socket];
    const glm::vec3 restDir = socketRot * glm::normalize(skeleton.joints[j1].restOffset);
    const glm::vec3 pole = socketRot * poleInSocketFrame;
    const glm::vec3 goal = target.position - target.rotation * skeleton.joints[tip].restOffset;

    const TwoBoneIkResult result =
        solveTwoBoneIk(wt.positions[socket], goal, pole, len1, len2, restDir);

    skeleton.joints[j1].localRot = glm::normalize(glm::inverse(socketRot) * result.rot1);
    skeleton.joints[j2].localRot = glm::normalize(glm::inverse(result.rot1) * result.rot2);
    skeleton.joints[tip].localRot = glm::normalize(glm::inverse(result.rot2) * target.rotation);
}
