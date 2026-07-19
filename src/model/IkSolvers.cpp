#include "IkSolvers.h"

#include "IkMath.h"

#include <cmath>
#include <glm/gtc/constants.hpp>

void solveAnchor(Skeleton& skeleton, int jointIndex, const IkTarget& target)
{
    skeleton.rootPosition = target.position;
    skeleton.joints[jointIndex].localRot = target.rotation;
}

void solveChain(Skeleton& skeleton, const WorldTransforms& wt, int rootIndex,
                const std::vector<int>& chain, const IkTarget& target)
{
    if (chain.empty())
        return;

    // The end bone is rigidly attached to the target: it takes the target
    // rotation exactly, and the remaining segments aim for the implied goal
    // (the end bone's base), mirroring solveTwoBone's implied-goal pattern.
    const int endJoint = chain.back();
    const glm::vec3 endOffset = skeleton.joints[endJoint].restOffset;
    const float endLen = glm::length(endOffset);
    const glm::vec3 goal = target.position - target.rotation * endOffset;

    const glm::quat rootRot = wt.rotations[rootIndex];
    const glm::vec3 rootPos = wt.positions[rootIndex];

    const size_t count = chain.size() - 1;  // segments between root and end bone
    if (count == 0)
    {
        skeleton.joints[endJoint].localRot =
            glm::normalize(glm::inverse(rootRot) * target.rotation);
        return;
    }

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

    const glm::vec3 toGoal = goal - rootPos;
    const float dist = glm::length(toGoal);
    if (dist < 1e-4f)
        return;

    // Work in the root frame: s0 = rest chain direction, dir = desired direction.
    const glm::quat invRoot = glm::inverse(rootRot);
    const glm::vec3 s0 = glm::normalize(netOffset);
    const glm::vec3 dir = invRoot * (toGoal / dist);

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
    std::vector<float> midW(count);
    float accum = 0.0f;
    for (size_t i = 0; i < count; ++i)
    {
        midW[i] = (accum + 0.5f * len[i]) / totalLen;
        accum += len[i];
    }
    const auto endPos = [&](float psi)
    {
        glm::vec3 p{0.0f};
        for (size_t i = 0; i < count; ++i)
            p += glm::angleAxis(psi * midW[i], axis) * (s0 * len[i]);
        return p;
    };

    // Solve the curl angle so the segment chain ends at the goal distance.
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

    // Swing aligns the (possibly curled) segment chain with the goal direction.
    const glm::quat swing = quatFromTo(glm::normalize(endPos(psi)), dir);

    // Twist left over once the end bone's minimal swing from the root frame is
    // accounted for: a pure rotation about the end bone's world direction,
    // distributed along the chain by length so the spine rolls gradually.
    const glm::vec3 endDirRest = (endLen > 1e-6f) ? endOffset / endLen : s0;
    const glm::vec3 endDirWorld = target.rotation * endDirRest;
    const glm::quat endRef = quatFromTo(rootRot * endDirRest, endDirWorld) * rootRot;
    glm::quat twistDelta = glm::normalize(target.rotation * glm::inverse(endRef));
    if (twistDelta.w < 0.0f)
        twistDelta = -twistDelta;  // shortest-path twist in (-180, 180]
    const float twistAngle = 2.0f * std::atan2(
        glm::dot(glm::vec3(twistDelta.x, twistDelta.y, twistDelta.z), endDirWorld),
        twistDelta.w);
    const float twistTotalLen = totalLen + endLen;

    // Segment bone orientations: minimal swing from the root frame onto the
    // solved segment direction (keeps FK positions exact), plus blended twist.
    glm::quat prevWorld = rootRot;
    accum = 0.0f;
    for (size_t i = 0; i < count; ++i)
    {
        const glm::vec3 segDirWorld =
            rootRot * (swing * (glm::angleAxis(psi * midW[i], axis) * s0));
        const glm::vec3 restDir =
            (len[i] > 1e-6f) ? skeleton.joints[chain[i]].restOffset / len[i] : s0;
        const glm::quat swingRot = quatFromTo(rootRot * restDir, segDirWorld);
        accum += len[i];
        const glm::quat twist = glm::angleAxis(twistAngle * accum / twistTotalLen, segDirWorld);
        const glm::quat worldRot = glm::normalize(twist * swingRot * rootRot);
        skeleton.joints[chain[i]].localRot =
            glm::normalize(glm::inverse(prevWorld) * worldRot);
        prevWorld = worldRot;
    }
    skeleton.joints[endJoint].localRot =
        glm::normalize(glm::inverse(prevWorld) * target.rotation);
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

    // solveTwoBoneIk works with world rest directions (restDir already includes
    // socketRot), so its rotations compose on top of the socket's frame:
    // worldRot_i = result.rot_i * socketRot maps the local rest offset onto the
    // solved world direction.
    const glm::quat world1 = result.rot1 * socketRot;
    const glm::quat world2 = result.rot2 * socketRot;
    skeleton.joints[j1].localRot = glm::normalize(glm::inverse(socketRot) * world1);
    skeleton.joints[j2].localRot = glm::normalize(glm::inverse(world1) * world2);
    skeleton.joints[tip].localRot = glm::normalize(glm::inverse(world2) * target.rotation);
}
