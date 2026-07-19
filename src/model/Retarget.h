#pragma once

#include <optional>
#include <string>
#include <vector>

class Skeleton;

// Mapping from dst joints to src joints, built once by bone connectivity:
// a dst bone parent->child matches the src bone connecting the same two joint
// names in either direction. This makes pose transfer exact even when the two
// hierarchies are rooted differently (e.g. a head-rooted source driving a
// hip-rooted VRChat-style avatar, where the spine chain runs opposite ways).
struct RetargetMap
{
    // Per dst joint: the src joint whose world rotation to copy, or nullopt
    // when the dst bone (or, for the root, the dst joint name) has no
    // counterpart in src.
    std::vector<std::optional<int>> dstToSrc;

    // The anchor pair: the dst joint whose world position is pinned to the
    // matching src joint's position (typically "head" — the HMD stays fixed,
    // the avatar root is placed under it), or nullopt when no name matches.
    std::optional<int> anchorDst;
    std::optional<int> anchorSrc;
};

// Builds the retarget mapping between two skeletons by joint names.
RetargetMap buildRetargetMap(const Skeleton& src, const Skeleton& dst);

// Copies the pose from src to dst through the map: world rotations transfer
// per matched bone, unmatched dst joints keep their current rotation, and
// dst's rootPosition is shifted so the anchor joint lands exactly on the src
// anchor joint's world position. Stateless; call every frame after the src
// pose is final.
void retargetPose(const Skeleton& src, Skeleton& dst, const RetargetMap& map);

// Names of dst joints with no src counterpart (for one-time logging).
std::vector<std::string> unmatchedBones(const Skeleton& dst, const RetargetMap& map);
