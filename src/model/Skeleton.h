#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "BodyProportions.h"

struct Joint
{
    std::string name;
    std::optional<int> parentIndex = std::nullopt;
    glm::vec3 restOffset{0.0f, 0.0f, 0.0f};      // translation from parent joint, meters
    glm::quat localRot{1.0f, 0.0f, 0.0f, 0.0f};  // identity at rest; not serialized
};

// Pure data class: joints are public and kept sorted parent-before-child.
// The only logic here is JSON serialization.
class Skeleton
{
public:
    std::vector<Joint> joints;

    // World position of the root joint. Runtime-only (not serialized); initialized
    // from the root's restOffset at load time. The root's restOffset is NOT applied
    // by forward kinematics — it only seeds this value.
    glm::vec3 rootPosition{0.0f, 0.0f, 0.0f};

    // Builds the default SlimeVR-style head-rooted skeleton (22 joints, fixed
    // hierarchy and bone names) scaled to the given body proportions. Bone
    // lengths equal the proportions; landmark heights hold relative to the
    // skeleton's own ankles: Chest (arm attachment) sits shoulderHeight above
    // them, Waist navelHeight, Hips upperLeg + lowerLeg. The root's rest Y
    // (shoulderHeight + neckLength) only seeds rootPosition — a rigid
    // translation of the rest pose, overwritten by calibration in VR modes.
    // The fictional mid-spine joint splits its span at the midpoint;
    // head-height (0.15, Head->Neck) and hand/foot bone lengths (0.12/0.08)
    // are internal constants — hand/foot lengths provably cancel out of the
    // capture-mode IK (the calibration offset gains exactly the term the
    // two-bone solver subtracts back).
    static Skeleton makeDefault(const BodyProportions& proportions = BodyProportions());

    // Builds the same skeleton as makeDefault() but rooted at "Hips" with the
    // spine chain reversed (Hips -> Waist -> ... -> Head), like
    // VRChat/Unity avatars. Rest world positions are identical.
    static Skeleton makeDefaultHipRooted();
};

void to_json(nlohmann::json& j, const Skeleton& skeleton);
void from_json(const nlohmann::json& j, Skeleton& skeleton);

// FK result for the whole skeleton.
struct WorldTransforms
{
    std::vector<glm::vec3> positions;
    std::vector<glm::quat> rotations;  // world orientation of the bone ending at each joint
};

// Render-ready transform of a single bone (the segment parent joint -> joint).
// `rotation` maps the unit pyramid's +Y axis onto the bone's current world
// direction AND carries the joint's real axial twist, so it is a truthful
// visualization of the bone's world orientation (unlike a minimal rotation
// from +Y, which is degenerate near the -Y antipode where every downward
// bone lives). Zero-length bones are omitted. Indexes the original joint.
struct BoneFrame
{
    glm::vec3 base{0.0f, 0.0f, 0.0f};      // parent joint world position
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};  // bone world frame, +Y along the bone
    float length{0.0f};                    // |restOffset|, meters
    int joint{-1};
};

// One BoneFrame per non-root, non-zero-length joint, in joint order. Computed
// from the live world transforms: rotation = wt.rotations[i] composed with the
// (constant per bone) rotation that takes +Y onto the rest offset direction.
// Twist of the joint's localRot is reflected; positions are the live FK
// positions, so posed skeletons render correctly.
std::vector<BoneFrame> computeBoneFrames(const Skeleton& skeleton);

// Hierarchical forward kinematics, one linear pass over the (parent-before-child
// sorted) joints. The root sits at skeleton.rootPosition with orientation
// localRot. Children: worldRot = parentWorldRot * localRot,
// pos = parentPos + worldRot * restOffset.
WorldTransforms computeWorldTransforms(const Skeleton& skeleton);

// Convenience wrapper: positions only.
std::vector<glm::vec3> computeWorldPositions(const Skeleton& skeleton);

// Rest-pose world positions: restOffsets accumulated down the hierarchy with
// localRot ignored (identity at rest), root at the origin. Pose-independent,
// so it is valid on a skeleton whose localRots have already been posed by
// retargetPose. Used to measure bone lengths / landmark heights without caring
// about the current pose.
std::vector<glm::vec3> computeRestPositions(const Skeleton& skeleton);

// Head-to-feet Y span of the rest pose, by bone name: Head.y minus the lowest
// present foot joint (LeftFoot/RightFoot) y. Translation-invariant (rooted at
// the origin), so it equals the body height regardless of where the root sits.
// Returns 0 when Head or both feet are missing, or the span is not positive.
float restHeight(const Skeleton& skeleton);

// Uniform scale of the rest pose: every restOffset and rootPosition multiplied
// by `scale`. Pure data mutation; leaves localRot untouched.
void scaleSkeleton(Skeleton& skeleton, float scale);

// Scales `dst` so restHeight(dst) == restHeight(src). Returns the applied scale
// (dst.height / src.height); 1.0 and leaves dst untouched when either height is
// unusable (a missing Head or both feet). Call once after loading the avatar,
// before retargeting/correction maps are built.
float matchRestHeight(const Skeleton& src, Skeleton& dst);
