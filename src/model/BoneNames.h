#pragma once

// Canonical names of the default skeleton's bones, in one place so the
// skeleton, the default IK config and any name-based lookups agree. The names
// follow Unity's HumanBodyBones naming — matching what
// unity/AvatarSkeletonExporter.cs emits — so a same-named avatar skeleton
// retargets with no name translation. The head-rooted spine chain runs
// Head -> Neck -> Chest -> Spine -> Waist -> Hips (so our "upper chest", the
// shoulder parent, is Unity's "Chest", and our "chest" is Unity's "Spine").

namespace BoneNames
{
inline constexpr const char* Head = "Head";
inline constexpr const char* Neck = "Neck";
inline constexpr const char* Chest = "Chest";
inline constexpr const char* Spine = "Spine";
inline constexpr const char* Waist = "Waist";
inline constexpr const char* Hips = "Hips";

inline constexpr const char* LeftHip = "LeftHip";
inline constexpr const char* LeftUpperLeg = "LeftUpperLeg";
inline constexpr const char* LeftLowerLeg = "LeftLowerLeg";
inline constexpr const char* LeftFoot = "LeftFoot";

inline constexpr const char* RightHip = "RightHip";
inline constexpr const char* RightUpperLeg = "RightUpperLeg";
inline constexpr const char* RightLowerLeg = "RightLowerLeg";
inline constexpr const char* RightFoot = "RightFoot";

inline constexpr const char* LeftShoulder = "LeftShoulder";
inline constexpr const char* LeftUpperArm = "LeftUpperArm";
inline constexpr const char* LeftLowerArm = "LeftLowerArm";
inline constexpr const char* LeftHand = "LeftHand";

inline constexpr const char* RightShoulder = "RightShoulder";
inline constexpr const char* RightUpperArm = "RightUpperArm";
inline constexpr const char* RightLowerArm = "RightLowerArm";
inline constexpr const char* RightHand = "RightHand";
} // namespace BoneNames
