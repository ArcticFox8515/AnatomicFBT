#pragma once

#include "Pose.h"

#include <array>
#include <optional>
#include <string_view>
#include <vector>

class IkRig;
class Skeleton;
class TrackerCalibration;
struct TrackedDevice;

// Correction groups: trackers that move together are toggled as one rather
// than per-bone — there is no real case for correcting one leg but not the
// other. Each group shares a single enable + rotation-enable checkbox in the
// UI. New groups (Knees, Elbows, Chest) are forward-declared here so the
// classifier and the arrays are ready before those bones become IK targets.
enum class CorrectionGroup : int
{
    Hands = 0,
    Feet,
    Hips,
    Knees,
    Elbows,
    Chest,
    Count
};
constexpr size_t kCorrectionGroupCount = static_cast<size_t>(CorrectionGroup::Count);

// Maps a bone name to its correction group, or nullopt when the bone is not
// correctable (e.g. the Head anchor) or does not belong to any group. Based on
// BoneNames; left/right symmetric.
std::optional<CorrectionGroup> correctionGroupForBone(std::string_view name);

// Human-readable label for a group (used by the ImGui panel).
const char* correctionGroupName(CorrectionGroup group);

// Per-IK-target correction state: the avatar joint each target's tracker is
// placed on (matched by joint name — built once at startup), the correction
// group the target belongs to, and the per-group enable/rotation toggles.
// Indices parallel IkRig::targets.
//
// The corrected pose is the avatar joint's world pose directly — the tracker
// is placed at the center of the avatar's matching bone, with no strap offset.
// The bone-local offset captured at calibration is deliberately dropped: the
// reference and avatar skeletons have differently-oriented bone frames (rest
// rotations / bone roll), so re-hanging in the local frame would rotate the
// tracker wrong. The joint center is frame-independent. See doc/plan.md
// milestone 4.
struct CorrectionMap
{
    // Per target index: the avatar joint whose world pose the tracker is
    // re-hung on, or nullopt when the avatar has no joint of that name (or the
    // target was skipped — e.g. the Head anchor).
    std::vector<std::optional<int>> avatarJoint;

    // Per target index: the correction group the target belongs to, or
    // nullopt when the target has no group (unmapped, Head, or a bone not
    // covered by any group). Targets with no group are never corrected.
    std::vector<std::optional<CorrectionGroup>> targetGroup;

    // Per group: user toggle for position correction. Defaults to true for
    // every group that has at least one mapped target; groups with no mapped
    // target stay false and the UI does not render them.
    std::array<bool, kCorrectionGroupCount> groupEnabled{};

    // Per group: user toggle for rotation correction. Controllers are always
    // rotation-locked regardless of this value (aiming must not change), so it
    // is meaningless for the Hands group when all its bound devices are
    // controllers — the UI disables the checkbox then. When false on a tracker,
    // the tracker keeps its raw rotation (position-only correction, same as a
    // controller) — useful when avatar bone-roll differences produce a visually
    // wrong tracker orientation.
    std::array<bool, kCorrectionGroupCount> groupRotationEnabled{};

    // Per group: whether at least one mapped target belongs to this group, set
    // once at build time so the UI knows which groups to render.
    std::array<bool, kCorrectionGroupCount> groupPresent{};
};

// Builds the correction map by matching each IK target's joint name to an
// avatar joint and classifying it into a correction group. Mapped groups start
// enabled (both position and rotation). One linear scan per target; call once
// at startup (the rig config is loaded once and the avatar skeleton does not
// change at runtime).
CorrectionMap buildCorrectionMap(const IkRig& rig, const Skeleton& avatar);

// One corrected device pose: the target index it corrects, the stable device
// id (so the future driver-emission step and the UI readout can carry it), and
// the pose to feed back as the device's new world pose. `rotationLocked` is
// set when the raw device rotation must be preserved (controllers — always —
// or trackers with rotation correction disabled): the pose carries the
// corrected position but the raw rotation, and correctionOffsets emits an
// identity-rotation delta.
struct CorrectedPose
{
    size_t targetIndex = 0;
    int deviceId = -1;
    Pose pose;
    bool rotationLocked = false;
};

// Computes corrected device poses for every enabled, mapped, bound target in
// an enabled group: corrected = the avatar joint's world pose (position +
// rotation), no strap offset. Controllers always keep their raw rotation
// (aiming must not change); trackers keep it too when the group's
// `groupRotationEnabled` is false. A device bound but absent from `devices`
// this frame is skipped (no raw pose to correct, no stale marker to render).
// Empty when uncalibrated. One FK pass over `avatar`. The avatar must already
// have been posed (retargetPose called this frame) before this is called.
std::vector<CorrectedPose> correctDevicePoses(const TrackerCalibration& calibration,
                                              const CorrectionMap& map,
                                              const Skeleton& avatar,
                                              const std::vector<TrackedDevice>& devices);

// One device's correction for the driver: a rigid world-space delta such that
// `compose(delta, rawDevicePose)` yields the corrected pose. The driver
// premultiplies `worldFromDriver` by it.
struct DeviceOffset
{
    int deviceId = -1;
    Pose delta;
};

// Builds the per-frame override set from the corrected poses and the raw
// device poses: `delta = compose(corrected, inverse(raw))`. When a corrected
// pose has `rotationLocked`, the delta is emitted with an exactly-identity
// rotation (position only) — the driver applies a pure translation so the
// controller's aim is untouched. Iterates `corrected` only — an unticked group
// simply has no entry this frame, and the driver expires its last override
// after `kOverrideStaleSeconds`.
std::vector<DeviceOffset> correctionOffsets(const std::vector<CorrectedPose>& corrected,
                                            const std::vector<TrackedDevice>& devices);
