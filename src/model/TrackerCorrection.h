#pragma once

#include "Pose.h"

#include <optional>
#include <vector>

class IkRig;
class Skeleton;
class TrackerCalibration;

// Per-IK-target correction state: the avatar joint each target's tracker is
// placed on (matched by joint name — built once at startup) and whether the
// user has enabled the correction for that target. Targets with no matching
// avatar joint can never be enabled. Indices parallel IkRig::targets.
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
    // re-hung on, or nullopt when the avatar has no joint of that name.
    std::vector<std::optional<int>> avatarJoint;

    // Per target index: user toggle. Defaults to true for every mapped
    // target; unmapped targets stay false and BeginDisabled in the UI.
    std::vector<bool> enabled;
};

// Builds the correction map by matching each IK target's joint name to an
// avatar joint. mapped targets start enabled. One linear scan per target;
// call once at startup (the rig config is loaded once and the avatar
// skeleton does not change at runtime).
CorrectionMap buildCorrectionMap(const IkRig& rig, const Skeleton& avatar);

// One corrected device pose: the target index it corrects, the stable device
// id (so the future driver-emission step and the UI readout can carry it), and
// the pose to feed back as the device's new world pose.
struct CorrectedPose
{
    size_t targetIndex = 0;
    int deviceId = -1;
    Pose pose;
};

// Computes corrected device poses for every enabled, mapped, bound target:
// corrected = the avatar joint's world pose (position + rotation), no strap
// offset. Empty when uncalibrated. One FK pass over `avatar`. The avatar must
// already have been posed (retargetPose called this frame) before this is
// called.
std::vector<CorrectedPose> correctDevicePoses(const TrackerCalibration& calibration,
                                              const CorrectionMap& map,
                                              const Skeleton& avatar);
