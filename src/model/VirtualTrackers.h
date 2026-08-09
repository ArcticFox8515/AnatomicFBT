#pragma once

#include "Pose.h"

#include <string>
#include <vector>

class IkRig;
class Skeleton;
class TrackerCalibration;

// Bones eligible for a virtual tracker (doc/virtual-trackers-plan.md step 1):
// joints present in BOTH the source skeleton (rig.skeleton) and the avatar
// skeleton, excluding joints that are IK targets — those are "mapped to
// trackers" (Head is the HMD anchor; hands/feet/Hips are tracker targets),
// so a virtual tracker never collides with a real one. Exclusion is by
// target configuration, not by device binding: an IK target bone is out
// whether or not a device is currently bound to it, so the list is stable
// across calibration state (the UI shows the same candidates before and
// after calibration). No socket/anchor special-casing beyond "is a target".
//
// The `calibration` parameter is retained for signature compatibility but
// is not consulted — eligibility no longer depends on device binding.
//
// Names are compared, not rest offsets, so a height-scaled avatar yields the
// same set as an unscaled one. Returned in source-skeleton order
// (parent before child) for stable UI listing.
std::vector<std::string> eligibleVirtualTrackerBones(
    const IkRig& rig, const Skeleton& avatar, const TrackerCalibration& calibration);

// One emitted virtual tracker (doc/virtual-trackers-plan.md step 2): the bone
// name (matches the step-1 list) and its world pose on the avatar.
struct VirtualTrackerPose
{
    std::string name;
    Pose pose;
};

// Computes the virtual-tracker pose for each named bone from the avatar's
// current pose (run after retargetPose so the avatar reflects the live pose):
// position = the midpoint of the bone's two joint world positions (the avatar
// joint the bone ends at, plus its parent), rotation = that avatar joint's
// world rotation (the bone's world orientation). Recomputed every frame —
// the source is the live avatar FK, not the rest pose. A bone not found in
// the avatar, or whose avatar joint is the root (no parent), is skipped.
// Output order matches the input order. Model layer only; no GL, no OpenVR.
std::vector<VirtualTrackerPose> computeVirtualTrackerPoses(
    const Skeleton& avatar, const std::vector<std::string>& boneNames);

// Owns the virtual-tracker visualization: bound to the rig, avatar, and the
// user's ticked-bone selection, it produces the per-frame marker poses the
// render loop draws. main.cpp constructs one after the avatar is built and
// the settings are loaded, calls `poses()` each frame after the avatar is
// retargeted, and hands the result straight to `scene.renderMarkers` — no
// per-frame logic in the wiring (the class is the logic, like the driver's
// Server/Observer classes). Construction is cheap; the avatar is held by
// reference and must outlive the renderer.
class VirtualTrackerRenderer
{
public:
    VirtualTrackerRenderer(const IkRig& rig, const Skeleton& avatar,
                           const TrackerCalibration& calibration,
                           const std::vector<std::string>& selectedBones);

    // The marker poses for the ticked eligible bones, computed from the
    // avatar's current pose. Call after retargetPose so the avatar reflects
    // the live pose. Returns one pose per ticked eligible bone (ineligible
    // or absent bones are skipped), in source-skeleton order.
    std::vector<Pose> poses() const;

private:
    const IkRig& rig_;
    const Skeleton& avatar_;
    const TrackerCalibration& calibration_;
    const std::vector<std::string>& selectedBones_;
};
