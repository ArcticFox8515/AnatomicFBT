#pragma once

#include "IkSolvers.h"
#include "Pose.h"
#include "TrackedDevice.h"

#include <glm/glm.hpp>

#include <optional>
#include <utility>
#include <vector>

class IkRig;

// Result of matching tracked devices to IK targets by proximity.
// deviceIndex[targetIndex] / targetIndex[deviceIndex] hold the counterpart's
// index, or -1 when a target/device has no counterpart (more of one than the
// other, or no devices at all).
struct DeviceAssignment
{
    std::vector<int> deviceIndex;
    std::vector<int> targetIndex;
};

// Greedy nearest-pair matching between device and target positions: the
// globally closest pair is locked first, then the next closest of the
// remainder, etc. Deterministic (ties broken by input order). Used during
// calibration so trackers self-identify as feet/hip by where they sit
// relative to the rest-pose targets (user stands in T-pose).
DeviceAssignment assignDevicesToTargets(const std::vector<glm::vec3>& devicePositions,
                                        const std::vector<glm::vec3>& targetPositions);

// Device-to-target binding remembered at calibration time: each bound target
// keeps the pose of its device in the target's frame (the "offset"), so that
// composing a live device pose with the offset reproduces the bone pose the
// tracker was calibrated against. Knows nothing about OpenVR — devices
// arrive as plain (deviceId, pose) pairs with an opaque caller-chosen int id.
class TrackerCalibration
{
public:
    // Captures the offset of every assigned target: offset = inverse(device)
    // * boneWorld. assignment.deviceIndex holds device ids into `devices`;
    // boneWorldPoses is indexed by target index. Unassigned targets stay
    // unbound.
    void calibrate(const DeviceAssignment& assignment,
                   const std::vector<std::pair<int, Pose>>& devices,
                   const std::vector<Pose>& boneWorldPoses);

    // Writes the raw pose of each bound target's device into the target —
    // this is what gets rendered as the marker and what the user perceives
    // as "the tracker". Targets with a missing device keep their current
    // pose. Returns the number of targets updated.
    size_t applyDevicePoses(const std::vector<std::pair<int, Pose>>& devices,
                            std::vector<IkTarget>& targets) const;

    // Transforms targets (holding raw device poses) into solver goals:
    // goal = target * offset, per bound target. Unbound targets pass through
    // unchanged. Typically applied to a copy of the rig's targets which is
    // then handed to IkRig::solve(goals).
    void applyOffsets(std::vector<IkTarget>& goals) const;

    void clear();

    bool isCalibrated() const;

    // Device id bound to a target, or std::nullopt (for UI display).
    std::optional<int> boundDevice(size_t targetIndex) const;

private:
    struct Binding
    {
        int deviceId = -1;
        Pose offset;
    };

    static const Pose* findDevice(const std::vector<std::pair<int, Pose>>& devices, int deviceId);

    std::vector<std::optional<Binding>> bindings_;  // indexed by target index
};

// Result of one calibration-mode frame (see updateCalibrationFrame).
struct CalibrationFrame
{
    DeviceAssignment assignment;       // deviceIndex entries are positions in
                                       // the device list passed in (for UI)
    std::vector<Pose> boneWorldPoses;  // per-target world poses of the resting,
                                       // root-aligned skeleton (for calibrate())
};

// Runs one calibration frame on the rig: resets the skeleton to the rest
// pose, places the root from the HMD, matches devices to targets by
// proximity, and mirrors the matched devices' raw poses into the rig's
// targets (what gets rendered).
//
// Root placement: the HMD sits an arbitrary offset away from the Head joint,
// so the root is NOT pinned to the HMD position. The user T-poses, and in a
// T-pose the midpoint of the hands coincides with the Chest joint (the end
// of the neck bone) — an exact invariant of the rest skeleton. The hands are
// the two Controller-kind devices (the calibration gesture itself requires
// two controllers, so they are always present). When both are tracked, the
// root is shifted so the FK Chest lands on the measured midpoint; the
// correction is masked in the HMD-yaw frame, keeping height and forward from
// the hands and taking lateral from the HMD (assumed centered on the head).
// Without an HMD or both controllers the root falls back to plain HMD
// alignment; with no HMD it is left as-is. The HMD's offset from the Head
// joint ends up in the head target's Binding like every other target.
CalibrationFrame updateCalibrationFrame(IkRig& rig, const std::vector<TrackedDevice>& devices);

// Freezes a calibration frame into per-target offsets: binds each assigned
// target to the stable id of its device and stores offset =
// inverse(devicePose) * boneWorldPose. Call when the user confirms the
// calibration (both triggers).
void captureOffsets(TrackerCalibration& calibration, const CalibrationFrame& frame,
                    const std::vector<TrackedDevice>& devices);

// Runs one capture-mode frame: mirrors the raw poses of bound devices into
// the rig's targets (what gets rendered) and returns the solver goals — a
// copy of the targets with the calibrated offsets applied. The returned goals
// always parallel the rig's targets, so they can be handed straight to
// IkRig::solve(goals).
std::vector<IkTarget> updateCaptureFrame(IkRig& rig, const TrackerCalibration& calibration,
                                         const std::vector<TrackedDevice>& devices);
