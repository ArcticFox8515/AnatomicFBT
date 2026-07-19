#pragma once

#include "IkSolvers.h"
#include "Pose.h"

#include <glm/glm.hpp>

#include <optional>
#include <utility>
#include <vector>

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
