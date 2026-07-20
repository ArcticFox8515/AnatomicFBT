#pragma once

#include "Pose.h"

#include <glm/glm.hpp>

#include <utility>
#include <vector>

// Hardware-agnostic tracked-device category, as reported by a tracking
// provider (the VR layer) and consumed by the model layer.
enum class TrackedDeviceKind
{
    Hmd,
    Controller,
    Tracker,
    Other
};

// One frame's snapshot of a single tracked device, in world space
// (right-handed, Y-up, meters). `id` is an opaque stable identifier chosen by
// the tracking provider (for OpenVR: the tracked device index).
struct TrackedDevice
{
    int id = -1;
    TrackedDeviceKind kind = TrackedDeviceKind::Other;
    Pose pose;
};

// First HMD in a device snapshot list, or nullptr.
const TrackedDevice* findHmd(const std::vector<TrackedDevice>& devices);

// Short label for UI display.
const char* deviceKindName(TrackedDeviceKind kind);

// Packs the snapshot list into (id, pose) pairs — the device id space
// TrackerCalibration binds against.
std::vector<std::pair<int, Pose>> devicePosePairs(const std::vector<TrackedDevice>& devices);
