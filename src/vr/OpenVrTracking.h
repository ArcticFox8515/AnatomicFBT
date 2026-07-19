#pragma once

#include "model/Pose.h"

#include <glm/glm.hpp>

#include <array>
#include <vector>

// Device category reported by SteamVR tracking.
enum class VrDeviceKind
{
    Hmd,
    Controller,
    Tracker,
    Other
};

// One frame's snapshot of a single tracked device, in SteamVR's standing
// tracking universe (right-handed, Y-up, meters — same convention as our scene).
struct VrDeviceSnapshot
{
    int deviceIndex = -1;  // OpenVR tracked device index; doubles as the stable id
    VrDeviceKind kind = VrDeviceKind::Other;
    Pose pose;
};

// Polls tracked device poses and controller trigger state from OpenVR.
// Non-owning of anything except the VR_Init session: construction is cheap and
// never initializes OpenVR (per the no-throwing-constructors convention);
// call init() explicitly, it throws Error on failure and can be retried.
class OpenVrTracking
{
public:
    OpenVrTracking() = default;
    ~OpenVrTracking();  // calls VR_Shutdown when initialized

    OpenVrTracking(const OpenVrTracking&) = delete;
    OpenVrTracking& operator=(const OpenVrTracking&) = delete;

    // Initializes OpenVR as a background application (never launches SteamVR;
    // fails fast when it is not running). Idempotent: re-calling after
    // success is a no-op, so the UI can offer a retry after failure.
    void init();

    bool isInitialized() const;

    // Pose of every connected device with a valid pose this frame.
    std::vector<VrDeviceSnapshot> pollPoses() const;

    // Rising-edge detection: true on the frame both controllers' triggers
    // transitioned to pressed (the second one counts). State is kept between
    // calls, so call exactly once per frame.
    bool bothTriggersJustPressed();

private:
    bool initialized_ = false;
    std::array<bool, 2> triggerHeld_{false, false};  // [left, right]
};
