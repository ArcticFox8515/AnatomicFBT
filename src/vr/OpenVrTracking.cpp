#include "OpenVrTracking.h"

#include "model/Error.h"

#include <openvr.h>

namespace
{
TrackedDeviceKind classify(vr::ETrackedDeviceClass deviceClass)
{
    switch (deviceClass)
    {
    case vr::TrackedDeviceClass_HMD:
        return TrackedDeviceKind::Hmd;
    case vr::TrackedDeviceClass_Controller:
        return TrackedDeviceKind::Controller;
    case vr::TrackedDeviceClass_GenericTracker:
        return TrackedDeviceKind::Tracker;
    default:
        return TrackedDeviceKind::Other;
    }
}

// HmdMatrix34 is a row-major 3x4 rigid transform in a right-handed, Y-up,
// meter space — the same convention as glm, so a plain transpose into a
// column-major mat4 is the whole conversion.
Pose toPose(const vr::HmdMatrix34_t& m)
{
    const glm::mat4 mat(
        m.m[0][0], m.m[1][0], m.m[2][0], 0.0f,
        m.m[0][1], m.m[1][1], m.m[2][1], 0.0f,
        m.m[0][2], m.m[1][2], m.m[2][2], 0.0f,
        m.m[0][3], m.m[1][3], m.m[2][3], 1.0f);
    return {glm::vec3(mat[3]), glm::quat_cast(mat)};
}

bool triggerPressed(vr::IVRSystem* system, vr::ETrackedControllerRole role, bool& wasHeld)
{
    const vr::TrackedDeviceIndex_t index = system->GetTrackedDeviceIndexForControllerRole(role);
    if (index == vr::k_unTrackedDeviceIndexInvalid)
    {
        wasHeld = false;
        return false;
    }

    vr::VRControllerState_t state{};
    if (!system->GetControllerState(index, &state, sizeof(state)))
    {
        wasHeld = false;
        return false;
    }

    const bool held =
        (state.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger)) != 0;
    const bool justPressed = held && !wasHeld;
    wasHeld = held;
    return justPressed;
}
} // namespace

OpenVrTracking::~OpenVrTracking()
{
    if (initialized_)
        vr::VR_Shutdown();
}

void OpenVrTracking::init()
{
    if (initialized_)
        return;

    vr::EVRInitError initError = vr::VRInitError_None;
    vr::VR_Init(&initError, vr::VRApplication_Background);
    if (initError != vr::VRInitError_None)
        throw Error(std::string("OpenVR init failed: ")
                    + vr::VR_GetVRInitErrorAsEnglishDescription(initError));
    initialized_ = true;
}

bool OpenVrTracking::isInitialized() const
{
    return initialized_;
}

std::vector<TrackedDevice> OpenVrTracking::pollPoses() const
{
    std::vector<TrackedDevice> result;
    if (!initialized_)
        return result;

    vr::IVRSystem* system = vr::VRSystem();
    if (!system)
        return result;

    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
    system->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0f, poses,
                                            vr::k_unMaxTrackedDeviceCount);

    for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
    {
        if (!poses[i].bPoseIsValid || !poses[i].bDeviceIsConnected)
            continue;
        const TrackedDeviceKind kind = classify(system->GetTrackedDeviceClass(i));
        if (kind == TrackedDeviceKind::Other)
            continue;
        result.push_back({static_cast<int>(i), kind, toPose(poses[i].mDeviceToAbsoluteTracking)});
    }
    return result;
}

bool OpenVrTracking::bothTriggersJustPressed()
{
    if (!initialized_)
        return false;

    vr::IVRSystem* system = vr::VRSystem();
    if (!system)
        return false;

    const bool leftJustPressed = triggerPressed(system, vr::TrackedControllerRole_LeftHand,
                                                triggerHeld_[0]);
    const bool rightJustPressed = triggerPressed(system, vr::TrackedControllerRole_RightHand,
                                                 triggerHeld_[1]);
    // True when the second trigger goes down while the first is still held.
    return (leftJustPressed && triggerHeld_[1]) || (rightJustPressed && triggerHeld_[0]);
}
