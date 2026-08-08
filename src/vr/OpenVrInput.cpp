#include "OpenVrInput.h"

#include "model/Error.h"

#include <openvr.h>

namespace
{
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

OpenVrInput::~OpenVrInput()
{
    if (initialized_)
        vr::VR_Shutdown();
}

void OpenVrInput::init()
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

bool OpenVrInput::isInitialized() const
{
    return initialized_;
}

bool OpenVrInput::bothTriggersJustPressed()
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