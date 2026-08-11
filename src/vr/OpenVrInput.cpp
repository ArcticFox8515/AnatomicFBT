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

// Reads Prop_RenderModelName_String for `device`. Returns "" on any error
// (the caller skips the device — no grip offset, identity pose).
std::string renderModelNameFor(vr::IVRSystem* system, vr::TrackedDeviceIndex_t device)
{
    char buffer[vr::k_unMaxPropertyStringSize] = {};
    vr::ETrackedPropertyError error = vr::TrackedProp_Success;
    system->GetStringTrackedDeviceProperty(device, vr::Prop_RenderModelName_String,
                                           buffer, sizeof(buffer), &error);
    return error == vr::TrackedProp_Success ? std::string(buffer) : std::string{};
}

// "grip" has no openvr.h constant (only gdc2015/base/tip/handgrip/status do)
// — it is an OpenXR-era addition present in the render-model JSON of most
// modern controllers. String literal is the only form; probe with
// RenderModelHasComponent so controllers that lack it fall back to handgrip.
constexpr const char* kGripComponent = "grip";

// Resolves one controller's grip offset from its render-model component.
// `models` caches the per-model-name component pick so left+right of the
// same model don't both probe every component. componentName is "grip",
// "handgrip", or "" (no usable component — deviceToGrip stays identity).
std::string pickComponentName(vr::IVRRenderModels* models, const std::string& renderModel)
{
    if (renderModel.empty())
        return {};
    // Force the render-model JSON parse before the per-component queries:
    // GetComponentState can return identity for a not-yet-loaded model.
    if (models->GetComponentCount(renderModel.c_str()) == 0)
        return {};
    if (models->RenderModelHasComponent(renderModel.c_str(), kGripComponent))
        return kGripComponent;
    if (models->RenderModelHasComponent(renderModel.c_str(),
                                        vr::k_pch_Controller_Component_HandGrip))
        return vr::k_pch_Controller_Component_HandGrip;
    return {};
}

Pose gripOffsetFor(vr::IVRRenderModels* models, const std::string& renderModel,
                   const std::string& component)
{
    if (component.empty())
        return {};  // identity
    vr::VRControllerState_t controllerState{};
    vr::RenderModel_ControllerMode_State_t modeState{false};
    vr::RenderModel_ComponentState_t componentState{};
    if (!models->GetComponentState(renderModel.c_str(), component.c_str(),
                                   &controllerState, &modeState, &componentState))
        return {};  // GetComponentState returns identity on failure, but be safe
    return poseFromHmdMatrix34(componentState.mTrackingToComponentLocal.m);
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

    // Resolve per-controller grip offsets while the VR_Init session is alive
    // (VR_Shutdown in the dtor invalidates IVRRenderModels). A failure of any
    // single device's query never throws — the offset stays identity and
    // componentName is "" so main can log it.
    queryGripOffsets();
}

void OpenVrInput::queryGripOffsets()
{
    gripOffsets_.clear();

    vr::IVRSystem* system = vr::VRSystem();
    vr::IVRRenderModels* models = vr::VRRenderModels();
    if (!system || !models)
        return;

    for (vr::TrackedDeviceIndex_t device = 0; device < vr::k_unMaxTrackedDeviceCount; ++device)
    {
        if (system->GetTrackedDeviceClass(device) != vr::TrackedDeviceClass_Controller)
            continue;

        GripOffsetInfo info;
        info.deviceId = static_cast<int>(device);
        info.renderModelName = renderModelNameFor(system, device);
        info.componentName = pickComponentName(models, info.renderModelName);
        info.offset.deviceId = info.deviceId;
        info.offset.deviceToGrip = gripOffsetFor(models, info.renderModelName, info.componentName);
        gripOffsets_.push_back(std::move(info));
    }
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
