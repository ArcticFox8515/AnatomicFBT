#include "SpikeNames.h"

#include <openvr_driver.h>

namespace spike
{
const char* deviceClassName(int deviceClass)
{
    switch (deviceClass)
    {
    case vr::TrackedDeviceClass_Invalid: return "invalid";
    case vr::TrackedDeviceClass_HMD: return "hmd";
    case vr::TrackedDeviceClass_Controller: return "controller";
    case vr::TrackedDeviceClass_GenericTracker: return "tracker";
    case vr::TrackedDeviceClass_TrackingReference: return "reference";
    case vr::TrackedDeviceClass_DisplayRedirect: return "display_redirect";
    default: return "unknown";
    }
}

const char* roleHintName(int role)
{
    switch (role)
    {
    case vr::TrackedControllerRole_Invalid: return "invalid";
    case vr::TrackedControllerRole_LeftHand: return "left_hand";
    case vr::TrackedControllerRole_RightHand: return "right_hand";
    case vr::TrackedControllerRole_OptOut: return "opt_out";
    case vr::TrackedControllerRole_Treadmill: return "treadmill";
    case vr::TrackedControllerRole_Stylus: return "stylus";
    default: return "unknown";
    }
}

bool endsWith(const std::string& text, const std::string& suffix)
{
    return text.size() >= suffix.size()
           && text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool isTriggerClick(const std::string& componentName)
{
    return endsWith(componentName, "/input/trigger/click");
}
} // namespace spike
