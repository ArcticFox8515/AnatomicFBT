#pragma once

// Throwaway step-1 spike (doc/driver-plan.md): the log labels for OpenVR enum values
// plus the component-name predicate.
//
// Deliberately openvr-free in the header (the values arrive as ints) so both the
// driver DLL (openvr_driver.h) and the spike client (openvr.h) can use it without
// pulling the other API's headers in.

#include <string>

namespace spike
{
// vr::ETrackedDeviceClass -> "hmd" / "controller" / "tracker" / ...
const char* deviceClassName(int deviceClass);

// vr::ETrackedControllerRole -> "left_hand" / "right_hand" / ...
const char* roleHintName(int role);

bool endsWith(const std::string& text, const std::string& suffix);

// The one input component the real driver cares about (doc/driver-plan.md): the
// calibration gesture is both triggers pressed.
bool isTriggerClick(const std::string& componentName);
} // namespace spike
