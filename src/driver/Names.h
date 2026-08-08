#pragma once

// The log labels for OpenVR enum values.
//
// Deliberately openvr-free in the header (the values arrive as ints) so the
// driver DLL (openvr_driver.h) can use it without pulling the client API's headers in.

namespace driver
{
// vr::ETrackedDeviceClass -> "hmd" / "controller" / "tracker" / ...
const char* deviceClassName(int deviceClass);

// vr::ETrackedControllerRole -> "left_hand" / "right_hand" / ...
const char* roleHintName(int role);
} // namespace driver
