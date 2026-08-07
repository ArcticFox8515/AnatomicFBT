#pragma once

// Throwaway step-1 spike (doc/driver-plan.md): the log labels for OpenVR enum values.
//
// Deliberately openvr-free in the header (the values arrive as ints) so both the
// driver DLL (openvr_driver.h) and the spike client (openvr.h) can use it without
// pulling the other API's headers in.

namespace spike
{
// vr::ETrackedDeviceClass -> "hmd" / "controller" / "tracker" / ...
const char* deviceClassName(int deviceClass);

// vr::ETrackedControllerRole -> "left_hand" / "right_hand" / ...
const char* roleHintName(int role);
} // namespace spike
