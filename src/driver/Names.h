#pragma once

// The log labels for OpenVR enum values.
//
// Deliberately openvr-free in the header (the values arrive as ints) so the
// driver DLL (openvr_driver.h) can use it without pulling the client API's headers in.

namespace driver
{
// The tracking system name that identifies devices belonging to this driver
// (matches the `name` field in driver.vrdrivermanifest and the value written to
// Prop_TrackingSystemName_String on virtual trackers). The Observer uses it to
// filter our own devices out of the downstream device stream (step 7).
constexpr const char* kOurTrackingSystemName = "00trackingcorrector";

// vr::ETrackedDeviceClass -> "hmd" / "controller" / "tracker" / ...
const char* deviceClassName(int deviceClass);

// vr::ETrackedControllerRole -> "left_hand" / "right_hand" / ...
const char* roleHintName(int role);
} // namespace driver
