#pragma once

#include <array>

// On-demand OpenVR background client used only in Calibration mode to read the
// both-triggers gesture (no driver-side input source exists — see
// doc/driver-plan.md "Buttons stay client-side"). Poses come from the driver
// link (OpenVrTracking in the model layer); this is the only place openvr.h is
// included. Owned by main as a `unique_ptr`, constructed when entering
// Calibration and destroyed on leaving it (dtor calls VR_Shutdown).
class OpenVrInput
{
public:
    OpenVrInput() = default;
    ~OpenVrInput();  // calls VR_Shutdown when initialized

    OpenVrInput(const OpenVrInput&) = delete;
    OpenVrInput& operator=(const OpenVrInput&) = delete;

    // Initializes OpenVR as a background application (never launches SteamVR;
    // fails fast when it is not running). Idempotent: re-calling after
    // success is a no-op, so the UI can offer a retry after failure.
    void init();

    bool isInitialized() const;

    // Rising-edge detection: true on the frame both controllers' triggers
    // transitioned to pressed (the second one counts). State is kept between
    // calls, so call exactly once per frame.
    bool bothTriggersJustPressed();

private:
    bool initialized_ = false;
    std::array<bool, 2> triggerHeld_{false, false};  // [left, right]
};