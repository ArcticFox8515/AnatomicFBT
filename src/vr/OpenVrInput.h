#pragma once

#include <array>
#include <string>
#include <vector>

#include "model/FrameTick.h"
#include "model/GripOffsets.h"

// On-demand OpenVR background client used only in Calibration mode to read the
// both-triggers gesture (no driver-side input source exists — see
// doc/driver-plan.md "Buttons stay client-side"). Poses come from the driver
// link (OpenVrTracking in the model layer); this is the only place openvr.h is
// included. Owned by main as a `unique_ptr`, constructed when entering
// Calibration and destroyed on leaving it (dtor calls VR_Shutdown). Implements
// IGestureSource so the frame tick (model layer) can read the gesture through
// a seam without itself depending on openvr.
//
// Also resolves per-controller grip-pose offsets from the OpenVR render-model
// component API (IVRRenderModels::GetComponentState on the "grip" component,
// fallback "handgrip" on controllers that lack one — see
// doc/ik-improvements-plan.md "controller pose = tip, not palm"). The query
// runs once in init() while the VR_Init session is alive; main caches the
// result and feeds it to pollAndUpdate so the controller pose lands at the
// grip point every frame, live and in replay. The model layer's GripOffset is
// the data the frame tick consumes; GripOffsetInfo carries the diagnostic
// strings (render model name, component name) main logs once per device.
class OpenVrInput : public IGestureSource
{
public:
    OpenVrInput() = default;
    ~OpenVrInput();  // calls VR_Shutdown when initialized

    OpenVrInput(const OpenVrInput&) = delete;
    OpenVrInput& operator=(const OpenVrInput&) = delete;

    // Initializes OpenVR as a background application (never launches SteamVR;
    // fails fast when it is not running). Idempotent: re-calling after
    // success is a no-op, so the UI can offer a retry after failure. On
    // success also resolves the per-controller grip offsets (see
    // gripOffsets); failures of the grip query never throw — the offset
    // stays identity for that device and is reported through gripOffsets.
    void init();

    bool isInitialized() const override;

    // Rising-edge detection: true on the frame both controllers' triggers
    // transitioned to pressed (the second one counts). State is kept between
    // calls, so call exactly once per frame.
    bool bothTriggersJustPressed() override;

    // One resolved grip offset per controller detected at init() time.
    // renderModelName / componentName are the strings main logs once per
    // device; componentName is "grip", "handgrip", or "" (no usable
    // component — deviceToGrip is identity). Empty before init() runs.
    struct GripOffsetInfo
    {
        int deviceId = -1;
        std::string renderModelName;
        std::string componentName;
        GripOffset offset;  // identity deviceToGrip by default
    };
    const std::vector<GripOffsetInfo>& gripOffsets() const { return gripOffsets_; }

private:
    bool initialized_ = false;
    std::array<bool, 2> triggerHeld_{false, false};  // [left, right]
    std::vector<GripOffsetInfo> gripOffsets_;

    void queryGripOffsets();
};
