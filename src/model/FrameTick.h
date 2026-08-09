#pragma once

#include "ModeController.h"
#include "Retarget.h"
#include "TrackerCorrection.h"
#include "TrackedDevice.h"
#include "link/Log.h"

#include <vector>

class IkRig;
class ReplaySession;
class SessionRecorder;
class Skeleton;
class TrackerCalibration;

// Seams for the hardware-backed sources the per-frame tick drives, so the
// orchestration is unit-testable without OpenVR / pipe bindings. The app's
// concrete sources implement these: `OpenVrTracking` (src/model) is the
// pose source (driver-link poses + override shipping); `OpenVrInput`
// (src/vr) is the gesture source (both-triggers edge, the only openvr.h
// site). Pure model code holds the seams; the implementations live where
// their dependencies do.
struct IPoseSource
{
    virtual ~IPoseSource() = default;
    virtual bool isInitialized() const = 0;
    virtual std::vector<TrackedDevice> pollPoses() = 0;
    virtual void sendOffsets(const std::vector<DeviceOffset>& offsets) = 0;
};

struct IGestureSource
{
    virtual ~IGestureSource() = default;
    virtual bool isInitialized() const = 0;
    virtual bool bothTriggersJustPressed() = 0;
};

// Result of the per-frame update phase (poll + mode update + record). The
// caller tears down the gesture source when `tearDownGestureSource` is set
// (an action on app-owned state the model cannot own) — every other event
// (capture, recording start/stop/error) is reported through the injected
// `link::Logger` as fully-formatted lines, same as the driver link layer.
struct UpdateResult
{
    std::vector<TrackedDevice> devices;
    FramePlan plan;
    bool tearDownGestureSource = false;  // post-update mode is not Calibration
};

// The head of one frame: picks the device source by mode (Replay pulls from
// the loaded recording via `replay`; otherwise from `poses` when connected,
// empty when not), reads the both-triggers gesture only in Calibration from
// `gesture` (may be null — Calibration before a trigger reader exists),
// advances the mode controller, and runs the recorder. `now` is the
// absolute clock the recorder timestamps frames with (the caller's frame
// time). `replay` is dereferenced only in Replay mode, so it may be null
// when the caller never enters Replay (tests). `logger` receives one
// fully-formatted line per notable event (capture, recording start/stop/
// error); it may have no sink installed (lines dropped, same as link).
//
// This is the shared logic between the visible and the minimized frame
// paths: the driver link must be pumped (pollPoses) and overrides shipped
// every frame whether the window renders or not, so the two paths differ
// only in the render/ImGui tail.
UpdateResult pollAndUpdate(ModeController& controller, IkRig& rig,
                           ReplaySession* replay,
                           SessionRecorder& recorder,
                           IPoseSource& poses,
                           IGestureSource* gesture,
                           double now,
                           link::Logger& logger);

// The tail of one frame: retarget the (already solved) rig pose onto the
// avatar, re-hang trackers at the avatar joints (correctDevicePoses), and
// ship the world-space deltas to the driver when `poses` is connected.
// Returns the corrected poses for rendering / the UI readout. Pure logic —
// the caller draws from the result. Call after the rig pose is final
// (Goals solved, or the gizmo-driven Targets solve in the visible path).
std::vector<CorrectedPose> retargetAndShip(IkRig& rig,
                                           Skeleton& avatar,
                                           const RetargetMap& retargetMap,
                                           const CorrectionMap& correctionMap,
                                           const TrackerCalibration& calibration,
                                           const std::vector<TrackedDevice>& devices,
                                           IPoseSource& poses);
