#pragma once

#include "IkSolvers.h"
#include "TrackedDevice.h"
#include "TrackerCalibration.h"

#include <vector>

class IkRig;

// Application mode: ManualPose = gizmo-dragged targets (no VR input);
// Calibration = targets mirror raw device poses while the skeleton rests,
// until both triggers freeze device->target offsets; Capture = offsets
// applied every frame, IK solver active; Replay = identical to Capture,
// except the caller feeds recorded device snapshots (calibrated once from
// the recording's first frame via calibrateFromFrame) instead of live ones.
enum class Mode
{
    ManualPose,
    Calibration,
    Capture,
    Replay
};

// What the render loop should do with the rig after ModeController::update.
enum class SolveMode
{
    None,     // leave the pose as update left it (Calibration)
    Targets,  // call IkRig::solve() on the (gizmo-edited) targets (ManualPose)
    Goals     // call IkRig::solve(goals) (Capture)
};

// Per-frame output of ModeController::update.
struct FramePlan
{
    SolveMode solve = SolveMode::None;
    std::vector<IkTarget> goals;  // valid when solve == Goals; always parallels
                                  // the rig's targets, freshly derived this frame
    bool capturedOffsets = false;  // true on the frame calibration froze (for logging)
};

// The ManualPose/Calibration/Capture state machine, hardware-free: the caller
// feeds device snapshots and the calibration gesture per frame, and executes
// the returned FramePlan. Owns the TrackerCalibration and the live assignment
// (for UI). The central invariant: whenever the plan says Goals, those goals
// were produced from the rig's current targets on THIS frame — including the
// calibration->capture transition frame — so IkRig::solve(goals) can never
// receive a stale or wrongly-sized vector.
class ModeController
{
public:
    explicit ModeController(Mode initial = Mode::ManualPose) : mode_(initial) {}

    Mode mode() const { return mode_; }

    void switchToManual() { mode_ = Mode::ManualPose; }

    // (Re-)enters calibration: drops any previously captured offsets.
    void switchToCalibration()
    {
        calibration_.clear();
        mode_ = Mode::Calibration;
    }

    // Enters replay: drops any previously captured offsets. The caller then
    // calibrates from a recording's first frame (calibrateFromFrame) and
    // feeds recorded device snapshots to update — the solver path is exactly
    // the Capture one.
    void switchToReplay()
    {
        calibration_.clear();
        mode_ = Mode::Replay;
    }

    // Calibrates from one device snapshot exactly as the live gesture path
    // does (rest pose with the root placed from the HMD and the T-pose hand
    // landmark, proximity assignment, frozen offsets) — given a recording's
    // first frame this reproduces the live session's offsets bit for bit.
    // Mutates the rig like a calibration frame; does not change the mode.
    void calibrateFromFrame(IkRig& rig, const std::vector<TrackedDevice>& devices);

    // Live device->target assignment from the last calibration frame (for UI;
    // device indices are positions in the device list passed to update).
    const DeviceAssignment& liveAssignment() const { return liveAssignment_; }

    // The calibration (offsets, bindings) the controller owns. Read by the
    // app to compute corrected device poses from the retargeted avatar
    // skeleton (see TrackerCorrection); never written outside update.
    const TrackerCalibration& calibration() const { return calibration_; }

    // Advances the state machine by one frame. `devices` is the latest
    // snapshot list (empty when tracking is unavailable); `captureGesture` is
    // the both-triggers edge (only meaningful in Calibration, ignored
    // otherwise). Mutates the rig as the mode requires and returns what the
    // render loop should solve.
    FramePlan update(IkRig& rig, const std::vector<TrackedDevice>& devices, bool captureGesture);

private:
    Mode mode_;
    TrackerCalibration calibration_;
    DeviceAssignment liveAssignment_;
};
