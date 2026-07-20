#include "ModeController.h"

#include "IkRig.h"

FramePlan ModeController::update(IkRig& rig, const std::vector<TrackedDevice>& devices,
                                 bool captureGesture)
{
    FramePlan plan;

    switch (mode_)
    {
    case Mode::ManualPose:
        plan.solve = SolveMode::Targets;
        break;

    case Mode::Calibration:
    {
        const CalibrationFrame frame = updateCalibrationFrame(rig, devices);
        liveAssignment_ = frame.assignment;
        if (!captureGesture)
            break;
        captureOffsets(calibration_, frame, devices);
        mode_ = Mode::Capture;
        plan.capturedOffsets = true;
        // Fall straight into capture handling so the transition frame already
        // produces fresh goals — solve(plan.goals) must never see an empty or
        // stale vector.
        [[fallthrough]];
    }

    case Mode::Capture:
    case Mode::Replay:
        plan.goals = updateCaptureFrame(rig, calibration_, devices);
        plan.solve = SolveMode::Goals;
        break;
    }

    return plan;
}

void ModeController::calibrateFromFrame(IkRig& rig, const std::vector<TrackedDevice>& devices)
{
    calibration_.clear();
    const CalibrationFrame frame = updateCalibrationFrame(rig, devices);
    liveAssignment_ = frame.assignment;
    captureOffsets(calibration_, frame, devices);
}
