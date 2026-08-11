#include "FrameTick.h"

#include "IkRig.h"
#include "ModeController.h"
#include "Recording.h"
#include "ReplaySession.h"
#include "SessionRecorder.h"
#include "VirtualTrackers.h"

#include <algorithm>

namespace
{
void reportEvents(link::Logger& logger, bool capturedOffsets,
                  const SessionRecorder::Event& recorded)
{
    if (capturedOffsets)
        logger.write("Calibration captured; entering capture mode");
    if (recorded.started)
        logger.write((std::string("Recording capture session to ") + kRecordingPath).c_str());
    if (recorded.stopped)
        logger.write((std::string("Recording saved to ") + kRecordingPath).c_str());
    if (!recorded.error.empty())
        logger.write((std::string("Recording failed; stopping it: ") + recorded.error).c_str());
}
} // namespace

UpdateResult pollAndUpdate(ModeController& controller, IkRig& rig,
                           ReplaySession* replay,
                           SessionRecorder& recorder,
                           IPoseSource& poses,
                           IGestureSource* gesture,
                           double now,
                           const std::vector<GripOffset>& grips,
                           link::Logger& logger)
{
    UpdateResult result;

    // The recorder needs the RAW (pre-grip-shift) poses: the recording stores
    // raw frames and writes the grip offsets into the roster on frame 0, so
    // the loaded frames reproduce the live shift. Replay frames are already
    // grip-applied by the loader, so grips are not applied there (passing them
    // would double-shift).
    std::vector<TrackedDevice> raw;
    if (controller.mode() == Mode::Replay && replay)
        result.devices = replay->currentDevices();
    else if (poses.isInitialized())
    {
        raw = poses.pollPoses();
        result.devices = applyGripOffsets(raw, grips);
    }

    bool captureGesture = false;
    if (controller.mode() == Mode::Calibration && gesture && gesture->isInitialized())
        captureGesture = gesture->bothTriggersJustPressed();

    result.plan = controller.update(rig, result.devices, captureGesture);
    // Calibration owns the VR_Init/VR_Shutdown session; the moment the mode
    // is no longer Calibration (manual switch or the automatic capture
    // transition inside update) the trigger reader is torn down by the caller.
    result.tearDownGestureSource = controller.mode() != Mode::Calibration;
    const SessionRecorder::Event recorded =
        recorder.update(controller.mode(), result.plan.capturedOffsets, now, raw, grips);
    reportEvents(logger, result.plan.capturedOffsets, recorded);
    return result;
}

RetargetResult retargetAndShip(IkRig& rig, Skeleton& avatar,
                               const RetargetMap& retargetMap,
                               const CorrectionMap& correctionMap,
                               const TrackerCalibration& calibration,
                               Mode mode,
                               const std::vector<TrackedDevice>& devices,
                               IPoseSource& poses,
                               const std::vector<std::string>& selectedBones)
{
    retargetPose(rig.skeleton, avatar, retargetMap);
    std::vector<CorrectedPose> corrected =
        correctDevicePoses(calibration, correctionMap, avatar, devices);
    if (poses.isInitialized())
        poses.sendOffsets(correctionOffsets(corrected, devices));
    // Virtual tracker markers: ticked eligible bones' avatar poses, computed
    // from the retargeted avatar above. Flows out in the result so the render
    // loop draws them via the same call — no separate wiring to forget.
    // Shipped upstream only in Capture after calibration: no virtual-tracker
    // traffic reaches the driver in ManualPose, Calibration, Replay, or
    // Capture before calibration (doc/virtual-trackers-plan.md step 5).
    std::vector<Pose> virtualTrackers;
    if (!selectedBones.empty())
    {
        const std::vector<std::string> eligible =
            eligibleVirtualTrackerBones(rig, avatar, calibration);
        std::vector<std::string> tickedEligible;
        for (const std::string& bone : selectedBones)
            if (std::find(eligible.begin(), eligible.end(), bone) != eligible.end())
                tickedEligible.push_back(bone);
        std::vector<VirtualTrackerPose> vt =
            computeVirtualTrackerPoses(avatar, tickedEligible);
        virtualTrackers.reserve(vt.size());
        for (const VirtualTrackerPose& p : vt)
            virtualTrackers.push_back(p.pose);
        if (mode == Mode::Capture && calibration.isCalibrated() && poses.isInitialized())
            poses.sendVirtualTrackers(vt);
    }
    return {std::move(corrected), std::move(virtualTrackers)};
}
