#include "FrameTick.h"

#include "IkRig.h"
#include "ModeController.h"
#include "Recording.h"
#include "ReplaySession.h"
#include "SessionRecorder.h"

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
                           link::Logger& logger)
{
    UpdateResult result;

    if (controller.mode() == Mode::Replay && replay)
        result.devices = replay->currentDevices();
    else if (poses.isInitialized())
        result.devices = poses.pollPoses();

    bool captureGesture = false;
    if (controller.mode() == Mode::Calibration && gesture && gesture->isInitialized())
        captureGesture = gesture->bothTriggersJustPressed();

    result.plan = controller.update(rig, result.devices, captureGesture);
    // Calibration owns the VR_Init/VR_Shutdown session; the moment the mode
    // is no longer Calibration (manual switch or the automatic capture
    // transition inside update) the trigger reader is torn down by the caller.
    result.tearDownGestureSource = controller.mode() != Mode::Calibration;
    const SessionRecorder::Event recorded =
        recorder.update(controller.mode(), result.plan.capturedOffsets, now, result.devices);
    reportEvents(logger, result.plan.capturedOffsets, recorded);
    return result;
}

std::vector<CorrectedPose> retargetAndShip(IkRig& rig, Skeleton& avatar,
                                           const RetargetMap& retargetMap,
                                           const CorrectionMap& correctionMap,
                                           const TrackerCalibration& calibration,
                                           const std::vector<TrackedDevice>& devices,
                                           IPoseSource& poses)
{
    retargetPose(rig.skeleton, avatar, retargetMap);
    std::vector<CorrectedPose> corrected =
        correctDevicePoses(calibration, correctionMap, avatar, devices);
    if (poses.isInitialized())
        poses.sendOffsets(correctionOffsets(corrected, devices));
    return corrected;
}
