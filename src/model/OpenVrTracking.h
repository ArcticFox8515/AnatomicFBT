#pragma once

// Pose source for the app, fed by the SteamVR driver link
// (doc/driver-plan.md phase A, step 4). Replaces the old client-API poll:
// poses now arrive over a named pipe as `link::DevicePose` frames (one per
// driver-side TrackedDevicePoseUpdated), folded here into the same
// `TrackedDevice` snapshot the model layer consumes. Trigger input stayed
// on the OpenVR client binding and moved to `OpenVrInput` (src/vr).
//
// The pipe factory is supplied by the exe (system-dependent); the clock only
// paces reconnect attempts (1/s) when the driver is absent, so tests inject a
// fake. The class owns a `link::MessageChannel` and is otherwise model-only.
//
// Observable behavior mirrors the old `OpenVrTracking::pollPoses`: a device is
// in the snapshot only while the driver reports it tracking and of a known
// class (Hmd/Controller/Tracker); the list is ordered by device id, matching
// the old OpenVR-index ascending order. A `Lost` frame removes the device (the
// old code skipped devices whose pose was invalid or that had disconnected).

#include "link/Log.h"
#include "link/Pipe.h"
#include "link/MessageChannel.h"

#include "model/TrackedDevice.h"
#include "model/TrackerCorrection.h"

#include <functional>
#include <vector>

class OpenVrTracking
{
public:
    // `factory` builds an unconnected client pipe (the exe supplies a
    // `Win32ClientPipe` on `kDriverPipeName`); `now` is seconds, used only to
    // throttle reconnect attempts while the driver pipe is absent.
    OpenVrTracking(link::Logger& logger, link::PipeFactoryFn factory,
                   std::function<double()> now);
    ~OpenVrTracking();

    OpenVrTracking(const OpenVrTracking&) = delete;
    OpenVrTracking& operator=(const OpenVrTracking&) = delete;

    // Creates the channel and pumps it once; throws `Error` when the driver
    // pipe is not connected after that first pump (the old `VR_Init`-failed
    // path, so the app's startup try/catch and the calibration-button retry
    // keep working verbatim). Idempotent once connected.
    void init();

    // Currently connected to the driver pipe. A drop flips this to false until
    // the next successful connect (which `pollPoses`/`init` re-attempt).
    bool isInitialized() const;

    // Drains pending frames, folds them into the device list, and returns the
    // current snapshot. While disconnected it re-attempts the connect at most
    // once per second (per the injected clock) and returns an empty list, as
    // the old poll did when uninitialized.
    std::vector<TrackedDevice> pollPoses();

    // Sends one `PoseOverride` frame per device offset to the driver, so the
    // driver can rewrite each device's pose before handing it to SteamVR. The
    // channel is duplex and this class owns the only end of it, so the upstream
    // direction lives here. No-op while disconnected.
    void sendOffsets(const std::vector<DeviceOffset>& offsets);

private:
    void applyPose(const link::DevicePose& pose);

    link::MessageChannel channel_;
    std::function<double()> now_;
    std::vector<TrackedDevice> devices_;
    double nextAttemptAt_ = 0.0;
};