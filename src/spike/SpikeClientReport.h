#pragma once

// Throwaway step-1 spike (doc/driver-plan.md): the client half of the composition
// proof, with the IVRSystem calls behind `ClientPoseSource` so the sampling loop and
// the line formatting run in a unit test (SpikeClient.cpp is then a pure adapter).
//
// openvr.h is deliberately NOT included here: this header is shared with code built
// against openvr_driver.h, and the poses arrive already converted.

#include "SpikeLog.h"
#include "SpikePoseMath.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace spike
{
struct ClientDeviceSample
{
    uint32_t index = 0;
    int deviceClass = 0;
    std::string serial;
    bool poseValid = false;
    int trackingResult = 0;
    // The two universes: raw is what the driver-side composition must reproduce,
    // standing is the app-facing space (they differ by the room calibration).
    RigidPose raw;
    RigidPose standing;
};

class ClientPoseSource
{
public:
    virtual ~ClientPoseSource() = default;

    // Every connected device with a known class, once per call.
    virtual std::vector<ClientDeviceSample> sample() = 0;
};

// Pure: the three log lines for one device, in the driver's format so the two logs
// can be diffed line by line.
std::vector<std::string> formatClientDeviceLines(const ClientDeviceSample& device);

// Pure loop condition: a duration of zero or less means "until Ctrl+C".
bool clientShouldContinue(int durationSeconds, double elapsedSeconds);

// Samples until keepGoing() says stop, logging each device, waiting between passes.
void runClientSampling(ClientPoseSource& source, Logger& logger,
                       const std::function<bool()>& keepGoing,
                       const std::function<void()>& waitBetweenSamples);
} // namespace spike
