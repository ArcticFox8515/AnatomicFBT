#include "SpikeClientReport.h"

#include "SpikeLog.h"
#include "SpikeNames.h"

#include <cstdio>

namespace spike
{
std::vector<std::string> formatClientDeviceLines(const ClientDeviceSample& device)
{
    char header[256] = {};
    std::snprintf(header, sizeof(header), "dev %u %s \"%s\" valid=%d result=%d", device.index,
                  deviceClassName(device.deviceClass), device.serial.c_str(),
                  static_cast<int>(device.poseValid), device.trackingResult);

    return {header, "     raw              " + formatPose(device.raw),
            "     standing         " + formatPose(device.standing)};
}

bool clientShouldContinue(int durationSeconds, double elapsedSeconds)
{
    return durationSeconds <= 0 || elapsedSeconds < static_cast<double>(durationSeconds);
}

void runClientSampling(ClientPoseSource& source, Logger& logger,
                       const std::function<bool()>& keepGoing,
                       const std::function<void()>& waitBetweenSamples)
{
    while (keepGoing())
    {
        for (const ClientDeviceSample& device : source.sample())
            for (const std::string& line : formatClientDeviceLines(device))
                logger.write(line.c_str());
        waitBetweenSamples();
    }
}
} // namespace spike
