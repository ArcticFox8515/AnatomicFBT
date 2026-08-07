// Throwaway step-1 spike (doc/driver-plan.md): the client half of the
// DriverPose_t composition proof. Prints every tracked device's client-side
// TrackingUniverseRaw pose (plus TrackingUniverseStanding for reference) once per
// second, in the same format and to the same log directory as the spike driver, so
// the two logs can be compared line by line while the devices are held still.
//
// Run it with SteamVR up and the spike driver installed, then diff the timestamps:
// whichever of the driver's A / B compositions equals the raw pose here is the
// formula the real driver must use.
//
// ADAPTER ONLY: the sampling loop and the line formatting live in
// SpikeClientReport (unit-tested); everything here is an IVRSystem / Win32
// forwarder.

#include "SpikeClientReport.h"
#include "SpikeLog.h"
#include "SpikeLogFile.h"
#include "SpikePoseMath.h"

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <windows.h>

#include <openvr.h>

#include <cstdio>
#include <cstdlib>
#include <string>

namespace
{
class Win32ProcessApi final : public spike::ProcessApi
{
public:
    unsigned long environmentVariable(const char* name, char* buffer,
                                      unsigned long size) override
    {
        return GetEnvironmentVariableA(name, buffer, size);
    }

    unsigned long executablePath(char* buffer, unsigned long size) override
    {
        return GetModuleFileNameA(nullptr, buffer, size);
    }
};

// spdlog owns the file, the timestamps, the flushing and creating the directory the
// path names; the spike Logger already formats the whole line, hence "%v".
spike::LogSink fileSink(const std::string& path)
{
    auto logger = spdlog::basic_logger_mt("spike-client", path);
    logger->set_pattern("%v");
    return [logger](const char* message) { logger->info(message); };
}

std::string stringProperty(vr::IVRSystem* system, vr::TrackedDeviceIndex_t index,
                          vr::ETrackedDeviceProperty property)
{
    char buffer[512] = {};
    system->GetStringTrackedDeviceProperty(index, property, buffer, sizeof(buffer));
    return buffer;
}

class OpenVrClientSource final : public spike::ClientPoseSource
{
public:
    explicit OpenVrClientSource(vr::IVRSystem* system) : system_(system) {}

    std::vector<spike::ClientDeviceSample> sample() override
    {
        vr::TrackedDevicePose_t raw[vr::k_unMaxTrackedDeviceCount] = {};
        vr::TrackedDevicePose_t standing[vr::k_unMaxTrackedDeviceCount] = {};
        system_->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.0f, raw,
                                               vr::k_unMaxTrackedDeviceCount);
        system_->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0f, standing,
                                               vr::k_unMaxTrackedDeviceCount);

        std::vector<spike::ClientDeviceSample> devices;
        for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
        {
            if (!raw[i].bDeviceIsConnected)
                continue;
            const vr::ETrackedDeviceClass deviceClass = system_->GetTrackedDeviceClass(i);
            if (deviceClass == vr::TrackedDeviceClass_Invalid)
                continue;

            spike::ClientDeviceSample device;
            device.index = i;
            device.deviceClass = deviceClass;
            device.serial = stringProperty(system_, i, vr::Prop_SerialNumber_String);
            device.poseValid = raw[i].bPoseIsValid;
            device.trackingResult = raw[i].eTrackingResult;
            device.raw = spike::poseFromRowMajor34(raw[i].mDeviceToAbsoluteTracking.m);
            device.standing = spike::poseFromRowMajor34(standing[i].mDeviceToAbsoluteTracking.m);
            devices.push_back(std::move(device));
        }
        return devices;
    }

private:
    vr::IVRSystem* system_ = nullptr;
};
} // namespace

int main(int argc, char** argv)
{
    const int durationSeconds = argc > 1 ? std::atoi(argv[1]) : 0;

    spike::Logger& log = spike::log();
    Win32ProcessApi processApi;
    log.setSink(spike::compositeSink(
        fileSink(spike::processLogPath(processApi, spike::kClientLogPrefix)),
        [](const char* message) {
            std::puts(message);
            std::fflush(stdout);
        }));

    vr::EVRInitError initError = vr::VRInitError_None;
    vr::IVRSystem* system = vr::VR_Init(&initError, vr::VRApplication_Background);
    if (!system)
    {
        log.logf("VR_Init failed: %s", vr::VR_GetVRInitErrorAsEnglishDescription(initError));
        return 1;
    }

    log.logf("=== TrackingCorrector spike client ===");
    log.logf("sampling once per second%s; Ctrl+C to stop",
             durationSeconds > 0 ? " for a limited time" : "");

    OpenVrClientSource source(system);
    const DWORD startedAt = GetTickCount();
    spike::runClientSampling(
        source, log,
        [&] {
            const double elapsed = static_cast<double>(GetTickCount() - startedAt) / 1000.0;
            return spike::clientShouldContinue(durationSeconds, elapsed);
        },
        [] { Sleep(1000); });

    vr::VR_Shutdown();
    return 0;
}
