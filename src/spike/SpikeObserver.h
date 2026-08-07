#pragma once

// Throwaway step-1 spike (doc/driver-plan.md): everything the hooks observe.
//
// This is the whole decision/bookkeeping surface of the spike, and it is a plain
// class in a static library rather than a global inside the DLL, so a test can
// construct it and drive every branch directly. Its dependencies on the outside
// world are injected:
//
//   * Logger          — where the lines go,
//   * DeviceProperties — the vr::VRProperties() metadata reads (may be absent),
//   * InterfaceHooks   — the MinHook installs the interface dispatch triggers,
//   * a clock function  — so the 1 s housekeeping and 5 s statistics branches are
//     testable without sleeping.
//
// openvr_driver.h is included for its *types* only (DriverPose_t, handles, enums);
// nothing here calls into vrserver. Button/input capture is NOT part of the driver
// DLL's job: it is handled by a separate background client app, so this observer
// never polls VR events.
//
// Step 3 (doc/driver-plan.md): the observer forwards each pose to its
// `link::MessageChannel` (passed in at construction) so the app can consume
// driver-side poses without polling OpenVR itself. The convert+forward path runs
// with no mutex held — the metadata read (via `deviceIdentity`, which locks
// internally) happens first, then the pose is converted to the wire POD and
// handed to the channel outside the lock.

#include "SpikeLog.h"

#include "link/MessageChannel.h"

#include <openvr_driver.h>

#include <array>
#include <cstdint>
#include <functional>
#include <mutex>
#include <set>
#include <string>

namespace spike
{
// Seam over vr::VRProperties(): the metadata reads are the only thing the observer
// needs *from* vrserver, and their failure paths have to be reachable from a test.
class DeviceProperties
{
public:
    virtual ~DeviceProperties() = default;

    virtual vr::PropertyContainerHandle_t container(uint32_t deviceIndex) = 0;
    // False when the container holds no such device (yet).
    virtual bool stringProperty(vr::PropertyContainerHandle_t container,
                                vr::ETrackedDeviceProperty property, std::string& value) = 0;
    virtual int32_t int32Property(vr::PropertyContainerHandle_t container,
                                  vr::ETrackedDeviceProperty property) = 0;
};

// Seam over the vtable detour installs.
class InterfaceHooks
{
public:
    virtual ~InterfaceHooks() = default;

    virtual void hookServerDriverHost(void* host) = 0;
};

// Monotonic seconds; any origin.
using NowFn = std::function<double()>;

constexpr double kHousekeepingSeconds = 1.0;
constexpr double kStatsSeconds = 5.0;

class SpikeObserver
{
public:
    SpikeObserver(Logger& logger, InterfaceHooks& hooks, NowFn now,
                  link::MessageChannel& channel);

    // Available only once the driver context is initialized; nullptr means "no
    // metadata this frame".
    void setProperties(DeviceProperties* properties);

    // ---- hook thread entry points (never block, never throw out) ----

    void onInterfaceRequested(const char* version, void* interfacePtr);
    void onDeviceAdded(const char* serial, int deviceClass);
    void onPose(uint32_t index, const vr::DriverPose_t& pose, uint32_t poseStructSize);

    // ---- main thread (Init / RunFrame / Cleanup) ----

    void onInit();
    void onRunFrame();
    void onCleanup();

    // Thread-safe metadata getter (locks `mutex_` internally). Returns false
    // when the device's metadata is not known yet; the caller gets the cached
    // class + serial otherwise.
    bool deviceIdentity(uint32_t index, int& deviceClass, std::string& serial);

private:
    struct DeviceRecord
    {
        bool poseSeen = false;
        uint64_t poseCount = 0;
        uint64_t poseCountAtStats = 0;
        vr::DriverPose_t lastPose{};
        bool lastPoseValid = false;

        vr::PropertyContainerHandle_t container = vr::k_ulInvalidPropertyContainer;
        bool metadataKnown = false;
        int deviceClass = vr::TrackedDeviceClass_Invalid;
        int roleHint = vr::TrackedControllerRole_Invalid;
        std::string serial;
        std::string model;
        std::string trackingSystem;
    };

    void refreshDeviceMetadata();
    void logPoseSamples();
    void logRates(double elapsed);

    Logger& log_;
    InterfaceHooks& hooks_;
    NowFn now_;
    link::MessageChannel& channel_;

    std::mutex mutex_;
    DeviceProperties* properties_ = nullptr;
    std::array<DeviceRecord, vr::k_unMaxTrackedDeviceCount> devices_{};
    std::set<std::string> interfaces_;
    bool poseSizeWarned_ = false;

    // Main thread only (onInit / onRunFrame / onCleanup), hence unlocked.
    uint64_t runFrameCount_ = 0;
    uint64_t runFrameCountAtStats_ = 0;
    double startedAt_ = 0.0;
    double housekeepingAt_ = 0.0;
    double statsAt_ = 0.0;
};
} // namespace spike
