#pragma once

// Throwaway step-1 spike (doc/driver-plan.md): everything the hooks observe.
//
// This is the whole decision/bookkeeping surface of the spike, and it is a plain
// class in a static library rather than a global inside the DLL, so a test can
// construct it and drive every branch directly. Its three dependencies on the
// outside world are injected:
//
//   * Logger        — where the lines go,
//   * DeviceProperties — the vr::VRProperties() metadata reads (may be absent),
//   * InterfaceHooks   — the MinHook installs the interface dispatch triggers,
//   * a clock function — so the 1 s housekeeping and 5 s statistics branches are
//     testable without sleeping.
//
// openvr_driver.h is included for its *types* only (DriverPose_t, handles, enums);
// nothing here calls into vrserver.

#include "SpikeLog.h"

#include <openvr_driver.h>

#include <array>
#include <cstdint>
#include <functional>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

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
    virtual void hookDriverInput(void* input) = 0;
};

// Monotonic seconds; any origin.
using NowFn = std::function<double()>;

constexpr double kHousekeepingSeconds = 1.0;
constexpr double kStatsSeconds = 5.0;

class SpikeObserver
{
public:
    SpikeObserver(Logger& logger, InterfaceHooks& hooks, NowFn now);

    // Available only once the driver context is initialized; nullptr means "no
    // metadata this frame".
    void setProperties(DeviceProperties* properties);

    // ---- hook thread entry points (never block, never throw out) ----

    void onInterfaceRequested(const char* version, void* interfacePtr);
    void onDeviceAdded(const char* serial, int deviceClass);
    void onPose(uint32_t index, const vr::DriverPose_t& pose, uint32_t poseStructSize);
    void onBooleanComponentCreated(vr::PropertyContainerHandle_t container, const char* name,
                                   vr::VRInputComponentHandle_t handle);
    void onScalarComponentCreated(vr::PropertyContainerHandle_t container, const char* name,
                                  vr::VRInputComponentHandle_t handle);
    void onBooleanComponentUpdated(vr::VRInputComponentHandle_t handle, bool value);

    // ---- main thread (Init / RunFrame / Cleanup) ----

    void onInit();
    void onRunFrame();
    void onCleanup();

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

    struct ComponentRecord
    {
        vr::VRInputComponentHandle_t handle = vr::k_ulInvalidInputComponentHandle;
        vr::PropertyContainerHandle_t container = vr::k_ulInvalidPropertyContainer;
        std::string name;
        bool trigger = false;
        int deviceIndex = -1;
        bool valueKnown = false;
        bool value = false;
        uint64_t updates = 0;
    };

    void refreshDeviceMetadata();
    void resolveComponents();
    void logPoseSamples();
    void logRates(double elapsed);

    Logger& log_;
    InterfaceHooks& hooks_;
    NowFn now_;

    std::mutex mutex_;
    DeviceProperties* properties_ = nullptr;
    std::array<DeviceRecord, vr::k_unMaxTrackedDeviceCount> devices_{};
    std::vector<ComponentRecord> components_;
    std::unordered_map<uint64_t, size_t> componentByHandle_;
    std::set<std::string> interfaces_;
    bool poseSizeWarned_ = false;
    uint64_t unknownBooleanUpdates_ = 0;
    uint64_t unknownBooleanUpdatesAtStats_ = 0;

    // Main thread only (onInit / onRunFrame / onCleanup), hence unlocked.
    uint64_t runFrameCount_ = 0;
    uint64_t runFrameCountAtStats_ = 0;
    double startedAt_ = 0.0;
    double housekeepingAt_ = 0.0;
    double statsAt_ = 0.0;
};
} // namespace spike
