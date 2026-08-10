#pragma once

#include "PoseMath.h"

#include "link/Log.h"

#include "link/MessageChannel.h"

#include <openvr_driver.h>

#include <array>
#include <cstdint>
#include <functional>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace driver
{
class DeviceProperties
{
public:
    virtual ~DeviceProperties() = default;

    virtual vr::PropertyContainerHandle_t container(uint32_t deviceIndex) = 0;
    virtual bool stringProperty(vr::PropertyContainerHandle_t container,
                                vr::ETrackedDeviceProperty property, std::string& value) = 0;
    virtual int32_t int32Property(vr::PropertyContainerHandle_t container,
                                  vr::ETrackedDeviceProperty property) = 0;
    // Writes a string property on the given container. Returns true on success. Used by
    // the virtual-tracker emitter to set Manufacturer/ModelNumber/RenderModel/
    // TrackingSystem on a device whose container only exists after Activate assigned it
    // an index (the owner resolves the container via container(index), then writes).
    virtual bool setStringProperty(vr::PropertyContainerHandle_t container,
                                   vr::ETrackedDeviceProperty property, const std::string& value) = 0;
};

class InterfaceHooks
{
public:
    virtual ~InterfaceHooks() = default;

    virtual void hookServerDriverHost(void* host) = 0;
};

using NowFn = std::function<double()>;

constexpr double kHousekeepingSeconds = 1.0;
constexpr double kOverrideStaleSeconds = 0.5;

class Observer
{
public:
    Observer(link::Logger& logger, InterfaceHooks& hooks, NowFn now,
             link::MessageChannel& channel);

    void setProperties(DeviceProperties* properties);

    void onInterfaceRequested(const char* version, void* interfacePtr);
    // Forwards `pose` upstream as a `DevicePose` frame carrying the raw world
    // pose (the app derives the correction as `corrected - raw`, so feeding it
    // our own output would collapse the offset). When an override is active for
    // `index`, also patches `out` so the caller forwards it to SteamVR instead
    // of `pose`. Returns whether the caller should forward `out`.
    bool onPose(uint32_t index, const vr::DriverPose_t& pose, uint32_t poseStructSize,
                vr::DriverPose_t& out);
    void onRunFrame();
    void onMessages(const std::vector<link::Message>& messages);
    void clearOverrides();

private:
    class SeenInterfaces
    {
    public:
        bool firstTime(const std::string& name);

    private:
        std::mutex mutex_;
        std::set<std::string> names_;
    };

    class MetadataCache
    {
    public:
        struct Entry
        {
            bool known = false;
            int deviceClass = vr::TrackedDeviceClass_Invalid;
            std::string serial;
            bool isOurs = false;
        };

        bool beginRefresh(double now);
        bool store(uint32_t index, const Entry& entry);
        Entry lookup(uint32_t index) const;

    private:
        mutable std::mutex mutex_;
        std::array<Entry, vr::k_unMaxTrackedDeviceCount> entries_{};
        double refreshedAt_ = 0.0;
        bool refreshedEver_ = false;
    };

    // Per-device correction deltas received from the app. Written from
    // `onMessages` (RunFrame, vrserver main thread), read from `onPose`
    // (foreign driver pose threads), so the cache has its own mutex like the
    // metadata cache. Entries expire after `kOverrideStaleSeconds` so an app
    // that hangs or crashes stops mangling poses; `clear()` wipes them on a
    // pipe-drop edge (see Server::runFrame).
    class OverrideCache
    {
    public:
        struct Entry
        {
            bool enabled = false;
            RigidPose delta;
            double receivedAt = 0.0;
        };

        void store(const link::PoseOverride& poseOverride, double now);
        void expire(double now);
        void clear();
        Entry lookup(uint32_t index) const;

    private:
        mutable std::mutex mutex_;
        std::array<Entry, vr::k_unMaxTrackedDeviceCount> entries_{};
    };

    link::Logger& log_;
    InterfaceHooks& hooks_;
    NowFn now_;
    link::MessageChannel& channel_;
    SeenInterfaces seen_;
    MetadataCache cache_;
    OverrideCache overrides_;
    DeviceProperties* properties_ = nullptr;
};
} // namespace driver
