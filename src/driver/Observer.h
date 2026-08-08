#pragma once

#include "link/Log.h"

#include "link/MessageChannel.h"

#include <openvr_driver.h>

#include <array>
#include <cstdint>
#include <functional>
#include <mutex>
#include <set>
#include <string>

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
};

class InterfaceHooks
{
public:
    virtual ~InterfaceHooks() = default;

    virtual void hookServerDriverHost(void* host) = 0;
};

using NowFn = std::function<double()>;

constexpr double kHousekeepingSeconds = 1.0;

class Observer
{
public:
    Observer(link::Logger& logger, InterfaceHooks& hooks, NowFn now,
             link::MessageChannel& channel);

    void setProperties(DeviceProperties* properties);

    void onInterfaceRequested(const char* version, void* interfacePtr);
    void onPose(uint32_t index, const vr::DriverPose_t& pose, uint32_t poseStructSize);
    void onRunFrame();

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

    link::Logger& log_;
    InterfaceHooks& hooks_;
    NowFn now_;
    link::MessageChannel& channel_;
    SeenInterfaces seen_;
    MetadataCache cache_;
    DeviceProperties* properties_ = nullptr;
};
} // namespace driver
