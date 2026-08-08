#include "SpikeObserver.h"

#include "SpikeInterfaces.h"
#include "SpikeNames.h"
#include "SpikePoseMath.h"

#include "link/MessageChannel.h"
#include "link/Protocol.h"

#include <algorithm>
#include <cstring>
#include <utility>

namespace spike
{
namespace
{
link::DeviceKind deviceClassToKind(int deviceClass)
{
    switch (deviceClass)
    {
    case vr::TrackedDeviceClass_HMD: return link::DeviceKind::Hmd;
    case vr::TrackedDeviceClass_Controller: return link::DeviceKind::Controller;
    case vr::TrackedDeviceClass_GenericTracker: return link::DeviceKind::Tracker;
    default: return link::DeviceKind::Other;
    }
}
} // namespace

SpikeObserver::SpikeObserver(Logger& logger, InterfaceHooks& hooks, NowFn now,
                             link::MessageChannel& channel)
    : log_(logger), hooks_(hooks), now_(std::move(now)), channel_(channel)
{
}

void SpikeObserver::setProperties(DeviceProperties* properties)
{
    properties_ = properties;
}

bool SpikeObserver::SeenInterfaces::firstTime(const std::string& name)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return names_.insert(name).second;
}

bool SpikeObserver::MetadataCache::beginRefresh(double now)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (refreshedEver_ && now - refreshedAt_ < kHousekeepingSeconds)
        return false;
    refreshedAt_ = now;
    refreshedEver_ = true;
    return true;
}

bool SpikeObserver::MetadataCache::store(uint32_t index, const Entry& entry)
{
    std::lock_guard<std::mutex> lock(mutex_);
    Entry& current = entries_[index];
    if (current.known && current.deviceClass == entry.deviceClass && current.serial == entry.serial)
        return false;
    current = entry;
    return true;
}

SpikeObserver::MetadataCache::Entry SpikeObserver::MetadataCache::lookup(uint32_t index) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_[index];
}

void SpikeObserver::onInterfaceRequested(const char* version, void* interfacePtr)
{
    const std::string name = version ? version : "(null)";
    if (!seen_.firstTime(name))
        return;

    if (!interfacePtr)
    {
        log_.logf("interface \"%s\": requested, vrserver returned NULL", name.c_str());
        return;
    }

    switch (classifyInterface(name, vr::IVRServerDriverHost_Version))
    {
    case InterfaceAction::HookServerDriverHost:
        log_.logf("interface \"%s\": hooking", name.c_str());
        hooks_.hookServerDriverHost(interfacePtr);
        break;
    case InterfaceAction::UnsupportedVersion:
        log_.logf("interface \"%s\": NOT HOOKED — version differs from the one we build "
                  "against (%s). Devices using it would be invisible to us.",
                  name.c_str(), vr::IVRServerDriverHost_Version);
        break;
    case InterfaceAction::NotNeeded:
        log_.logf("interface \"%s\": seen, not hooked (not needed)", name.c_str());
        break;
    }
}

void SpikeObserver::onPose(uint32_t index, const vr::DriverPose_t& pose, uint32_t poseStructSize)
{
    if (index >= vr::k_unMaxTrackedDeviceCount)
        return;

    if (poseStructSize < sizeof(vr::DriverPose_t))
        return;

    const MetadataCache::Entry entry = cache_.lookup(index);

    link::DevicePose wire;
    wire.deviceId = index;
    wire.deviceKind = deviceClassToKind(entry.deviceClass);
    wire.tracking = (pose.poseIsValid && pose.deviceIsConnected)
                        ? link::TrackingState::Tracking
                        : link::TrackingState::Lost;

    const RigidPose worldFromDriver{{pose.vecWorldFromDriverTranslation[0],
                                     pose.vecWorldFromDriverTranslation[1],
                                     pose.vecWorldFromDriverTranslation[2]},
                                    {pose.qWorldFromDriverRotation.w,
                                     pose.qWorldFromDriverRotation.x,
                                     pose.qWorldFromDriverRotation.y,
                                     pose.qWorldFromDriverRotation.z}};
    const RigidPose local{{pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2]},
                          {pose.qRotation.w, pose.qRotation.x, pose.qRotation.y,
                           pose.qRotation.z}};
    const RigidPose driverFromHead{{pose.vecDriverFromHeadTranslation[0],
                                    pose.vecDriverFromHeadTranslation[1],
                                    pose.vecDriverFromHeadTranslation[2]},
                                   {pose.qDriverFromHeadRotation.w,
                                    pose.qDriverFromHeadRotation.x,
                                    pose.qDriverFromHeadRotation.y,
                                    pose.qDriverFromHeadRotation.z}};
    const RigidPose world = compose(compose(worldFromDriver, local), driverFromHead);

    wire.position[0] = static_cast<float>(world.pos.x);
    wire.position[1] = static_cast<float>(world.pos.y);
    wire.position[2] = static_cast<float>(world.pos.z);
    wire.rotation[0] = static_cast<float>(world.rot.x);
    wire.rotation[1] = static_cast<float>(world.rot.y);
    wire.rotation[2] = static_cast<float>(world.rot.z);
    wire.rotation[3] = static_cast<float>(world.rot.w);

    const std::size_t n = (std::min)(entry.serial.size(), sizeof(wire.serial) - 1);
    std::memcpy(wire.serial, entry.serial.data(), n);

    link::Message message;
    message.size = sizeof(link::DevicePose);
    message.type = link::MessageType::DevicePose;
    message.pose = wire;
    channel_.send(message);
}

void SpikeObserver::onRunFrame()
{
    if (!properties_)
        return;

    if (!cache_.beginRefresh(now_()))
        return;

    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
    {
        const vr::PropertyContainerHandle_t container = properties_->container(i);
        if (container == vr::k_ulInvalidPropertyContainer)
            continue;

        std::string serial;
        if (!properties_->stringProperty(container, vr::Prop_SerialNumber_String, serial))
            continue;

        MetadataCache::Entry entry;
        entry.known = true;
        entry.serial = serial;
        entry.deviceClass = properties_->int32Property(container, vr::Prop_DeviceClass_Int32);

        if (cache_.store(i, entry))
            log_.logf("device %u: class=%s(%d) serial=\"%s\" container=%llu",
                      i, deviceClassName(entry.deviceClass), entry.deviceClass,
                      entry.serial.c_str(),
                      static_cast<unsigned long long>(container));
    }
}
} // namespace spike
