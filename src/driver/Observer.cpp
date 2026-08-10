#include "Observer.h"

#include "Interfaces.h"
#include "Names.h"
#include "PoseMath.h"

#include "link/Convert.h"
#include "link/MessageChannel.h"
#include "link/Protocol.h"

#include <algorithm>
#include <cstring>
#include <utility>

namespace driver
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

Observer::Observer(link::Logger& logger, InterfaceHooks& hooks, NowFn now,
                   link::MessageChannel& channel)
    : log_(logger), hooks_(hooks), now_(std::move(now)), channel_(channel)
{
}

void Observer::setProperties(DeviceProperties* properties)
{
    properties_ = properties;
}

bool Observer::SeenInterfaces::firstTime(const std::string& name)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return names_.insert(name).second;
}

bool Observer::MetadataCache::beginRefresh(double now)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (refreshedEver_ && now - refreshedAt_ < kHousekeepingSeconds)
        return false;
    refreshedAt_ = now;
    refreshedEver_ = true;
    return true;
}

bool Observer::MetadataCache::store(uint32_t index, const Entry& entry)
{
    std::lock_guard<std::mutex> lock(mutex_);
    Entry& current = entries_[index];
    if (current.known && current.deviceClass == entry.deviceClass
        && current.serial == entry.serial && current.isOurs == entry.isOurs)
        return false;
    current = entry;
    return true;
}

Observer::MetadataCache::Entry Observer::MetadataCache::lookup(uint32_t index) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_[index];
}

void Observer::OverrideCache::store(const link::PoseOverride& poseOverride, double now)
{
    if (poseOverride.deviceId >= vr::k_unMaxTrackedDeviceCount)
        return;
    std::lock_guard<std::mutex> lock(mutex_);
    Entry& entry = entries_[poseOverride.deviceId];
    entry.enabled = true;
    entry.delta.pos = link::fromWireVec3<V3>(poseOverride.position);
    entry.delta.rot = link::fromWireQuat<Q>(poseOverride.rotation);
    entry.receivedAt = now;
}

void Observer::OverrideCache::expire(double now)
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (Entry& entry : entries_)
        if (entry.enabled && now - entry.receivedAt > kOverrideStaleSeconds)
            entry.enabled = false;
}

void Observer::OverrideCache::clear()
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (Entry& entry : entries_)
        entry.enabled = false;
}

Observer::OverrideCache::Entry Observer::OverrideCache::lookup(uint32_t index) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_[index];
}

void Observer::onInterfaceRequested(const char* version, void* interfacePtr)
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

bool Observer::onPose(uint32_t index, const vr::DriverPose_t& pose, uint32_t poseStructSize,
                      vr::DriverPose_t& out)
{
    if (index >= vr::k_unMaxTrackedDeviceCount)
        return false;

    if (poseStructSize < sizeof(vr::DriverPose_t))
        return false;

    const MetadataCache::Entry entry = cache_.lookup(index);

    // Our own virtual trackers: do not feed them back to the app (step 7). The app
    // would otherwise see them as tracked devices and could bind calibration targets
    // to them. Also skip override application — the app never sees these indices, so
    // no override should arrive, but defensively return the pose unchanged.
    if (entry.isOurs)
        return false;

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

    // The upstream frame carries the raw world pose. The app derives the
    // correction as `corrected - raw`; feeding it our own corrected output
    // would collapse the offset (and, since the app's IK goals track the raw
    // pose, drift the avatar). Do not touch `wire` with the delta.
    wire.position = link::toWireVec3(world.pos);
    wire.rotation = link::toWireQuat(world.rot);

    const std::size_t n = (std::min)(entry.serial.size(), sizeof(wire.serial) - 1);
    std::memcpy(wire.serial, entry.serial.data(), n);

    link::Message message;
    message.size = sizeof(link::DevicePose);
    message.type = link::MessageType::DevicePose;
    message.devicePose = wire;
    channel_.send(message);

    // Apply the correction to the pose handed to SteamVR. Premultiplying
    // `worldFromDriver` by the delta yields `world' = delta o world`, and the
    // local pose plus all velocities stay in their original frame, so vrserver's
    // pose prediction (which runs in driver-local space) stays exact.
    const OverrideCache::Entry ov = overrides_.lookup(index);
    if (!ov.enabled)
        return false;

    const RigidPose correctedWorldFromDriver = compose(ov.delta, worldFromDriver);
    out = pose;
    out.vecWorldFromDriverTranslation[0] = correctedWorldFromDriver.pos.x;
    out.vecWorldFromDriverTranslation[1] = correctedWorldFromDriver.pos.y;
    out.vecWorldFromDriverTranslation[2] = correctedWorldFromDriver.pos.z;
    out.qWorldFromDriverRotation.w = correctedWorldFromDriver.rot.w;
    out.qWorldFromDriverRotation.x = correctedWorldFromDriver.rot.x;
    out.qWorldFromDriverRotation.y = correctedWorldFromDriver.rot.y;
    out.qWorldFromDriverRotation.z = correctedWorldFromDriver.rot.z;
    return true;
}

void Observer::onRunFrame()
{
    overrides_.expire(now_());

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

        std::string trackingSystem;
        if (properties_->stringProperty(container, vr::Prop_TrackingSystemName_String,
                                        trackingSystem))
            entry.isOurs = (trackingSystem == kOurTrackingSystemName);

        if (cache_.store(i, entry))
            log_.logf("device %u: class=%s(%d) serial=\"%s\" ours=%d container=%llu",
                      i, deviceClassName(entry.deviceClass), entry.deviceClass,
                      entry.serial.c_str(), static_cast<int>(entry.isOurs),
                      static_cast<unsigned long long>(container));
    }
}

void Observer::onMessages(const std::vector<link::Message>& messages)
{
    for (const link::Message& message : messages)
    {
        if (message.type != link::MessageType::PoseOverride)
            continue;
        overrides_.store(message.poseOverride, now_());
    }
}

void Observer::clearOverrides()
{
    overrides_.clear();
}
} // namespace driver
