#include "SpikeObserver.h"

#include "SpikeInterfaces.h"
#include "SpikeNames.h"
#include "SpikePoseMath.h"

#include <utility>

namespace spike
{
SpikeObserver::SpikeObserver(Logger& logger, InterfaceHooks& hooks, NowFn now)
    : log_(logger), hooks_(hooks), now_(std::move(now))
{
}

void SpikeObserver::setProperties(DeviceProperties* properties)
{
    std::lock_guard<std::mutex> lock(mutex_);
    properties_ = properties;
}

void SpikeObserver::onInterfaceRequested(const char* version, void* interfacePtr)
{
    const std::string name = version ? version : "(null)";
    {
        // Once per version string: vrserver hands the same interface out to every
        // driver, and the log must stay readable.
        std::lock_guard<std::mutex> lock(mutex_);
        if (!interfaces_.insert(name).second)
            return;
    }

    if (!interfacePtr)
    {
        log_.logf("interface \"%s\": requested, vrserver returned NULL", name.c_str());
        return;
    }

    switch (classifyInterface(name, vr::IVRServerDriverHost_Version, vr::IVRDriverInput_Version))
    {
    case InterfaceAction::HookServerDriverHost:
        log_.logf("interface \"%s\": hooking", name.c_str());
        hooks_.hookServerDriverHost(interfacePtr);
        break;
    case InterfaceAction::HookDriverInput:
        log_.logf("interface \"%s\": hooking", name.c_str());
        hooks_.hookDriverInput(interfacePtr);
        break;
    case InterfaceAction::UnsupportedVersion:
        log_.logf("interface \"%s\": NOT HOOKED — version differs from the one we build "
                  "against (%s / %s). Devices using it would be invisible to us.",
                  name.c_str(), vr::IVRServerDriverHost_Version, vr::IVRDriverInput_Version);
        break;
    case InterfaceAction::NotNeeded:
        log_.logf("interface \"%s\": seen, not hooked (not needed)", name.c_str());
        break;
    }
}

void SpikeObserver::onDeviceAdded(const char* serial, int deviceClass)
{
    log_.logf("TrackedDeviceAdded: serial=\"%s\" class=%s(%d)", serial ? serial : "(null)",
              deviceClassName(deviceClass), deviceClass);
}

void SpikeObserver::onPose(uint32_t index, const vr::DriverPose_t& pose, uint32_t poseStructSize)
{
    if (index >= vr::k_unMaxTrackedDeviceCount)
        return;

    std::lock_guard<std::mutex> lock(mutex_);
    DeviceRecord& device = devices_[index];
    ++device.poseCount;

    const bool sizeOk = poseStructSize >= sizeof(vr::DriverPose_t);
    if (sizeOk)
    {
        device.lastPose = pose;
        device.lastPoseValid = true;
    }
    else if (!poseSizeWarned_)
    {
        poseSizeWarned_ = true;
        log_.logf("TrackedDevicePoseUpdated: unPoseStructSize=%u < sizeof(DriverPose_t)=%zu — "
                  "pose contents NOT read for such callers",
                  poseStructSize, sizeof(vr::DriverPose_t));
    }

    if (!device.poseSeen)
    {
        device.poseSeen = true;
        if (sizeOk)
            log_.logf("first pose update from device %u (poseIsValid=%d connected=%d)", index,
                      static_cast<int>(pose.poseIsValid),
                      static_cast<int>(pose.deviceIsConnected));
        else
            log_.logf("first pose update from device %u (truncated pose struct)", index);
    }
}

void SpikeObserver::onBooleanComponentCreated(vr::PropertyContainerHandle_t container,
                                              const char* name,
                                              vr::VRInputComponentHandle_t handle)
{
    ComponentRecord record;
    record.handle = handle;
    record.container = container;
    record.name = name ? name : "(null)";
    record.trigger = isTriggerClick(record.name);

    std::lock_guard<std::mutex> lock(mutex_);
    componentByHandle_[handle] = components_.size();
    components_.push_back(record);
    log_.logf("CreateBooleanComponent: container=%llu handle=%llu name=\"%s\"%s",
              static_cast<unsigned long long>(container),
              static_cast<unsigned long long>(handle), record.name.c_str(),
              record.trigger ? " <-- trigger click" : "");
}

void SpikeObserver::onScalarComponentCreated(vr::PropertyContainerHandle_t container,
                                             const char* name,
                                             vr::VRInputComponentHandle_t handle)
{
    // Not hooked for updates — logged only to see whether a controller exposes a
    // scalar-only trigger (the documented fallback in doc/driver-plan.md).
    log_.logf("CreateScalarComponent: container=%llu handle=%llu name=\"%s\"",
              static_cast<unsigned long long>(container),
              static_cast<unsigned long long>(handle), name ? name : "(null)");
}

void SpikeObserver::onBooleanComponentUpdated(vr::VRInputComponentHandle_t handle, bool value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = componentByHandle_.find(handle);
    if (it == componentByHandle_.end())
    {
        // A component created before our hook went in — i.e. a coverage gap.
        ++unknownBooleanUpdates_;
        return;
    }

    ComponentRecord& component = components_[it->second];
    ++component.updates;
    const bool changed = !component.valueKnown || component.value != value;
    component.valueKnown = true;
    component.value = value;

    if (component.trigger && changed)
        log_.logf("trigger %s: device %d (%s) component \"%s\"", value ? "DOWN" : "up  ",
                  component.deviceIndex,
                  component.deviceIndex >= 0 ? devices_[component.deviceIndex].serial.c_str()
                                             : "?",
                  component.name.c_str());
}

void SpikeObserver::onInit()
{
    const double now = now_();
    startedAt_ = now;
    housekeepingAt_ = now;
    statsAt_ = now;
}

void SpikeObserver::onRunFrame()
{
    ++runFrameCount_;
    const double now = now_();

    if (runFrameCount_ == 1)
        log_.logf("first RunFrame call");

    // Housekeeping on the very first frame too, so devices and components show up
    // immediately instead of a second late.
    if (runFrameCount_ == 1 || now - housekeepingAt_ >= kHousekeepingSeconds)
    {
        housekeepingAt_ = now;
        refreshDeviceMetadata();
        resolveComponents();
        logPoseSamples();
    }

    if (now - statsAt_ >= kStatsSeconds)
    {
        const double elapsed = now - statsAt_;
        statsAt_ = now;
        logRates(elapsed);
    }
}

void SpikeObserver::onCleanup()
{
    log_.logf("summary: %llu RunFrame calls in %.1f s",
              static_cast<unsigned long long>(runFrameCount_), now_() - startedAt_);

    std::lock_guard<std::mutex> lock(mutex_);
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
    {
        const DeviceRecord& device = devices_[i];
        if (device.poseCount == 0 && !device.metadataKnown)
            continue;
        log_.logf("summary: device %u %s \"%s\": %llu pose updates", i,
                  deviceClassName(device.deviceClass), device.serial.c_str(),
                  static_cast<unsigned long long>(device.poseCount));
    }
    for (const ComponentRecord& component : components_)
        log_.logf("summary: component \"%s\" device=%d updates=%llu", component.name.c_str(),
                  component.deviceIndex, static_cast<unsigned long long>(component.updates));
}

// Proves that a driver can enumerate *any* device's metadata by index, without ever
// having implemented ITrackedDeviceServerDriver for it.
void SpikeObserver::refreshDeviceMetadata()
{
    DeviceProperties* properties = nullptr;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        properties = properties_;
    }
    if (!properties)
        return;

    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
    {
        const vr::PropertyContainerHandle_t container = properties->container(i);
        if (container == vr::k_ulInvalidPropertyContainer)
            continue;

        std::string serial;
        if (!properties->stringProperty(container, vr::Prop_SerialNumber_String, serial))
            continue; // No such device (yet).

        std::lock_guard<std::mutex> lock(mutex_);
        DeviceRecord& device = devices_[i];
        if (device.metadataKnown && device.serial == serial)
            continue;

        device.container = container;
        device.serial = serial;
        device.metadataKnown = true;
        device.deviceClass = properties->int32Property(container, vr::Prop_DeviceClass_Int32);
        device.roleHint =
            properties->int32Property(container, vr::Prop_ControllerRoleHint_Int32);
        properties->stringProperty(container, vr::Prop_ModelNumber_String, device.model);
        properties->stringProperty(container, vr::Prop_TrackingSystemName_String,
                                   device.trackingSystem);

        log_.logf("device %u: class=%s(%d) role=%s serial=\"%s\" model=\"%s\" "
                  "trackingSystem=\"%s\" container=%llu",
                  i, deviceClassName(device.deviceClass), device.deviceClass,
                  roleHintName(device.roleHint), device.serial.c_str(), device.model.c_str(),
                  device.trackingSystem.c_str(), static_cast<unsigned long long>(container));
    }
}

// container -> device index, the mapping the real driver needs to attribute a boolean
// component to a device.
void SpikeObserver::resolveComponents()
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (ComponentRecord& component : components_)
    {
        if (component.deviceIndex >= 0)
            continue;
        for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
        {
            if (!devices_[i].metadataKnown || devices_[i].container != component.container)
                continue;
            component.deviceIndex = static_cast<int>(i);
            log_.logf("component \"%s\" resolved to device %u (\"%s\")", component.name.c_str(),
                      i, devices_[i].serial.c_str());
            break;
        }
    }
}

// The point of the whole spike: both candidate compositions, side by side, to be
// compared against spike_client's TrackingUniverseRaw pose for the same device.
void SpikeObserver::logPoseSamples()
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
    {
        const DeviceRecord& device = devices_[i];
        if (!device.lastPoseValid)
            continue;

        const vr::DriverPose_t& pose = device.lastPose;

        log_.logf("pose dev %u %s \"%s\" valid=%d connected=%d result=%d timeOffset=%.5f", i,
                  deviceClassName(device.deviceClass), device.serial.c_str(),
                  static_cast<int>(pose.poseIsValid), static_cast<int>(pose.deviceIsConnected),
                  static_cast<int>(pose.result), pose.poseTimeOffset);

        // `lastPoseValid` means "a pose struct was stored", not that SteamVR vouches
        // for it. Standby drops poseIsValid to 0 while still sending poses 80 s stale;
        // base stations send an all-zero quaternion with valid=0; the Tundra Tracker
        // reported 9 km with valid=1. Composing those produces numbers, not
        // information, in the same format as the A/B lines the spike exists to
        // compare — so gate the composition on poseIsValid and keep only the header
        // (which is how standby staleness became visible at all).
        if (!pose.poseIsValid)
            continue;

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
        const RigidPose a = compose(worldFromDriver, local);
        const RigidPose b = compose(a, driverFromHead);

        log_.logf("     local            %s", formatPose(local).c_str());
        log_.logf("     worldFromDriver  %s", formatPose(worldFromDriver).c_str());
        log_.logf("     driverFromHead   %s", formatPose(driverFromHead).c_str());
        log_.logf("     A = wFd o local  %s", formatPose(a).c_str());
        log_.logf("     B = A o dFh      %s", formatPose(b).c_str());
    }
}

void SpikeObserver::logRates(double elapsed)
{
    log_.logf("RunFrame: %.1f Hz (%llu calls total)",
              static_cast<double>(runFrameCount_ - runFrameCountAtStats_) / elapsed,
              static_cast<unsigned long long>(runFrameCount_));
    runFrameCountAtStats_ = runFrameCount_;

    std::lock_guard<std::mutex> lock(mutex_);
    if (unknownBooleanUpdates_ != unknownBooleanUpdatesAtStats_)
    {
        log_.logf("UpdateBooleanComponent: %llu updates for components created before our "
                  "hook was installed (invisible to us)",
                  static_cast<unsigned long long>(unknownBooleanUpdates_
                                                  - unknownBooleanUpdatesAtStats_));
        unknownBooleanUpdatesAtStats_ = unknownBooleanUpdates_;
    }

    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
    {
        DeviceRecord& device = devices_[i];
        if (device.poseCount == device.poseCountAtStats)
            continue;
        log_.logf("pose rate: device %u %s \"%s\": %.1f Hz (%llu total)", i,
                  deviceClassName(device.deviceClass), device.serial.c_str(),
                  static_cast<double>(device.poseCount - device.poseCountAtStats) / elapsed,
                  static_cast<unsigned long long>(device.poseCount));
        device.poseCountAtStats = device.poseCount;
    }
}
} // namespace spike
