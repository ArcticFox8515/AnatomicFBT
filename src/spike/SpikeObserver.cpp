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
// vr::ETrackedDeviceClass -> link::DeviceKind. The link layer is openvr-free, so
// the mapping lives here in the spike (which sees both).
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

void SpikeObserver::onDeviceAdded(const char* serial, int deviceClass)
{
    log_.logf("TrackedDeviceAdded: serial=\"%s\" class=%s(%d)", serial ? serial : "(null)",
              deviceClassName(deviceClass), deviceClass);
}

void SpikeObserver::onPose(uint32_t index, const vr::DriverPose_t& pose, uint32_t poseStructSize)
{
    if (index >= vr::k_unMaxTrackedDeviceCount)
        return;

    const bool sizeOk = poseStructSize >= sizeof(vr::DriverPose_t);

    // --- pose record update (under mutex, as today) ---
    bool firstPose = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        DeviceRecord& device = devices_[index];
        ++device.poseCount;

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
            firstPose = true;
        }
    }

    if (firstPose)
    {
        if (sizeOk)
            log_.logf("first pose update from device %u (poseIsValid=%d connected=%d)", index,
                      static_cast<int>(pose.poseIsValid),
                      static_cast<int>(pose.deviceIsConnected));
        else
            log_.logf("first pose update from device %u (truncated pose struct)", index);
    }

    // A truncated pose struct cannot be converted — its fields beyond the
    // caller's struct size do not exist.
    if (!sizeOk)
        return;

    // --- read cached metadata (via the thread-safe getter) ---
    int deviceClass = vr::TrackedDeviceClass_Invalid;
    std::string serial;
    deviceIdentity(index, deviceClass, serial);

    // --- convert + forward (no mutex on this path) ---
    link::DevicePose wire;
    wire.deviceId = index;
    wire.deviceKind = deviceClassToKind(deviceClass);
    wire.tracking = (pose.poseIsValid && pose.deviceIsConnected)
                        ? link::TrackingState::Tracking
                        : link::TrackingState::Lost;

    // World-space pose = worldFromDriver o local o driverFromHead — the same
    // "B" composition logPoseSamples computes for the A/B comparison. For
    // trackers driverFromHead is identity, so this reduces to A; for the HMD
    // it includes the head transform the app's client-side pose carries.
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

    const std::size_t n = (std::min)(serial.size(), sizeof(wire.serial) - 1);
    std::memcpy(wire.serial, serial.data(), n);

    channel_.send(link::MessageType::DevicePose,
                  reinterpret_cast<const std::uint8_t*>(&wire), sizeof(wire));
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

    // Housekeeping on the very first frame too, so devices show up immediately
    // instead of a second late.
    if (runFrameCount_ == 1 || now - housekeepingAt_ >= kHousekeepingSeconds)
    {
        housekeepingAt_ = now;
        refreshDeviceMetadata();
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
}

bool SpikeObserver::deviceIdentity(uint32_t index, int& deviceClass, std::string& serial)
{
    if (index >= vr::k_unMaxTrackedDeviceCount)
        return false;
    std::lock_guard<std::mutex> lock(mutex_);
    const DeviceRecord& device = devices_[index];
    if (!device.metadataKnown)
        return false;
    deviceClass = device.deviceClass;
    serial = device.serial;
    return true;
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
