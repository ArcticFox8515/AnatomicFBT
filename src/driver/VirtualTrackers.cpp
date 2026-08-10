#include "VirtualTrackers.h"

#include "Guard.h"
#include "Names.h"

#include "link/Convert.h"
#include "link/Protocol.h"

#include <cstring>
#include <memory>
#include <utility>

namespace driver
{
namespace
{
// The values written to the four string properties on every virtual tracker. The
// render model is the Vive tracker so SteamVR renders something familiar; the
// tracking system name is the shared constant (kOurTrackingSystemName in Names.h)
// so the Observer can filter our own devices out of the downstream stream (step 7).
constexpr const char* kManufacturer = "TrackingCorrector";
constexpr const char* kModelNumber = "TrackingCorrector Virtual Tracker";
constexpr const char* kRenderModelName = "{htc}/rendermodels/vr_tracker_vive_1_0";

// Identity quaternion as DriverPose_t expects it: {w=1, x=0, y=0, z=0}.
vr::HmdQuaternion_t identityQuaternion()
{
    return {1.0, 0.0, 0.0, 0.0};
}
} // namespace

// ---- serial ----

std::string serialForBone(const std::string& boneName)
{
    return "TC-" + boneName;
}

// ---- device ----

VirtualTracker::VirtualTracker(const std::string& serial, const std::string& boneName)
    : serial_(serial), boneName_(boneName)
{
}

void VirtualTracker::setPose(const vr::DriverPose_t& pose)
{
    pose_.store(pose);
}

vr::EVRInitError VirtualTracker::Activate(uint32_t unObjectId)
{
    try
    {
        deviceIndex_.store(unObjectId);
        return vr::VRInitError_None;
    }
    catch (...)
    {
        return vr::VRInitError_Driver_Failed;
    }
}

void VirtualTracker::Deactivate()
{
    runGuarded([&] { deviceIndex_.store(vr::k_unTrackedDeviceIndexInvalid); });
}

void VirtualTracker::EnterStandby()
{
    runGuarded([&] {});
}

void* VirtualTracker::GetComponent(const char* /*pchComponentNameAndVersion*/)
{
    return nullptr;
}

void VirtualTracker::DebugRequest(const char* /*pchRequest*/, char* /*pchResponseBuffer*/,
                                  uint32_t /*unResponseBufferSize*/)
{
    runGuarded([&] {});
}

vr::DriverPose_t VirtualTracker::GetPose()
{
    return pose_.load();
}

// ---- provider ----

VirtualTrackerProvider::VirtualTrackerProvider(link::Logger& logger, DeviceProperties& properties,
                                               ServerDriverHost& host, NowFn now)
    : log_(logger), properties_(properties), host_(host), now_(std::move(now))
{
}

void VirtualTrackerProvider::onMessages(const std::vector<link::Message>& messages)
{
    for (const link::Message& message : messages)
    {
        if (message.type != link::MessageType::VirtualTracker)
            continue;

        const link::VirtualTracker& wire = message.virtualTracker;
        const std::string boneName(wire.name);

        Entry& entry = devices_[boneName];
        if (!entry.device)
        {
            // A new bone name: create the device and register it with SteamVR. SteamVR
            // calls Activate on the device (recording its index) as part of Add.
            const std::string serial = serialForBone(boneName);
            entry.device = std::make_unique<VirtualTracker>(serial, boneName);
            const bool added = host_.trackedDeviceAdded(
                serial.c_str(), vr::TrackedDeviceClass_GenericTracker, entry.device.get());
            if (!added)
                log_.logf("virtual tracker \"%s\": TrackedDeviceAdded rejected", boneName.c_str());
        }

        // Build and store the connected pose, stamp the refresh time.
        const vr::DriverPose_t pose = buildPose(wire);
        entry.device->setPose(pose);
        entry.lastSeenAt = now_();
        entry.connected = true;

        // Write the properties once the index is valid (post-Activate). Before that,
        // the property container does not exist.
        if (entry.device->isActivated() && !entry.propsWritten)
        {
            writeProperties(entry.device->deviceIndex());
            entry.propsWritten = true;
        }

        // Push the pose to SteamVR. Nothing is pushed before Activate: the index is
        // still invalid and SteamVR would not know which device to update.
        pushPose(*entry.device, pose);
    }
}

void VirtualTrackerProvider::onRunFrame()
{
    const double now = now_();
    for (auto& [name, entry] : devices_)
    {
        if (!entry.device || !entry.device->isActivated() || !entry.connected)
            continue;
        if (now - entry.lastSeenAt > kVirtualTrackerStaleSeconds)
            markDisconnected(entry);
    }
}

void VirtualTrackerProvider::markAllDisconnected()
{
    for (auto& [name, entry] : devices_)
    {
        if (entry.device && entry.device->isActivated() && entry.connected)
            markDisconnected(entry);
    }
}

vr::DriverPose_t VirtualTrackerProvider::buildPose(const link::VirtualTracker& wire) const
{
    vr::DriverPose_t pose{};
    pose.poseIsValid = true;
    pose.deviceIsConnected = true;
    pose.result = vr::TrackingResult_Running_OK;

    pose.qWorldFromDriverRotation = identityQuaternion();
    pose.qDriverFromHeadRotation = identityQuaternion();

    const driver::V3 position = link::fromWireVec3<driver::V3>(wire.position);
    const driver::Q rotation = link::fromWireQuat<driver::Q>(wire.rotation);
    pose.vecPosition[0] = position.x;
    pose.vecPosition[1] = position.y;
    pose.vecPosition[2] = position.z;
    pose.qRotation.w = rotation.w;
    pose.qRotation.x = rotation.x;
    pose.qRotation.y = rotation.y;
    pose.qRotation.z = rotation.z;

    return pose;
}

vr::DriverPose_t VirtualTrackerProvider::disconnectedPose() const
{
    vr::DriverPose_t pose{};
    pose.poseIsValid = false;
    pose.deviceIsConnected = false;
    pose.result = vr::TrackingResult_Uninitialized;
    pose.qWorldFromDriverRotation = identityQuaternion();
    pose.qDriverFromHeadRotation = identityQuaternion();
    return pose;
}

void VirtualTrackerProvider::writeProperties(uint32_t index)
{
    const vr::PropertyContainerHandle_t container = properties_.container(index);
    if (container == vr::k_ulInvalidPropertyContainer)
        return;
    properties_.setStringProperty(container, vr::Prop_ManufacturerName_String, kManufacturer);
    properties_.setStringProperty(container, vr::Prop_ModelNumber_String, kModelNumber);
    properties_.setStringProperty(container, vr::Prop_RenderModelName_String, kRenderModelName);
    properties_.setStringProperty(container, vr::Prop_TrackingSystemName_String,
                                  kOurTrackingSystemName);
}

void VirtualTrackerProvider::pushPose(VirtualTracker& device, const vr::DriverPose_t& pose)
{
    if (!device.isActivated())
        return;
    host_.poseUpdated(device.deviceIndex(), pose, sizeof(vr::DriverPose_t));
}

void VirtualTrackerProvider::markDisconnected(Entry& entry)
{
    const vr::DriverPose_t pose = disconnectedPose();
    entry.device->setPose(pose);
    pushPose(*entry.device, pose);
    entry.connected = false;
}
} // namespace driver
