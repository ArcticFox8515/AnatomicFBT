#include "model/OpenVrTracking.h"

#include "link/Convert.h"
#include "link/Protocol.h"

#include "model/Error.h"

#include <algorithm>
#include <cstring>

namespace
{
TrackedDeviceKind mapKind(link::DeviceKind kind)
{
    switch (kind)
    {
    case link::DeviceKind::Hmd: return TrackedDeviceKind::Hmd;
    case link::DeviceKind::Controller: return TrackedDeviceKind::Controller;
    case link::DeviceKind::Tracker: return TrackedDeviceKind::Tracker;
    default: return TrackedDeviceKind::Other;
    }
}
} // namespace

OpenVrTracking::OpenVrTracking(link::Logger& logger, link::PipeFactoryFn factory,
                               std::function<double()> now)
    : channel_(logger, std::move(factory)), now_(std::move(now))
{
}

OpenVrTracking::~OpenVrTracking() = default;

void OpenVrTracking::init()
{
    if (channel_.connected())
        return;
    std::vector<link::Message> messages;
    channel_.receive(messages);
    if (!channel_.connected())
        throw Error("OpenVR driver not connected");
}

bool OpenVrTracking::isInitialized() const
{
    return channel_.connected();
}

std::vector<TrackedDevice> OpenVrTracking::pollPoses()
{
    if (!channel_.connected())
    {
        if (now_() < nextAttemptAt_)
            return {};
        nextAttemptAt_ = now_() + 1.0;
    }

    std::vector<link::Message> messages;
    channel_.receive(messages);
    if (!channel_.connected())
        return {};

    for (const link::Message& message : messages)
    {
        if (message.type != link::MessageType::DevicePose)
            continue;
        applyPose(message.devicePose);
    }
    return devices_;
}

void OpenVrTracking::applyPose(const link::DevicePose& frame)
{
    // `Other` was skipped by the old pollPoses (base stations, etc.).
    if (frame.deviceKind == link::DeviceKind::Other)
        return;

    auto it = std::lower_bound(devices_.begin(), devices_.end(),
                               static_cast<int>(frame.deviceId),
                               [](const TrackedDevice& d, int id) { return d.id < id; });

    // A Lost frame is the driver-side equivalent of the old
    // `!bPoseIsValid || !bDeviceIsConnected` skip — the device leaves the
    // snapshot this frame, and returns once Tracking resumes.
    if (frame.tracking == link::TrackingState::Lost)
    {
        if (it != devices_.end() && it->id == static_cast<int>(frame.deviceId))
            devices_.erase(it);
        return;
    }

    const TrackedDeviceKind kind = mapKind(frame.deviceKind);
    const glm::vec3 position = link::fromWireVec3<glm::vec3>(frame.position);
    const glm::quat rotation = link::fromWireQuat<glm::quat>(frame.rotation);

    if (it != devices_.end() && it->id == static_cast<int>(frame.deviceId))
    {
        it->kind = kind;
        it->pose = {position, rotation};
    }
    else
    {
        devices_.insert(it, {static_cast<int>(frame.deviceId), kind, {position, rotation}});
    }
}

void OpenVrTracking::sendOffsets(const std::vector<DeviceOffset>& offsets)
{
    for (const DeviceOffset& offset : offsets)
    {
        link::Message message;
        message.size = sizeof(link::PoseOverride);
        message.type = link::MessageType::PoseOverride;
        message.poseOverride.deviceId = static_cast<std::uint32_t>(offset.deviceId);
        message.poseOverride.position = link::toWireVec3(offset.delta.position);
        message.poseOverride.rotation = link::toWireQuat(offset.delta.rotation);
        channel_.send(message);
    }
}

void OpenVrTracking::sendVirtualTrackers(const std::vector<VirtualTrackerPose>& trackers)
{
    for (const VirtualTrackerPose& tracker : trackers)
    {
        link::VirtualTracker wire;
        const std::size_t n = std::min(tracker.name.size(), sizeof(wire.name) - 1);
        std::memcpy(wire.name, tracker.name.data(), n);
        wire.tracking = link::TrackingState::Tracking;
        wire.position = link::toWireVec3(tracker.pose.position);
        wire.rotation = link::toWireQuat(tracker.pose.rotation);

        link::Message message;
        message.size = sizeof(link::VirtualTracker);
        message.type = link::MessageType::VirtualTracker;
        message.virtualTracker = wire;
        channel_.send(message);
    }
}