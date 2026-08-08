#include "model/OpenVrTracking.h"

#include "link/Protocol.h"

#include "model/Error.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>

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
        applyPose(message.pose);
    return devices_;
}

void OpenVrTracking::applyPose(const link::DevicePose& pose)
{
    // `Other` was skipped by the old pollPoses (base stations, etc.).
    if (pose.deviceKind == link::DeviceKind::Other)
        return;

    auto it = std::lower_bound(devices_.begin(), devices_.end(),
                               static_cast<int>(pose.deviceId),
                               [](const TrackedDevice& d, int id) { return d.id < id; });

    // A Lost frame is the driver-side equivalent of the old
    // `!bPoseIsValid || !bDeviceIsConnected` skip — the device leaves the
    // snapshot this frame, and returns once Tracking resumes.
    if (pose.tracking == link::TrackingState::Lost)
    {
        if (it != devices_.end() && it->id == static_cast<int>(pose.deviceId))
            devices_.erase(it);
        return;
    }

    const TrackedDeviceKind kind = mapKind(pose.deviceKind);
    const glm::vec3 position(pose.position[0], pose.position[1], pose.position[2]);
    const glm::quat rotation(pose.rotation[3], pose.rotation[0],
                             pose.rotation[1], pose.rotation[2]);

    if (it != devices_.end() && it->id == static_cast<int>(pose.deviceId))
    {
        it->kind = kind;
        it->pose = {position, rotation};
    }
    else
    {
        devices_.insert(it, {static_cast<int>(pose.deviceId), kind, {position, rotation}});
    }
}