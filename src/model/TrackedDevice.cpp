#include "TrackedDevice.h"

const TrackedDevice* findHmd(const std::vector<TrackedDevice>& devices)
{
    for (const TrackedDevice& device : devices)
        if (device.kind == TrackedDeviceKind::Hmd)
            return &device;
    return nullptr;
}

const char* deviceKindName(TrackedDeviceKind kind)
{
    switch (kind)
    {
    case TrackedDeviceKind::Hmd:
        return "HMD";
    case TrackedDeviceKind::Controller:
        return "controller";
    case TrackedDeviceKind::Tracker:
        return "tracker";
    default:
        return "device";
    }
}

std::vector<std::pair<int, Pose>> devicePosePairs(const std::vector<TrackedDevice>& devices)
{
    std::vector<std::pair<int, Pose>> pairs;
    pairs.reserve(devices.size());
    for (const TrackedDevice& device : devices)
        pairs.emplace_back(device.id, device.pose);
    return pairs;
}
