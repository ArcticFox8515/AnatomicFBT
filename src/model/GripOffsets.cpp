#include "model/GripOffsets.h"

#include <algorithm>

std::vector<TrackedDevice> applyGripOffsets(const std::vector<TrackedDevice>& devices,
                                            const std::vector<GripOffset>& offsets)
{
    std::vector<TrackedDevice> result;
    result.reserve(devices.size());
    for (const TrackedDevice& device : devices)
    {
        if (device.kind != TrackedDeviceKind::Controller)
        {
            result.push_back(device);
            continue;
        }
        const auto it = std::find_if(offsets.begin(), offsets.end(),
                                     [&](const GripOffset& o) { return o.deviceId == device.id; });
        if (it == offsets.end())
        {
            result.push_back(device);
            continue;
        }
        const Pose shifted = compose(device.pose, it->deviceToGrip);
        result.push_back({device.id, device.kind, shifted});
    }
    return result;
}

std::vector<GripOffset> mergeGripOffsets(const std::vector<GripOffset>& cached,
                                         const std::vector<GripOffset>& fresh)
{
    std::vector<GripOffset> result = fresh;
    for (const GripOffset& c : cached)
    {
        const auto it = std::find_if(result.begin(), result.end(),
                                     [&](const GripOffset& o) { return o.deviceId == c.deviceId; });
        if (it == result.end())
            result.push_back(c);
    }
    return result;
}
