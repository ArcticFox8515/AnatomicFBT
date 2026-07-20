#include "TrackerCalibration.h"

#include "IkRig.h"
#include "Skeleton.h"

#include <limits>

DeviceAssignment assignDevicesToTargets(const std::vector<glm::vec3>& devicePositions,
                                        const std::vector<glm::vec3>& targetPositions)
{
    DeviceAssignment result;
    result.deviceIndex.assign(targetPositions.size(), -1);
    result.targetIndex.assign(devicePositions.size(), -1);

    std::vector<bool> deviceTaken(devicePositions.size(), false);
    std::vector<bool> targetTaken(targetPositions.size(), false);

    for (size_t pairs = std::min(devicePositions.size(), targetPositions.size()); pairs > 0; --pairs)
    {
        float bestDist = std::numeric_limits<float>::max();
        size_t bestDevice = 0;
        size_t bestTarget = 0;
        for (size_t d = 0; d < devicePositions.size(); ++d)
        {
            if (deviceTaken[d])
                continue;
            for (size_t t = 0; t < targetPositions.size(); ++t)
            {
                if (targetTaken[t])
                    continue;
                const glm::vec3 diff = devicePositions[d] - targetPositions[t];
                const float dist = glm::dot(diff, diff);
                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestDevice = d;
                    bestTarget = t;
                }
            }
        }
        deviceTaken[bestDevice] = true;
        targetTaken[bestTarget] = true;
        result.targetIndex[bestDevice] = static_cast<int>(bestTarget);
        result.deviceIndex[bestTarget] = static_cast<int>(bestDevice);
    }

    return result;
}

const Pose* TrackerCalibration::findDevice(const std::vector<std::pair<int, Pose>>& devices,
                                           int deviceId)
{
    for (const auto& [id, pose] : devices)
        if (id == deviceId)
            return &pose;
    return nullptr;
}

void TrackerCalibration::calibrate(const DeviceAssignment& assignment,
                                   const std::vector<std::pair<int, Pose>>& devices,
                                   const std::vector<Pose>& boneWorldPoses)
{
    bindings_.clear();
    bindings_.resize(boneWorldPoses.size());

    for (size_t t = 0; t < assignment.deviceIndex.size(); ++t)
    {
        const int deviceId = assignment.deviceIndex[t];
        const Pose* devicePose = deviceId >= 0 ? findDevice(devices, deviceId) : nullptr;
        if (!devicePose)
            continue;
        bindings_[t] = Binding{deviceId, compose(inverse(*devicePose), boneWorldPoses[t])};
    }
}

size_t TrackerCalibration::applyDevicePoses(const std::vector<std::pair<int, Pose>>& devices,
                                            std::vector<IkTarget>& targets) const
{
    size_t updated = 0;
    for (size_t t = 0; t < bindings_.size() && t < targets.size(); ++t)
    {
        if (!bindings_[t])
            continue;
        const Pose* devicePose = findDevice(devices, bindings_[t]->deviceId);
        if (!devicePose)
            continue;
        targets[t].position = devicePose->position;
        targets[t].rotation = devicePose->rotation;
        ++updated;
    }
    return updated;
}

void TrackerCalibration::applyOffsets(std::vector<IkTarget>& goals) const
{
    for (size_t t = 0; t < bindings_.size() && t < goals.size(); ++t)
    {
        if (!bindings_[t])
            continue;
        const Pose goal = compose({goals[t].position, goals[t].rotation}, bindings_[t]->offset);
        goals[t].position = goal.position;
        goals[t].rotation = goal.rotation;
    }
}

void TrackerCalibration::clear()
{
    bindings_.clear();
}

bool TrackerCalibration::isCalibrated() const
{
    for (const auto& binding : bindings_)
        if (binding)
            return true;
    return false;
}

std::optional<int> TrackerCalibration::boundDevice(size_t targetIndex) const
{
    if (targetIndex >= bindings_.size() || !bindings_[targetIndex])
        return std::nullopt;
    return bindings_[targetIndex]->deviceId;
}

CalibrationFrame updateCalibrationFrame(IkRig& rig, const std::vector<TrackedDevice>& devices)
{
    CalibrationFrame frame;

    for (Joint& joint : rig.skeleton.joints)
        joint.localRot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    int rootIndex = -1;
    for (size_t i = 0; i < rig.skeleton.joints.size(); ++i)
        if (!rig.skeleton.joints[i].parentIndex)
        {
            rootIndex = static_cast<int>(i);
            break;
        }

    const TrackedDevice* hmd = findHmd(devices);
    if (hmd && rootIndex >= 0)
    {
        rig.skeleton.rootPosition = hmd->pose.position;
        rig.skeleton.joints[rootIndex].localRot = yawOnly(hmd->pose.rotation);
    }

    const WorldTransforms wt = computeWorldTransforms(rig.skeleton);
    std::vector<glm::vec3> bonePositions(rig.targets.size());
    frame.boneWorldPoses.resize(rig.targets.size());
    for (size_t i = 0; i < rig.targets.size(); ++i)
    {
        frame.boneWorldPoses[i] = {wt.positions[rig.targets[i].jointIndex],
                                   wt.rotations[rig.targets[i].jointIndex]};
        bonePositions[i] = frame.boneWorldPoses[i].position;
    }

    std::vector<glm::vec3> devicePositions;
    devicePositions.reserve(devices.size());
    for (const TrackedDevice& device : devices)
        devicePositions.push_back(device.pose.position);

    frame.assignment = assignDevicesToTargets(devicePositions, bonePositions);

    // Show raw device poses at the target markers so the user sees what will
    // be captured.
    for (size_t i = 0; i < rig.targets.size(); ++i)
    {
        const int d = frame.assignment.deviceIndex[i];
        if (d < 0)
            continue;
        rig.targets[i].position = devices[d].pose.position;
        rig.targets[i].rotation = devices[d].pose.rotation;
    }

    return frame;
}

void captureOffsets(TrackerCalibration& calibration, const CalibrationFrame& frame,
                    const std::vector<TrackedDevice>& devices)
{
    // Translate compact device-list positions to stable device ids.
    DeviceAssignment byIds = frame.assignment;
    for (int& d : byIds.deviceIndex)
        if (d >= 0)
            d = devices[d].id;
    calibration.calibrate(byIds, devicePosePairs(devices), frame.boneWorldPoses);
}

std::vector<IkTarget> updateCaptureFrame(IkRig& rig, const TrackerCalibration& calibration,
                                         const std::vector<TrackedDevice>& devices)
{
    calibration.applyDevicePoses(devicePosePairs(devices), rig.targets);
    std::vector<IkTarget> goals = rig.targets;
    calibration.applyOffsets(goals);
    return goals;
}
