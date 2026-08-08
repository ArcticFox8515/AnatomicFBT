#include "TrackerCorrection.h"

#include "IkRig.h"
#include "Skeleton.h"
#include "TrackerCalibration.h"
#include "TrackedDevice.h"

CorrectionMap buildCorrectionMap(const IkRig& rig, const Skeleton& avatar)
{
    CorrectionMap map;
    map.avatarJoint.resize(rig.targets.size());
    map.enabled.resize(rig.targets.size(), false);
    map.rotationEnabled.resize(rig.targets.size(), false);

    for (size_t t = 0; t < rig.targets.size(); ++t)
    {
        // The anchor target (Head = the HMD) is never a tracker and must never
        // be corrected — skip it outright so it has no avatar joint and no
        // checkbox.
        if (t < rig.config.targets.size() && rig.config.targets[t].solver == SolverType::Anchor)
            continue;
        const int jointIndex = rig.targets[t].jointIndex;
        if (jointIndex < 0 || static_cast<size_t>(jointIndex) >= rig.skeleton.joints.size())
            continue;
        const std::string& name = rig.skeleton.joints[static_cast<size_t>(jointIndex)].name;

        for (size_t a = 0; a < avatar.joints.size(); ++a)
        {
            if (avatar.joints[a].name == name)
            {
                map.avatarJoint[t] = static_cast<int>(a);
                map.enabled[t] = true;
                map.rotationEnabled[t] = true;
                break;
            }
        }
    }

    return map;
}

std::vector<CorrectedPose> correctDevicePoses(const TrackerCalibration& calibration,
                                              const CorrectionMap& map,
                                              const Skeleton& avatar,
                                              const std::vector<TrackedDevice>& devices)
{
    std::vector<CorrectedPose> result;
    if (map.avatarJoint.empty() || !calibration.isCalibrated())
        return result;

    const WorldTransforms world = computeWorldTransforms(avatar);

    for (size_t t = 0; t < map.avatarJoint.size(); ++t)
    {
        if (!map.enabled[t] || !map.avatarJoint[t])
            continue;
        const std::optional<int> deviceId = calibration.boundDevice(t);
        if (!deviceId)
            continue;

        // A device bound at calibration but absent from this frame's snapshot
        // has no raw pose to correct — skip it (no stale marker).
        const TrackedDevice* raw = nullptr;
        for (const TrackedDevice& d : devices)
            if (d.id == *deviceId)
            {
                raw = &d;
                break;
            }
        if (!raw)
            continue;

        // Place the tracker at the center of the avatar's matching bone: the
        // avatar joint's world pose, no strap offset. The bone-local offset
        // captured at calibration is deliberately dropped — the reference and
        // avatar skeletons have differently-oriented bone frames, so re-hanging
        // in the local frame rotates the tracker wrong. The joint center is
        // frame-independent.
        const int joint = *map.avatarJoint[t];
        const glm::vec3 avatarPosition = world.positions[static_cast<size_t>(joint)];

        // Controllers are aiming devices — their rotation is always locked to
        // the raw pose regardless of the user toggle. Trackers honor
        // `map.rotationEnabled[t]`: when false, they keep the raw rotation too
        // (position-only correction) — useful when avatar bone-roll differences
        // produce a visually wrong tracker orientation.
        const bool lockRotation = raw->kind == TrackedDeviceKind::Controller
            || !map.rotationEnabled[t];

        if (lockRotation)
        {
            result.push_back({t, *deviceId, Pose{avatarPosition, raw->pose.rotation}, true});
        }
        else
        {
            const Pose boneWorld{avatarPosition, world.rotations[static_cast<size_t>(joint)]};
            result.push_back({t, *deviceId, boneWorld, false});
        }
    }

    return result;
}

std::vector<DeviceOffset> correctionOffsets(const std::vector<CorrectedPose>& corrected,
                                            const std::vector<TrackedDevice>& devices)
{
    std::vector<DeviceOffset> result;
    for (const CorrectedPose& c : corrected)
    {
        const Pose* raw = nullptr;
        for (const TrackedDevice& d : devices)
            if (d.id == c.deviceId)
            {
                raw = &d.pose;
                break;
            }
        if (!raw)
            continue;
        // A locked corrected pose (controller or rotation-disabled tracker)
        // already carries the raw rotation, so the delta's rotation is exactly
        // identity — emit it directly rather than letting float epsilon leak
        // into the driver.
        if (c.rotationLocked)
            result.push_back({c.deviceId, Pose{c.pose.position - raw->position,
                                               glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}});
        else
            result.push_back({c.deviceId, compose(c.pose, inverse(*raw))});
    }
    return result;
}
