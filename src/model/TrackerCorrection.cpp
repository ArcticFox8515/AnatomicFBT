#include "TrackerCorrection.h"

#include "IkRig.h"
#include "Skeleton.h"
#include "TrackerCalibration.h"

CorrectionMap buildCorrectionMap(const IkRig& rig, const Skeleton& avatar)
{
    CorrectionMap map;
    map.avatarJoint.resize(rig.targets.size());
    map.enabled.resize(rig.targets.size(), false);

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
                break;
            }
        }
    }

    return map;
}

std::vector<CorrectedPose> correctDevicePoses(const TrackerCalibration& calibration,
                                              const CorrectionMap& map,
                                              const Skeleton& avatar)
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

        // Place the tracker at the center of the avatar's matching bone: the
        // avatar joint's world pose, no strap offset. The bone-local offset
        // captured at calibration is deliberately dropped — the reference and
        // avatar skeletons have differently-oriented bone frames, so re-hanging
        // in the local frame rotates the tracker wrong. The joint center is
        // frame-independent.
        const int joint = *map.avatarJoint[t];
        const Pose boneWorld{world.positions[static_cast<size_t>(joint)],
                             world.rotations[static_cast<size_t>(joint)]};
        result.push_back({t, *deviceId, boneWorld});
    }

    return result;
}
