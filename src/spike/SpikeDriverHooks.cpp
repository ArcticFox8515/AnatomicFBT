#include "SpikeDriverHooks.h"

#include "SpikeGuard.h"
#include "SpikeServer.h"

namespace spike
{
DriverHookSet::DriverHookSet(HookApi& api, Logger& logger, const DriverDetours& detours)
    : getGenericInterface(api, logger),
      trackedDeviceAdded(api, logger),
      poseUpdated(api, logger),
      detours_(detours)
{
}

void DriverHookSet::hookDriverContext(vr::IVRDriverContext* context)
{
    getGenericInterface.install(kGetGenericInterfaceHookName, context,
                                kDriverContextGetGenericInterfaceIndex,
                                detours_.getGenericInterface);
}

void DriverHookSet::hookServerDriverHost(void* host)
{
    trackedDeviceAdded.install(kTrackedDeviceAddedHookName, host,
                               kServerDriverHostTrackedDeviceAddedIndex,
                               detours_.trackedDeviceAdded);
    poseUpdated.install(kPoseUpdatedHookName, host,
                        kServerDriverHostTrackedDevicePoseUpdatedIndex, detours_.poseUpdated);
}

void DriverHookSet::removeAll()
{
    poseUpdated.remove();
    trackedDeviceAdded.remove();
    getGenericInterface.remove();
}

// ---- the detour bodies ------------------------------------------------------------

void* observeGetGenericInterface(DriverHookSet& hooks, SpikeObserver& observer,
                                 vr::IVRDriverContext* self, const char* version,
                                 vr::EVRInitError* error)
{
    // Observed *after* the call: the interface pointer is the whole point, and it only
    // exists once vrserver has returned it.
    void* result = hooks.getGenericInterface.original()(self, version, error);
    runGuarded([&] { observer.onInterfaceRequested(version, result); });
    return result;
}

bool observeTrackedDeviceAdded(DriverHookSet& hooks, SpikeObserver& observer,
                               vr::IVRServerDriverHost* self, const char* serial,
                               vr::ETrackedDeviceClass deviceClass,
                               vr::ITrackedDeviceServerDriver* driver)
{
    // Observed *before* the call: vrserver may call straight back into the device driver
    // (activation) while still inside TrackedDeviceAdded, and those callbacks should
    // find the device already recorded.
    runGuarded([&] { observer.onDeviceAdded(serial, deviceClass); });
    return hooks.trackedDeviceAdded.original()(self, serial, deviceClass, driver);
}

void observeTrackedDevicePoseUpdated(DriverHookSet& hooks, SpikeObserver& observer,
                                     vr::IVRServerDriverHost* self, uint32_t index,
                                     const vr::DriverPose_t& pose, uint32_t poseStructSize)
{
    runGuarded([&] { observer.onPose(index, pose, poseStructSize); });
    // Forwarded unchanged — this spike never modifies a pose.
    hooks.poseUpdated.original()(self, index, pose, poseStructSize);
}
} // namespace spike
