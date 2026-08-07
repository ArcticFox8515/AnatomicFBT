#pragma once

// Throwaway step-1 spike (doc/driver-plan.md): the driver's six hooks, as data plus
// behaviour, in SpikeLib rather than in the DLL.
//
// Everything here used to live in SpikeDriver.cpp, where no unit test can reach it:
// that file is compiled only into driver_00trackingcorrector, so the only thing that
// ever executed it was SpikeDriverTest via LoadLibrary — an integration test, and
// therefore no evidence at all under the bar in doc/driver-spike-handover.md §2.1.
// What moved:
//
//   * the vtable index table (which slot of which interface each hook patches),
//   * the install order and the reverse removal order,
//   * the body of every detour: forward unchanged, then notify the observer.
//
// The DLL keeps only six one-line detour functions whose address MinHook needs.
//
// openvr_driver.h is included for its types and its interface *layout*; nothing here
// calls into vrserver.

#include "SpikeHooks.h"
#include "SpikeLog.h"
#include "SpikeObserver.h"

#include <openvr_driver.h>

#include <cstdint>

namespace spike
{
// ---- the hooked functions, as callable types --------------------------------------

using GetGenericInterfaceFn = void*(*)(vr::IVRDriverContext*, const char*, vr::EVRInitError*);
using TrackedDeviceAddedFn = bool(*)(vr::IVRServerDriverHost*, const char*, vr::ETrackedDeviceClass,
                                     vr::ITrackedDeviceServerDriver*);
using TrackedDevicePoseUpdatedFn = void(*)(vr::IVRServerDriverHost*, uint32_t,
                                           const vr::DriverPose_t&, uint32_t);
using CreateBooleanComponentFn = vr::EVRInputError(*)(vr::IVRDriverInput*,
                                                      vr::PropertyContainerHandle_t, const char*,
                                                      vr::VRInputComponentHandle_t*);
using UpdateBooleanComponentFn = vr::EVRInputError(*)(vr::IVRDriverInput*,
                                                      vr::VRInputComponentHandle_t, bool, double);
using CreateScalarComponentFn = vr::EVRInputError(*)(vr::IVRDriverInput*,
                                                     vr::PropertyContainerHandle_t, const char*,
                                                     vr::VRInputComponentHandle_t*,
                                                     vr::EVRScalarType, vr::EVRScalarUnits);

// ---- the vtable index table -------------------------------------------------------
//
// Declaration order in the openvr_driver.h interfaces, which is the ABI. Named
// constants so a unit test can assert the table the plan fixed, instead of the numbers
// only existing as literals at the install call sites inside the DLL. That the real
// MinHook lands on the function these slots point at is a separate claim, proved by
// SpikeDriverTest against a fake vrserver.

constexpr int kDriverContextGetGenericInterfaceIndex = 0;
constexpr int kServerDriverHostTrackedDeviceAddedIndex = 0;
constexpr int kServerDriverHostTrackedDevicePoseUpdatedIndex = 1;
constexpr int kDriverInputCreateBooleanComponentIndex = 0;
constexpr int kDriverInputUpdateBooleanComponentIndex = 1;
constexpr int kDriverInputCreateScalarComponentIndex = 2;

// The log labels, also asserted by tests.
constexpr const char* kGetGenericInterfaceHookName = "IVRDriverContext::GetGenericInterface";
constexpr const char* kTrackedDeviceAddedHookName = "IVRServerDriverHost::TrackedDeviceAdded";
constexpr const char* kPoseUpdatedHookName = "IVRServerDriverHost::TrackedDevicePoseUpdated";
constexpr const char* kCreateBooleanHookName = "IVRDriverInput::CreateBooleanComponent";
constexpr const char* kUpdateBooleanHookName = "IVRDriverInput::UpdateBooleanComponent";
constexpr const char* kCreateScalarHookName = "IVRDriverInput::CreateScalarComponent";

// The six detour addresses. Only the DLL can supply these: MinHook needs the address of
// a function with the exact hooked signature, and those functions must live in the
// module that is allowed to be non-testable.
struct DriverDetours
{
    void* getGenericInterface = nullptr;
    void* trackedDeviceAdded = nullptr;
    void* poseUpdated = nullptr;
    void* createBoolean = nullptr;
    void* updateBoolean = nullptr;
    void* createScalar = nullptr;
};

// Owns the six hooks and knows where they go. Implements InterfaceHooks so the observer
// can trigger the two lazy installs the moment vrserver hands the interface over.
class DriverHookSet final : public InterfaceHooks
{
public:
    DriverHookSet(HookApi& api, Logger& logger, const DriverDetours& detours);

    // IVRDriverContext, hooked from Init with the context vrserver passed us.
    void hookDriverContext(vr::IVRDriverContext* context);

    void hookServerDriverHost(void* host) override;
    void hookDriverInput(void* input) override;

    // Reverse install order: the interface hooks come off before the context hook that
    // discovers them, so a call in flight cannot re-enter a hook being removed.
    void removeAll();

    VTableHook<GetGenericInterfaceFn> getGenericInterface;
    VTableHook<TrackedDeviceAddedFn> trackedDeviceAdded;
    VTableHook<TrackedDevicePoseUpdatedFn> poseUpdated;
    VTableHook<CreateBooleanComponentFn> createBoolean;
    VTableHook<UpdateBooleanComponentFn> updateBoolean;
    VTableHook<CreateScalarComponentFn> createScalar;

private:
    DriverDetours detours_;
};

// ---- the detour bodies ------------------------------------------------------------
//
// One per hook: forward to the original unchanged, tell the observer, return whatever
// vrserver returned. The observer call is guarded (runGuarded) because an exception
// escaping a detour lands inside vrserver.exe and kills SteamVR.
//
// Argument order and forwarding order are part of what is being proved: the spike must
// not perturb the driver whose calls it watches, so the pose hook forwards the *same*
// reference it was given and never modifies it.

void* observeGetGenericInterface(DriverHookSet& hooks, SpikeObserver& observer,
                                 vr::IVRDriverContext* self, const char* version,
                                 vr::EVRInitError* error);

bool observeTrackedDeviceAdded(DriverHookSet& hooks, SpikeObserver& observer,
                               vr::IVRServerDriverHost* self, const char* serial,
                               vr::ETrackedDeviceClass deviceClass,
                               vr::ITrackedDeviceServerDriver* driver);

void observeTrackedDevicePoseUpdated(DriverHookSet& hooks, SpikeObserver& observer,
                                     vr::IVRServerDriverHost* self, uint32_t index,
                                     const vr::DriverPose_t& pose, uint32_t poseStructSize);

vr::EVRInputError observeCreateBooleanComponent(DriverHookSet& hooks, SpikeObserver& observer,
                                                vr::IVRDriverInput* self,
                                                vr::PropertyContainerHandle_t container,
                                                const char* name,
                                                vr::VRInputComponentHandle_t* handle);

vr::EVRInputError observeUpdateBooleanComponent(DriverHookSet& hooks, SpikeObserver& observer,
                                                vr::IVRDriverInput* self,
                                                vr::VRInputComponentHandle_t handle, bool value,
                                                double timeOffset);

vr::EVRInputError observeCreateScalarComponent(DriverHookSet& hooks, SpikeObserver& observer,
                                               vr::IVRDriverInput* self,
                                               vr::PropertyContainerHandle_t container,
                                               const char* name,
                                               vr::VRInputComponentHandle_t* handle,
                                               vr::EVRScalarType type, vr::EVRScalarUnits units);
} // namespace spike
