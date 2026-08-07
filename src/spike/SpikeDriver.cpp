// Throwaway step-1 spike (doc/driver-plan.md). Observation only: this driver adds
// no devices, modifies no poses and forwards every hooked call unchanged. Its whole
// job is to close the unknowns the real driver will be designed around:
//
//   * does the DLL load at all, in which processes, and in what order,
//   * which interface version strings pass through GetGenericInterface (an
//     unrecognized version is a silent no-op, so it is logged loudly),
//   * device ids / classes / serials / role hints read driver-side via IVRProperties,
//   * boolean (and scalar) input component paths per device, and trigger transitions,
//   * pose update rate per device and RunFrame cadence,
//   * the DriverPose_t -> world composition: both candidate formulas are logged so
//     they can be compared against the client-side TrackingUniverseRaw pose that
//     spike_client prints for the same device at the same time.
//
// THIS FILE IS THE ADAPTER, AND NOTHING ELSE.
//
// It is compiled only into driver_00trackingcorrector, never into the test
// executable, so nothing in it can run in a unit test — SpikeDriverTest reaches it
// through LoadLibrary, which is an integration test and proves nothing under the bar
// in doc/driver-spike-handover.md §2.1. Therefore every function here is one
// instruction: a call into SpikeLib, or a Win32/openvr/MinHook call whose arguments it
// merely marshals. The two-line functions are exactly the leaked-singleton accessors
// (create the object, return it) and the provider Init entry points (open the log,
// then forward). There is no branch, no loop, no comparison and no arithmetic in this
// file: if you find yourself adding one, it belongs in SpikeLib with a test.
//
// Hard rule: no exception may leave a hook or a provider entry point — an exception
// escaping into vrserver.exe kills SteamVR. runGuarded() sits inside the SpikeLib
// forwarders (SpikeDriverHooks.cpp) and inside SpikeServer, i.e. on the other side of
// every call below.

#include "SpikeDriverEnvironment.h"
#include "SpikeDriverHooks.h"
#include "SpikeHooks.h"
#include "SpikeLog.h"
#include "SpikeObserver.h"
#include "SpikeServer.h"

#include <openvr_driver.h>

#include <windows.h>

#include <MinHook.h>

#include <chrono>
#include <string>

namespace
{
using spike::log;

// --------------------------------------------------------------- MinHook glue ----

class MinHookApi final : public spike::HookApi
{
public:
    int initialize() override { return MH_Initialize(); }

    void shutdown() override { MH_Uninitialize(); }

    bool isAlreadyInitialized(int status) override
    {
        return status == MH_ERROR_ALREADY_INITIALIZED;
    }

    int create(void* target, void* detour, void** original) override
    {
        return MH_CreateHook(target, detour, original);
    }

    int enable(void* target) override { return MH_EnableHook(target); }

    int remove(void* target) override { return MH_RemoveHook(target); }

    const char* statusName(int status) override
    {
        return MH_StatusToString(static_cast<MH_STATUS>(status));
    }
};

MinHookApi& hookApi()
{
    static MinHookApi* instance = new MinHookApi();
    return *instance;
}

// ---------------------------------------------------------------- Win32 glue ----

class Win32ModuleApi final : public spike::ModuleApi
{
public:
    int moduleFromAddress(void* address, void** module) override
    {
        return GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS
                                      | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                                  reinterpret_cast<LPCSTR>(address),
                                  reinterpret_cast<HMODULE*>(module));
    }

    unsigned long moduleFileName(void* module, char* buffer, unsigned long size) override
    {
        return GetModuleFileNameA(static_cast<HMODULE>(module), buffer, size);
    }
};

Win32ModuleApi& moduleApi()
{
    static Win32ModuleApi* instance = new Win32ModuleApi();
    return *instance;
}

double nowSeconds()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// ------------------------------------------------------------------ detours ----
//
// MinHook needs the address of a function with the hooked signature, and only this
// module may define one. Each is a single call into the SpikeLib forwarder that owns
// the actual behaviour (forward unchanged, then observe).

void* detourGetGenericInterface(vr::IVRDriverContext* self, const char* version,
                                vr::EVRInitError* error);
bool detourTrackedDeviceAdded(vr::IVRServerDriverHost* self, const char* serial,
                              vr::ETrackedDeviceClass deviceClass,
                              vr::ITrackedDeviceServerDriver* driver);
void detourTrackedDevicePoseUpdated(vr::IVRServerDriverHost* self, uint32_t index,
                                    const vr::DriverPose_t& pose, uint32_t poseStructSize);
vr::EVRInputError detourCreateBooleanComponent(vr::IVRDriverInput* self,
                                               vr::PropertyContainerHandle_t container,
                                               const char* name,
                                               vr::VRInputComponentHandle_t* handle);
vr::EVRInputError detourUpdateBooleanComponent(vr::IVRDriverInput* self,
                                               vr::VRInputComponentHandle_t handle, bool value,
                                               double timeOffset);
vr::EVRInputError detourCreateScalarComponent(vr::IVRDriverInput* self,
                                              vr::PropertyContainerHandle_t container,
                                              const char* name,
                                              vr::VRInputComponentHandle_t* handle,
                                              vr::EVRScalarType type, vr::EVRScalarUnits units);

// The hook set and the observer are process-wide and deliberately leaked, like the
// rest of the state here: a hook thread may still be inside a detour when the DLL's
// static destructors would otherwise run.

spike::DriverHookSet& hooks()
{
    static spike::DriverHookSet* instance = new spike::DriverHookSet(
        hookApi(), log(),
        spike::DriverDetours{reinterpret_cast<void*>(&detourGetGenericInterface),
                             reinterpret_cast<void*>(&detourTrackedDeviceAdded),
                             reinterpret_cast<void*>(&detourTrackedDevicePoseUpdated),
                             reinterpret_cast<void*>(&detourCreateBooleanComponent),
                             reinterpret_cast<void*>(&detourUpdateBooleanComponent),
                             reinterpret_cast<void*>(&detourCreateScalarComponent)});
    return *instance;
}

spike::SpikeObserver& observer()
{
    static spike::SpikeObserver* instance =
        new spike::SpikeObserver(log(), hooks(), &nowSeconds);
    return *instance;
}

void* detourGetGenericInterface(vr::IVRDriverContext* self, const char* version,
                                vr::EVRInitError* error)
{
    return spike::observeGetGenericInterface(hooks(), observer(), self, version, error);
}

bool detourTrackedDeviceAdded(vr::IVRServerDriverHost* self, const char* serial,
                              vr::ETrackedDeviceClass deviceClass,
                              vr::ITrackedDeviceServerDriver* driver)
{
    return spike::observeTrackedDeviceAdded(hooks(), observer(), self, serial, deviceClass,
                                            driver);
}

void detourTrackedDevicePoseUpdated(vr::IVRServerDriverHost* self, uint32_t index,
                                    const vr::DriverPose_t& pose, uint32_t poseStructSize)
{
    spike::observeTrackedDevicePoseUpdated(hooks(), observer(), self, index, pose,
                                           poseStructSize);
}

vr::EVRInputError detourCreateBooleanComponent(vr::IVRDriverInput* self,
                                               vr::PropertyContainerHandle_t container,
                                               const char* name,
                                               vr::VRInputComponentHandle_t* handle)
{
    return spike::observeCreateBooleanComponent(hooks(), observer(), self, container, name,
                                                handle);
}

vr::EVRInputError detourUpdateBooleanComponent(vr::IVRDriverInput* self,
                                               vr::VRInputComponentHandle_t handle, bool value,
                                               double timeOffset)
{
    return spike::observeUpdateBooleanComponent(hooks(), observer(), self, handle, value,
                                                timeOffset);
}

vr::EVRInputError detourCreateScalarComponent(vr::IVRDriverInput* self,
                                              vr::PropertyContainerHandle_t container,
                                              const char* name,
                                              vr::VRInputComponentHandle_t* handle,
                                              vr::EVRScalarType type, vr::EVRScalarUnits units)
{
    return spike::observeCreateScalarComponent(hooks(), observer(), self, container, name, handle,
                                               type, units);
}

// ------------------------------------------------- vrserver environment glue ----

class OpenVrServerEnvironment final : public spike::ServerEnvironment
{
public:
    vr::EVRInitError initContext(vr::IVRDriverContext* context) override
    {
        return vr::InitServerDriverContext(context);
    }

    void cleanupContext() override { vr::CleanupDriverContext(); }

    void routeLogToDriverLog() override { log().setSink(spike::driverLogSink(&vr::VRDriverLog)); }

    const char* initHookLibrary() override { return spike::initializeHookLibrary(hookApi()); }

    void shutdownHookLibrary() override { hookApi().shutdown(); }

    std::string modulePath() override
    {
        return spike::modulePathOfAddress(moduleApi(),
                                          reinterpret_cast<void*>(&detourGetGenericInterface));
    }

    unsigned long processId() override { return GetCurrentProcessId(); }

    spike::DeviceProperties* properties() override { return properties_.orNullIfUnavailable(); }

    void hookDriverContext(vr::IVRDriverContext* context) override
    {
        hooks().hookDriverContext(context);
    }

    void hookServerDriverHost() override { hooks().hookServerDriverHost(vr::VRServerDriverHost()); }

    void hookDriverInput() override { hooks().hookDriverInput(vr::VRDriverInput()); }

    void removeHooks() override { hooks().removeAll(); }

private:
    spike::OpenVrProperties properties_{&vr::VRProperties};
};

class OpenVrWatchdogEnvironment final : public spike::WatchdogEnvironment
{
public:
    vr::EVRInitError initContext(vr::IVRDriverContext* context) override
    {
        return vr::InitWatchdogDriverContext(context);
    }

    void cleanupContext() override { vr::CleanupDriverContext(); }
};

// ---------------------------------------------------------------- providers ----

// Every entry point offers the log a stream; Logger keeps the first usable one.
spike::Logger& openedLog()
{
    spike::openProcessLog(log(), "driver-spike");
    return log();
}

class SpikeServerProvider final : public vr::IServerTrackedDeviceProvider
{
public:
    vr::EVRInitError Init(vr::IVRDriverContext* context) override
    {
        openedLog();
        return server_.init(context);
    }

    void Cleanup() override { server_.cleanup(); }

    const char* const* GetInterfaceVersions() override { return vr::k_InterfaceVersions; }

    void RunFrame() override { server_.runFrame(); }

    bool ShouldBlockStandbyMode() override { return server_.shouldBlockStandbyMode(); }

    void EnterStandby() override { server_.enterStandby(); }

    void LeaveStandby() override { server_.leaveStandby(); }

private:
    OpenVrServerEnvironment environment_;
    spike::SpikeServer server_{log(), observer(), environment_};
};

class SpikeWatchdogProvider final : public vr::IVRWatchdogProvider
{
public:
    vr::EVRInitError Init(vr::IVRDriverContext* context) override
    {
        openedLog();
        return watchdog_.init(context);
    }

    void Cleanup() override { watchdog_.cleanup(); }

private:
    OpenVrWatchdogEnvironment environment_;
    spike::SpikeWatchdog watchdog_{log(), environment_};
};

spike::FactoryProviders& providers()
{
    static spike::FactoryProviders* instance = new spike::FactoryProviders{
        static_cast<vr::IServerTrackedDeviceProvider*>(new SpikeServerProvider()),
        static_cast<vr::IVRWatchdogProvider*>(new SpikeWatchdogProvider())};
    return *instance;
}
} // namespace

extern "C" __declspec(dllexport) void* HmdDriverFactory(const char* interfaceName, int* returnCode)
{
    return spike::serveFactoryRequest(openedLog(), interfaceName, returnCode, providers());
}
