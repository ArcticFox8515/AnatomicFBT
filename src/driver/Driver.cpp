// The SteamVR driver DLL (doc/driver-plan.md). This driver adds no devices, modifies
// no poses and forwards every hooked call unchanged: it observes vrserver's device
// surface and publishes each pose over the link channel so the app can consume
// driver-side poses. Pose modification and virtual trackers come later, but the pose
// hook is installed now — it is both the source of unmodified poses and the place
// corrections will be applied.
//
// THIS FILE IS THE ADAPTER, AND NOTHING ELSE.
//
// It is compiled only into driver_00trackingcorrector, never into the test
// executable, so nothing in it can run in a unit test — DriverTest reaches it
// through LoadLibrary, which is an integration test and proves nothing under the bar
// the test suite holds the rest of the code to. Therefore every function here is one
// instruction: a call into DriverLib, or a Win32/openvr/MinHook/spdlog call whose
// arguments it merely marshals. The two-line functions are exactly the ones that create
// an implementation object and then call a method on it: the leaked-singleton
// accessors, the spdlog logger factory, and the provider Init entry points. There is no
// branch, no loop, no comparison and no arithmetic in this file: if you find yourself
// adding one, it belongs in DriverLib with a test.
//
// Hard rule: no exception may leave a hook or a provider entry point — an exception
// escaping into vrserver.exe kills SteamVR. runGuarded() sits inside the DriverLib
// forwarders (DriverHooks.cpp) and inside Server, i.e. on the other side of
// every call below.
//
// Buttons / input are NOT captured by this driver: the live run showed zero calls on
// the hooked IVRDriverInput_003 (every controller asked for _004), and PollNextEvent
// proved unreliable as a substitute. Input is captured by a separate background
// client app instead (doc/driver-plan.md "Buttons").

#include "DriverEnvironment.h"
#include "DriverHooks.h"
#include "Hooks.h"
#include "LogFile.h"
#include "Observer.h"
#include "Server.h"

#include "pipe/Win32Pipe.h"

#include <openvr_driver.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <windows.h>

#include <MinHook.h>

#include <chrono>
#include <memory>
#include <string>

namespace
{
using link::log;

// Forward declaration: OpenVrServerEnvironment::routeLogToDriverLog composes the
// spdlog file sink with the IVRDriverLog sink, and needs it before the logger it is
// built from (created further down, next to the providers) is defined.
link::LogSink driverFileSink();

// --------------------------------------------------------------- MinHook glue ----

class MinHookApi final : public driver::HookApi
{
public:
    int initialize() override { return MH_Initialize(); }

    void shutdown() override { MH_Uninitialize(); }

    int alreadyInitializedStatus() override { return MH_ERROR_ALREADY_INITIALIZED; }

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

class Win32ModuleApi final : public driver::ModuleApi
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

class Win32ProcessApi final : public driver::ProcessApi
{
public:
    unsigned long environmentVariable(const char* name, char* buffer,
                                      unsigned long size) override
    {
        return GetEnvironmentVariableA(name, buffer, size);
    }

    unsigned long executablePath(char* buffer, unsigned long size) override
    {
        return GetModuleFileNameA(nullptr, buffer, size);
    }
};

Win32ProcessApi& processApi()
{
    static Win32ProcessApi* instance = new Win32ProcessApi();
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
// module may define one. Each is a single call into the DriverLib forwarder that owns
// the actual behaviour (forward unchanged, then observe).

void* detourGetGenericInterface(vr::IVRDriverContext* self, const char* version,
                                vr::EVRInitError* error);
bool detourTrackedDeviceAdded(vr::IVRServerDriverHost* self, const char* serial,
                              vr::ETrackedDeviceClass deviceClass,
                              vr::ITrackedDeviceServerDriver* driver);
void detourTrackedDevicePoseUpdated(vr::IVRServerDriverHost* self, uint32_t index,
                                    const vr::DriverPose_t& pose, uint32_t poseStructSize);

// The hook set and the observer are process-wide and deliberately leaked, like the
// rest of the state here: a hook thread may still be inside a detour when the DLL's
// static destructors would otherwise run.

link::MessageChannel& channel()
{
    static link::MessageChannel* instance = new link::MessageChannel(
        log(), [] { return std::make_shared<link::Win32ServerPipe>(link::kDriverPipeName); });
    return *instance;
}

driver::DriverHookSet& hooks()
{
    static driver::DriverHookSet* instance = new driver::DriverHookSet(
        hookApi(), log(),
        driver::DriverDetours{reinterpret_cast<void*>(&detourGetGenericInterface),
                             reinterpret_cast<void*>(&detourTrackedDeviceAdded),
                             reinterpret_cast<void*>(&detourTrackedDevicePoseUpdated)});
    return *instance;
}

driver::Observer& observer()
{
    static driver::Observer* instance =
        new driver::Observer(log(), hooks(), &nowSeconds, channel());
    return *instance;
}

void* detourGetGenericInterface(vr::IVRDriverContext* self, const char* version,
                                vr::EVRInitError* error)
{
    return driver::observeGetGenericInterface(hooks(), observer(), self, version, error);
}

bool detourTrackedDeviceAdded(vr::IVRServerDriverHost* self, const char* serial,
                              vr::ETrackedDeviceClass deviceClass,
                              vr::ITrackedDeviceServerDriver* driver)
{
    return driver::observeTrackedDeviceAdded(hooks(), observer(), self, serial, deviceClass,
                                             driver);
}

void detourTrackedDevicePoseUpdated(vr::IVRServerDriverHost* self, uint32_t index,
                                    const vr::DriverPose_t& pose, uint32_t poseStructSize)
{
    driver::observeTrackedDevicePoseUpdated(hooks(), observer(), self, index, pose,
                                           poseStructSize);
}

// ------------------------------------------------- vrserver environment glue ----

class OpenVrServerEnvironment final : public driver::ServerEnvironment
{
public:
    vr::EVRInitError initContext(vr::IVRDriverContext* context) override
    {
        return vr::InitServerDriverContext(context);
    }

    void cleanupContext() override { vr::CleanupDriverContext(); }

    void routeLogToDriverLog() override
    {
        link::log().setSink(
            link::compositeSink(driverFileSink(), driver::driverLogSink(&vr::VRDriverLog)));
    }

    const char* initHookLibrary() override { return driver::initializeHookLibrary(hookApi()); }

    void shutdownHookLibrary() override { hookApi().shutdown(); }

    std::string modulePath() override
    {
        return driver::modulePathOfAddress(moduleApi(),
                                           reinterpret_cast<void*>(&detourGetGenericInterface));
    }

    unsigned long processId() override { return GetCurrentProcessId(); }

    driver::DeviceProperties* properties() override { return properties_.orNullIfUnavailable(); }

    void hookDriverContext(vr::IVRDriverContext* context) override
    {
        hooks().hookDriverContext(context);
    }

    void hookServerDriverHost() override { hooks().hookServerDriverHost(vr::VRServerDriverHost()); }

    void removeHooks() override { hooks().removeAll(); }

private:
    driver::OpenVrProperties properties_{&vr::VRProperties};
};

class OpenVrWatchdogEnvironment final : public driver::WatchdogEnvironment
{
public:
    vr::EVRInitError initContext(vr::IVRDriverContext* context) override
    {
        return vr::InitWatchdogDriverContext(context);
    }

    void cleanupContext() override { vr::CleanupDriverContext(); }
};

// ---------------------------------------------------------------- providers ----

// The spdlog logger owns the file, timestamps, buffering and flushing — including
// creating the directory the path names. Created once, before any hook is installed.
// The link Logger's sink is initially spdlog-only (so lines logged before
// InitServerDriverContext — when vr::VRDriverLog() is still null — reach the file);
// routeLogToDriverLog replaces it with a composite that also fans out to IVRDriverLog
// once the context is up.
std::shared_ptr<spdlog::logger> makeFileLogger(const char* name, const std::string& path)
{
    auto logger = spdlog::basic_logger_mt(name, path);
    logger->set_pattern("%v");
    return logger;
}

std::shared_ptr<spdlog::logger> driverLogger()
{
    static std::shared_ptr<spdlog::logger> logger = makeFileLogger(
        "driver", driver::processLogPath(processApi(), driver::kDriverLogPrefix));
    return logger;
}

link::LogSink driverFileSink()
{
    return [logger = driverLogger()](const char* message) { logger->info(message); };
}

link::Logger& openedLog()
{
    static link::Logger& instance = link::loggingTo(log(), driverFileSink());
    return instance;
}

class ServerProvider final : public vr::IServerTrackedDeviceProvider
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
    driver::Server server_{log(), observer(), environment_, channel()};
};

class WatchdogProvider final : public vr::IVRWatchdogProvider
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
    driver::Watchdog watchdog_{log(), environment_};
};

driver::FactoryProviders& providers()
{
    static driver::FactoryProviders* instance = new driver::FactoryProviders{
        static_cast<vr::IServerTrackedDeviceProvider*>(new ServerProvider()),
        static_cast<vr::IVRWatchdogProvider*>(new WatchdogProvider())};
    return *instance;
}
} // namespace

extern "C" __declspec(dllexport) void* HmdDriverFactory(const char* interfaceName, int* returnCode)
{
    return driver::serveFactoryRequest(openedLog(), interfaceName, returnCode, providers());
}
