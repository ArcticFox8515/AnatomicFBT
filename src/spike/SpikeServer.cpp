#include "SpikeServer.h"

#include "SpikeGuard.h"

#include <cstring>

namespace spike
{
FactoryRequest classifyFactoryRequest(const char* interfaceName)
{
    if (!interfaceName)
        return FactoryRequest::Unknown;
    if (std::strcmp(vr::IServerTrackedDeviceProvider_Version, interfaceName) == 0)
        return FactoryRequest::Server;
    if (std::strcmp(vr::IVRWatchdogProvider_Version, interfaceName) == 0)
        return FactoryRequest::Watchdog;
    return FactoryRequest::Unknown;
}

bool componentWasCreated(vr::EVRInputError result, const vr::VRInputComponentHandle_t* handle)
{
    return result == vr::VRInputError_None && handle != nullptr;
}

void* serveFactoryRequest(Logger& logger, const char* interfaceName, int* returnCode,
                          const FactoryProviders& providers)
{
    logger.logf("HmdDriverFactory(\"%s\")", interfaceName ? interfaceName : "(null)");

    switch (classifyFactoryRequest(interfaceName))
    {
    case FactoryRequest::Server: return providers.server;
    case FactoryRequest::Watchdog: return providers.watchdog;
    case FactoryRequest::Unknown: break;
    }

    // SteamVR asks for interfaces we do not implement as a matter of course, so this is
    // not an error worth failing on — but the out parameter is optional.
    if (returnCode)
        *returnCode = vr::VRInitError_Init_InterfaceNotFound;
    return nullptr;
}

SpikeServer::SpikeServer(Logger& logger, SpikeObserver& observer,
                         ServerEnvironment& environment)
    : log_(logger), observer_(observer), environment_(environment)
{
}

vr::EVRInitError SpikeServer::init(vr::IVRDriverContext* context)
{
    // Note: InitServerDriverContext itself fetches IVRServerDriverHost / IVRSettings /
    // IVRProperties / IVRDriverLog through GetGenericInterface and caches them, i.e.
    // before our detour exists — which is why the interfaces we care about are also
    // hooked eagerly below, not only from the detour.
    const vr::EVRInitError contextError = environment_.initContext(context);
    if (contextError != vr::VRInitError_None)
    {
        log_.logf("InitServerDriverContext failed (%d)", static_cast<int>(contextError));
        return contextError;
    }

    try
    {
        environment_.routeLogToDriverLog();

        log_.logf("=== TrackingCorrector spike driver: server Init ===");
        log_.logf("log file: %s", log_.filePath().c_str());
        log_.logf("module: %s pid=%lu sizeof(DriverPose_t)=%zu",
                  environment_.modulePath().c_str(), environment_.processId(),
                  sizeof(vr::DriverPose_t));

        if (const char* hookError = environment_.initHookLibrary())
        {
            log_.logf("MH_Initialize failed (%s) — no hooks installed", hookError);
            return vr::VRInitError_Driver_Failed;
        }

        observer_.setProperties(environment_.properties());
        observer_.onInit();

        // Detour on the raw context pointer: every *other* driver's interface
        // acquisition passes through the same vrserver implementation, so this is how
        // devices belonging to drivers loaded after us become visible.
        environment_.hookDriverContext(context);

        environment_.hookServerDriverHost();
        // Triggers the detour above (IVRDriverInput is not fetched by the context init
        // macro), which installs the input hooks; the eager call is a fallback and is a
        // no-op when the detour already did it.
        environment_.hookDriverInput();

        log_.logf("Init complete");
        return vr::VRInitError_None;
    }
    catch (...)
    {
        // An exception must never leave a provider entry point, and a driver that
        // failed halfway through hooking must not pretend it is running.
        log_.logf("Init threw — failing the driver load");
        return vr::VRInitError_Driver_Failed;
    }
}

void SpikeServer::cleanup()
{
    runGuarded([&] {
        log_.logf("=== TrackingCorrector spike driver: Cleanup ===");
        observer_.onCleanup();
        observer_.setProperties(nullptr);
        environment_.removeHooks();
        environment_.shutdownHookLibrary();
    });
    environment_.cleanupContext();
    log_.close();
}

void SpikeServer::runFrame()
{
    runGuarded([&] { observer_.onRunFrame(); });
}

void SpikeServer::enterStandby()
{
    runGuarded([&] { log_.logf("EnterStandby"); });
}

void SpikeServer::leaveStandby()
{
    runGuarded([&] { log_.logf("LeaveStandby"); });
}

bool SpikeServer::shouldBlockStandbyMode() const
{
    return false;
}

SpikeWatchdog::SpikeWatchdog(Logger& logger, WatchdogEnvironment& environment)
    : log_(logger), environment_(environment)
{
}

vr::EVRInitError SpikeWatchdog::init(vr::IVRDriverContext* context)
{
    const vr::EVRInitError contextError = environment_.initContext(context);
    if (contextError != vr::VRInitError_None)
    {
        log_.logf("watchdog InitWatchdogDriverContext failed (%d)",
                  static_cast<int>(contextError));
        return contextError;
    }
    log_.logf("watchdog Init (this process installs no hooks)");
    return vr::VRInitError_None;
}

void SpikeWatchdog::cleanup()
{
    log_.logf("watchdog Cleanup");
    environment_.cleanupContext();
    log_.close();
}
} // namespace spike
