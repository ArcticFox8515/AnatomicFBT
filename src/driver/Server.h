#pragma once

// The provider lifecycle (doc/driver-plan.md).
//
// The two IServerTrackedDeviceProvider / IVRWatchdogProvider implementations in
// Driver.cpp are pure forwarders into these classes, and everything they need
// from vrserver, MinHook or Win32 goes through `ServerEnvironment` /
// `WatchdogEnvironment`. That is what makes the failure paths (context init failed,
// MinHook init failed, Init threw) reachable from a unit test instead of only from a
// live SteamVR session.

#include "link/Log.h"
#include "Observer.h"
#include "VirtualTrackers.h"

#include "link/MessageChannel.h"

#include <openvr_driver.h>

#include <string>

namespace driver
{
// Which provider HmdDriverFactory was asked for. Pure decision, tested directly —
// including the nullptr name SteamVR is not supposed to pass but might.
enum class FactoryRequest
{
    Server,
    Watchdog,
    Unknown,
};

FactoryRequest classifyFactoryRequest(const char* interfaceName);

// The two provider objects the DLL owns, as opaque pointers: only the DLL may cast them
// to vr::IServerTrackedDeviceProvider* / vr::IVRWatchdogProvider*, because only the DLL
// defines the classes. Serving them is a decision, so it lives here.
struct FactoryProviders
{
    void* server = nullptr;
    void* watchdog = nullptr;
};

// HmdDriverFactory's whole body: log the request, answer with the matching provider, and
// on an unrecognized interface report VRInitError_Init_InterfaceNotFound through
// returnCode (which SteamVR is allowed to pass as nullptr).
void* serveFactoryRequest(link::Logger& logger, const char* interfaceName, int* returnCode,
                          const FactoryProviders& providers);

class ServerEnvironment
{
public:
    virtual ~ServerEnvironment() = default;

    // vr::InitServerDriverContext / VR_CLEANUP_SERVER_DRIVER_CONTEXT.
    virtual vr::EVRInitError initContext(vr::IVRDriverContext* context) = 0;
    virtual void cleanupContext() = 0;

    // Adds IVRDriverLog (SteamVR's vrserver.txt) as a second destination for the
    // driver logger's sink, alongside the spdlog file destination the adapter installed.
    // Runs after initContext so vr::VRDriverLog() is valid when the sink calls it.
    virtual void routeLogToDriverLog() = 0;

    // MH_Initialize: nullptr on success, else the status text for the log.
    virtual const char* initHookLibrary() = 0;
    virtual void shutdownHookLibrary() = 0;

    virtual std::string modulePath() = 0;
    virtual unsigned long processId() = 0;

    // nullptr when vrserver has no IVRProperties for us.
    virtual DeviceProperties* properties() = 0;

    virtual void hookDriverContext(vr::IVRDriverContext* context) = 0;
    virtual void hookServerDriverHost() = 0;
    virtual void removeHooks() = 0;
};

class Server
{
public:
    Server(link::Logger& logger, Observer& observer, VirtualTrackerProvider& virtualTrackers,
           ServerEnvironment& environment, link::MessageChannel& channel);

    vr::EVRInitError init(vr::IVRDriverContext* context);
    void cleanup();
    void runFrame();
    void enterStandby();
    void leaveStandby();

    // An observation-only driver has no reason to keep SteamVR awake.
    bool shouldBlockStandbyMode() const;

private:
    link::Logger& log_;
    Observer& observer_;
    VirtualTrackerProvider& virtualTrackers_;
    ServerEnvironment& environment_;
    link::MessageChannel& channel_;
    bool wasConnected_ = false;
};

class WatchdogEnvironment
{
public:
    virtual ~WatchdogEnvironment() = default;

    virtual vr::EVRInitError initContext(vr::IVRDriverContext* context) = 0;
    virtual void cleanupContext() = 0;
};

// vrwatchdog.exe loads the same DLL. It installs no hooks — it exists to prove which
// processes load us.
class Watchdog
{
public:
    Watchdog(link::Logger& logger, WatchdogEnvironment& environment);

    vr::EVRInitError init(vr::IVRDriverContext* context);
    void cleanup();

private:
    link::Logger& log_;
    WatchdogEnvironment& environment_;
};
} // namespace driver
