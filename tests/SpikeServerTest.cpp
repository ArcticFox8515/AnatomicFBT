// Tests for the throwaway step-1 spike's provider lifecycle (doc/driver-plan.md).
//
// SpikeServer / SpikeWatchdog are what SteamVR's Init / Cleanup / RunFrame / standby
// calls actually run, and every failure they have to survive lives here rather than
// in a live session: the driver context refusing to initialize, MinHook refusing to
// initialize, and anything at all throwing on the way through. A throw reaching
// vrserver.exe kills the user's whole VR session, so "it did not throw" is asserted,
// not assumed.

#include "spike/SpikeLog.h"
#include "spike/SpikeObserver.h"
#include "spike/SpikeServer.h"

#include "FakePipe.h"
#include "link/MessageChannel.h"
#include "link/Pipe.h"
#include "link/Protocol.h"

#include <gtest/gtest.h>

#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
// Builds a complete frame (8-byte header + payload) for feeding the read side.
std::vector<std::uint8_t> frame(link::MessageType type, const std::uint8_t* payload,
                                std::size_t size)
{
    std::vector<std::uint8_t> out;
    const std::uint32_t len = static_cast<std::uint32_t>(size);
    out.push_back(static_cast<std::uint8_t>(len & 0xFF));
    out.push_back(static_cast<std::uint8_t>((len >> 8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((len >> 16) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((len >> 24) & 0xFF));
    const std::uint16_t t = static_cast<std::uint16_t>(type);
    out.push_back(static_cast<std::uint8_t>(t & 0xFF));
    out.push_back(static_cast<std::uint8_t>((t >> 8) & 0xFF));
    out.push_back(0);
    out.push_back(0);
    out.insert(out.end(), payload, payload + size);
    return out;
}

class NoInterfaceHooks : public spike::InterfaceHooks
{
public:
    void hookServerDriverHost(void*) override {}
};

class FakeServerEnvironment : public spike::ServerEnvironment
{
public:
    vr::EVRInitError initContext(vr::IVRDriverContext* context) override
    {
        contextInits.push_back(context);
        return contextError;
    }

    void cleanupContext() override { steps.emplace_back("cleanupContext"); }

    void routeLogToDriverLog() override
    {
        steps.emplace_back("routeLog");
        if (throwOnRouteLog)
            throw std::runtime_error("IVRDriverLog exploded");
    }

    const char* initHookLibrary() override
    {
        steps.emplace_back("initHooks");
        return hookLibraryError;
    }

    void shutdownHookLibrary() override { steps.emplace_back("shutdownHooks"); }

    std::string modulePath() override { return "C:\\build\\driver_00trackingcorrector.dll"; }

    unsigned long processId() override { return 4242; }

    spike::DeviceProperties* properties() override { return nullptr; }

    void hookDriverContext(vr::IVRDriverContext*) override
    {
        steps.emplace_back("hookContext");
    }

    void hookServerDriverHost() override { steps.emplace_back("hookHost"); }

    void removeHooks() override { steps.emplace_back("removeHooks"); }

    vr::EVRInitError contextError = vr::VRInitError_None;
    const char* hookLibraryError = nullptr;
    bool throwOnRouteLog = false;
    std::vector<vr::IVRDriverContext*> contextInits;
    std::vector<std::string> steps;
};

class FakeWatchdogEnvironment : public spike::WatchdogEnvironment
{
public:
    vr::EVRInitError initContext(vr::IVRDriverContext*) override
    {
        ++contextInits;
        return contextError;
    }

    void cleanupContext() override { ++contextCleanups; }

    vr::EVRInitError contextError = vr::VRInitError_None;
    int contextInits = 0;
    int contextCleanups = 0;
};

class SpikeServerTest : public ::testing::Test
{
protected:
    SpikeServerTest() : observer_(logger_, hooks_, [] { return 0.0; }, channel_)
    {
        logger_.setSink([this](const char* message) { lines_.emplace_back(message); });
    }

    bool logged(const std::string& needle) const
    {
        for (const std::string& line : lines_)
            if (line.find(needle) != std::string::npos)
                return true;
        return false;
    }

    std::vector<std::string> lines_;
    spike::Logger logger_;
    NoInterfaceHooks hooks_;
    link::MessageChannel channel_{logger_, [] { return nullptr; }};
    spike::SpikeObserver observer_{logger_, hooks_, [] { return 0.0; }, channel_};
    FakeServerEnvironment environment_;
    spike::SpikeServer server_{logger_, observer_, environment_, channel_};
};

TEST_F(SpikeServerTest, InitHooksInTheOrderTheContextForcesOnUs)
{
    vr::IVRDriverContext* const context = reinterpret_cast<vr::IVRDriverContext*>(0x1234);
    EXPECT_EQ(server_.init(context), vr::VRInitError_None);

    ASSERT_EQ(environment_.contextInits.size(), 1u);
    EXPECT_EQ(environment_.contextInits[0], context);

    // The context caches IVRServerDriverHost & co. *before* our GetGenericInterface
    // detour can exist, so those have to be hooked eagerly from the pointers it already
    // holds — and the detour has to be in place before that, for everything after.
    const std::vector<std::string> expected{"routeLog", "initHooks", "hookContext", "hookHost"};
    EXPECT_EQ(environment_.steps, expected);

    EXPECT_TRUE(logged("=== TrackingCorrector spike driver: server Init ==="));
    EXPECT_TRUE(logged("module: C:\\build\\driver_00trackingcorrector.dll pid=4242 "
                       "sizeof(DriverPose_t)="));
    EXPECT_TRUE(logged("Init complete"));
}

TEST_F(SpikeServerTest, AFailedDriverContextIsReportedAndNothingIsHooked)
{
    // SteamVR's own error is passed back unchanged: it is the only thing that says why.
    environment_.contextError = vr::VRInitError_Init_InterfaceNotFound;
    EXPECT_EQ(server_.init(nullptr), vr::VRInitError_Init_InterfaceNotFound);
    EXPECT_TRUE(environment_.steps.empty());
    EXPECT_TRUE(logged("InitServerDriverContext failed (105)"));
}

TEST_F(SpikeServerTest, WithoutMinHookTheDriverFailsToLoadRatherThanRunBlind)
{
    // Running with no hooks would look like a working driver that reports nothing.
    environment_.hookLibraryError = "MH_ERROR_MEMORY_ALLOC";
    EXPECT_EQ(server_.init(nullptr), vr::VRInitError_Driver_Failed);

    const std::vector<std::string> expected{"routeLog", "initHooks"};
    EXPECT_EQ(environment_.steps, expected);
    EXPECT_TRUE(logged("MH_Initialize failed (MH_ERROR_MEMORY_ALLOC) — no hooks installed"));
    EXPECT_FALSE(logged("Init complete"));
}

TEST_F(SpikeServerTest, AThrowDuringInitFailsTheLoadInsteadOfReachingVrserver)
{
    environment_.throwOnRouteLog = true;
    EXPECT_EQ(server_.init(nullptr), vr::VRInitError_Driver_Failed);
    EXPECT_TRUE(logged("Init threw — failing the driver load"));
    EXPECT_FALSE(logged("Init complete"));
}

TEST_F(SpikeServerTest, CleanupUnhooksBeforeTearingDownMinHookAndTheContext)
{
    ASSERT_EQ(server_.init(nullptr), vr::VRInitError_None);
    environment_.steps.clear();
    lines_.clear();

    server_.cleanup();

    // Order matters: hooks must be gone before MinHook goes, and the context must
    // outlive both.
    const std::vector<std::string> expected{"removeHooks", "shutdownHooks", "cleanupContext"};
    EXPECT_EQ(environment_.steps, expected);
    EXPECT_TRUE(logged("=== TrackingCorrector spike driver: Cleanup ==="));
}

TEST_F(SpikeServerTest, CleanupStillTearsDownTheContextIfObservationThrows)
{
    // Whatever went wrong, vrserver's context must be released and no exception may
    // leave Cleanup.
    ASSERT_EQ(server_.init(nullptr), vr::VRInitError_None);
    logger_.setSink([](const char*) { throw std::runtime_error("sink exploded"); });
    environment_.steps.clear();

    server_.cleanup();
    const std::vector<std::string> expected{"cleanupContext"};
    EXPECT_EQ(environment_.steps, expected);
}

TEST_F(SpikeServerTest, RunFrameAndStandbyAreObservedAndCannotThrow)
{
    ASSERT_EQ(server_.init(nullptr), vr::VRInitError_None);
    lines_.clear();

    server_.runFrame();

    server_.enterStandby();
    server_.leaveStandby();
    EXPECT_TRUE(logged("EnterStandby"));
    EXPECT_TRUE(logged("LeaveStandby"));

    // Same three entry points with everything downstream throwing: SteamVR must not
    // notice.
    logger_.setSink([](const char*) { throw std::runtime_error("sink exploded"); });
    server_.runFrame();
    server_.enterStandby();
    server_.leaveStandby();
}

TEST_F(SpikeServerTest, StandbyIsNeverBlocked)
{
    // An observation-only driver has no reason to keep the user's headset awake; the
    // answer SteamVR gets is the provider's, not a constant inside the DLL.
    EXPECT_FALSE(server_.shouldBlockStandbyMode());
}

// ---------------------------------------------------------- channel wiring ----

class SpikeServerPipeTest : public ::testing::Test
{
protected:
    SpikeServerPipeTest() : observer_(logger_, hooks_, [] { return 0.0; }, channel_),
                            server_(logger_, observer_, environment_, channel_)
    {
        logger_.setSink([this](const char* message) { lines_.emplace_back(message); });
    }

    std::vector<std::string> lines_;
    spike::Logger logger_;
    NoInterfaceHooks hooks_;
    link_test::FakePipe pipe_;
    link::MessageChannel channel_{logger_, link_test::borrowPipeFactory(pipe_)};
    spike::SpikeObserver observer_{logger_, hooks_, [] { return 0.0; }, channel_};
    FakeServerEnvironment environment_;
    spike::SpikeServer server_{logger_, observer_, environment_, channel_};
};

TEST_F(SpikeServerPipeTest, OnPoseForwardsThroughTheChannelAfterInit)
{
    ASSERT_EQ(server_.init(nullptr), vr::VRInitError_None);

    server_.runFrame();  // connects the pipe (factory hands out the FakePipe)

    vr::DriverPose_t pose{};
    pose.poseIsValid = true;
    pose.deviceIsConnected = true;
    observer_.onPose(3, pose, sizeof(pose));

    // The pose was forwarded: the pipe received a DevicePose frame.
    const std::size_t oneFrame = sizeof(link::DevicePose) + 8;  // 8-byte header
    EXPECT_EQ(pipe_.written.size(), oneFrame);
}

// --------------------------------------------------------------- watchdog ----

class SpikeWatchdogTest : public ::testing::Test
{
protected:
    SpikeWatchdogTest()
    {
        logger_.setSink([this](const char* message) { lines_.emplace_back(message); });
    }

    bool logged(const std::string& needle) const
    {
        for (const std::string& line : lines_)
            if (line.find(needle) != std::string::npos)
                return true;
        return false;
    }

    std::vector<std::string> lines_;
    spike::Logger logger_;
    FakeWatchdogEnvironment environment_;
    spike::SpikeWatchdog watchdog_{logger_, environment_};
};

TEST_F(SpikeWatchdogTest, WatchdogInitInstallsNothing)
{
    // vrwatchdog.exe loads the same DLL; the spike only wants to know that it does.
    EXPECT_EQ(watchdog_.init(nullptr), vr::VRInitError_None);
    EXPECT_EQ(environment_.contextInits, 1);
    EXPECT_TRUE(logged("watchdog Init (this process installs no hooks)"));

    watchdog_.cleanup();
    EXPECT_EQ(environment_.contextCleanups, 1);
    EXPECT_TRUE(logged("watchdog Cleanup"));
}

TEST_F(SpikeWatchdogTest, AFailedWatchdogContextIsPassedBack)
{
    environment_.contextError = vr::VRInitError_Init_InterfaceNotFound;
    EXPECT_EQ(watchdog_.init(nullptr), vr::VRInitError_Init_InterfaceNotFound);
    EXPECT_TRUE(logged("watchdog InitWatchdogDriverContext failed (105)"));
}

// ---------------------------------------------------------------- factory ----

TEST(SpikeFactory, TheTwoProvidersSteamVrAsksForAreRecognized)
{
    EXPECT_EQ(spike::classifyFactoryRequest(vr::IServerTrackedDeviceProvider_Version),
              spike::FactoryRequest::Server);
    EXPECT_EQ(spike::classifyFactoryRequest(vr::IVRWatchdogProvider_Version),
              spike::FactoryRequest::Watchdog);
}

TEST(SpikeFactory, AnythingElseIncludingNullIsUnknown)
{
    // nullptr is not supposed to happen; dereferencing it would take vrserver down
    // during load, before any log exists to say why.
    EXPECT_EQ(spike::classifyFactoryRequest(nullptr), spike::FactoryRequest::Unknown);
    EXPECT_EQ(spike::classifyFactoryRequest(""), spike::FactoryRequest::Unknown);
    EXPECT_EQ(spike::classifyFactoryRequest("IVRWatchdogProvider_002"),
              spike::FactoryRequest::Unknown);
    EXPECT_EQ(spike::classifyFactoryRequest("IDoesNotExist_001"),
              spike::FactoryRequest::Unknown);
}

// HmdDriverFactory's body. It lives here rather than in the DLL because the DLL is
// compiled into no test binary — SpikeDriverTest reaches it only through LoadLibrary,
// which proves the export works, not that these decisions are right.
class SpikeServeFactoryRequest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        logger_.setSink([this](const char* message) { lines_.emplace_back(message); });
    }

    bool logged(const std::string& needle) const
    {
        for (const std::string& line : lines_)
            if (line.find(needle) != std::string::npos)
                return true;
        return false;
    }

    int serverObject_ = 0;
    int watchdogObject_ = 0;
    spike::FactoryProviders providers_{&serverObject_, &watchdogObject_};

    std::vector<std::string> lines_;
    spike::Logger logger_;
};

TEST_F(SpikeServeFactoryRequest, ServesTheServerProvider)
{
    int code = 12345;
    EXPECT_EQ(spike::serveFactoryRequest(logger_, vr::IServerTrackedDeviceProvider_Version, &code,
                                         providers_),
              &serverObject_);
    // A recognized interface must leave the return code alone.
    EXPECT_EQ(code, 12345);
    EXPECT_TRUE(logged("HmdDriverFactory(\"IServerTrackedDeviceProvider_"));
}

TEST_F(SpikeServeFactoryRequest, ServesTheWatchdogProvider)
{
    int code = 12345;
    EXPECT_EQ(
        spike::serveFactoryRequest(logger_, vr::IVRWatchdogProvider_Version, &code, providers_),
        &watchdogObject_);
    EXPECT_EQ(code, 12345);
}

TEST_F(SpikeServeFactoryRequest, RejectsAnUnknownInterfaceThroughTheReturnCode)
{
    int code = 0;
    EXPECT_EQ(spike::serveFactoryRequest(logger_, "IDoesNotExist_001", &code, providers_), nullptr);
    EXPECT_EQ(code, vr::VRInitError_Init_InterfaceNotFound);
    EXPECT_TRUE(logged("HmdDriverFactory(\"IDoesNotExist_001\")"));
}

TEST_F(SpikeServeFactoryRequest, SurvivesANullInterfaceNameAndANullReturnCode)
{
    // Neither is supposed to happen; either one dereferenced would take vrserver down
    // during load, before there is a log to say why.
    EXPECT_EQ(spike::serveFactoryRequest(logger_, nullptr, nullptr, providers_), nullptr);
    EXPECT_TRUE(logged("HmdDriverFactory(\"(null)\")"));
    EXPECT_EQ(spike::serveFactoryRequest(logger_, "IDoesNotExist_001", nullptr, providers_),
              nullptr);
}
} // namespace
