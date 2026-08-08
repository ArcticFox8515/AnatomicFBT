// Tests for the driver's observation logic (doc/driver-plan.md).
//
// Observer is the whole decision surface of the driver, and it is driven here
// directly: no DLL, no MinHook, no vrserver, no clock, no filesystem. Its four
// dependencies (logger, hook installer, property reader, clock) are fakes, so every
// branch — including the ones a live SteamVR session would only show us by accident
// (a truncated pose struct, a device whose serial changed, an unhookable interface
// version) — is reachable and deterministic.
//
// Button/input capture is NOT part of the driver DLL: a separate background client
// app handles input, so this observer never polls VR events and there is no
// EventPoller seam to fake here.

#include "driver/Guard.h"
#include "link/Log.h"
#include "driver/Names.h"
#include "driver/Observer.h"

#include "FakePipe.h"
#include "link/MessageChannel.h"
#include "link/Protocol.h"

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace
{
constexpr vr::PropertyContainerHandle_t kContainerBase = 5000;
constexpr uint32_t kTracker = 3;

class FakeInterfaceHooks : public driver::InterfaceHooks
{
public:
    void hookServerDriverHost(void* host) override { hostHooks.push_back(host); }

    std::vector<void*> hostHooks;
};

class FakeProperties : public driver::DeviceProperties
{
public:
    struct Device
    {
        std::string serial;
        std::string model;
        std::string trackingSystem;
        int32_t deviceClass = vr::TrackedDeviceClass_Invalid;
        int32_t roleHint = vr::TrackedControllerRole_Invalid;
        bool readable = true; // false = container exists, device does not (yet)
    };

    void add(uint32_t index, Device device) { devices[index] = std::move(device); }

    vr::PropertyContainerHandle_t container(uint32_t deviceIndex) override
    {
        ++containerLookups;
        if (throwOnRead)
            throw std::runtime_error("vrserver blew up");
        // As in vrserver: a container exists for every index; only indices with a
        // device answer property reads.
        return devices.count(deviceIndex) ? kContainerBase + deviceIndex
                                          : vr::k_ulInvalidPropertyContainer;
    }

    bool stringProperty(vr::PropertyContainerHandle_t container,
                        vr::ETrackedDeviceProperty property, std::string& value) override
    {
        const Device* device = find(container);
        if (!device || !device->readable)
            return false;
        switch (property)
        {
        case vr::Prop_SerialNumber_String: value = device->serial; return true;
        case vr::Prop_ModelNumber_String: value = device->model; return true;
        case vr::Prop_TrackingSystemName_String: value = device->trackingSystem; return true;
        default: return false;
        }
    }

    int32_t int32Property(vr::PropertyContainerHandle_t container,
                          vr::ETrackedDeviceProperty property) override
    {
        const Device* device = find(container);
        if (!device)
            return 0;
        return property == vr::Prop_DeviceClass_Int32 ? device->deviceClass : device->roleHint;
    }

    std::map<uint32_t, Device> devices;
    uint32_t containerLookups = 0;
    bool throwOnRead = false;

private:
    const Device* find(vr::PropertyContainerHandle_t container) const
    {
        const auto it = devices.find(static_cast<uint32_t>(container - kContainerBase));
        return it == devices.end() ? nullptr : &it->second;
    }
};

vr::DriverPose_t makePose(double x = 1.0)
{
    vr::DriverPose_t pose{};
    pose.poseIsValid = true;
    pose.deviceIsConnected = true;
    pose.result = vr::TrackingResult_Running_OK;
    pose.poseTimeOffset = -0.011;
    // Exactly representable values: the log strings must match the test's own math.
    pose.qWorldFromDriverRotation = {0.0, 0.0, 1.0, 0.0}; // 180 deg about Y
    pose.vecWorldFromDriverTranslation[0] = 10.0;
    pose.vecPosition[0] = x;
    pose.vecPosition[1] = 2.0;
    pose.vecPosition[2] = 3.0;
    pose.qRotation = {0.0, 0.0, 1.0, 0.0};
    pose.qDriverFromHeadRotation = {1.0, 0.0, 0.0, 0.0};
    pose.vecDriverFromHeadTranslation[1] = 0.5;
    return pose;
}

class ObserverTest : public ::testing::Test
{
protected:
    ObserverTest() : observer_(logger_, hooks_, [this] { return now_; }, channel_)
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

    void bringUpTracker()
    {
        properties_.add(kTracker, {"LHR-TESTTRACKER", "Vive Tracker", "lighthouse",
                                   vr::TrackedDeviceClass_GenericTracker,
                                   vr::TrackedControllerRole_Invalid});
        observer_.setProperties(&properties_);
        observer_.onPose(kTracker, makePose(), sizeof(vr::DriverPose_t));
        observer_.onRunFrame();
    }

    double now_ = 100.0;
    std::vector<std::string> lines_;
    link::Logger logger_;
    FakeInterfaceHooks hooks_;
    FakeProperties properties_;
    link_test::FakePipe pipe_;
    link::MessageChannel channel_{logger_, link_test::borrowPipeFactory(pipe_)};
    driver::Observer observer_{logger_, hooks_, [this] { return now_; }, channel_};
};

// ------------------------------------------------- GetGenericInterface dispatch ----

TEST_F(ObserverTest, TheInterfacesWeBuildAgainstAreHooked)
{
    int host = 0;
    observer_.onInterfaceRequested(vr::IVRServerDriverHost_Version, &host);

    EXPECT_EQ(hooks_.hostHooks, std::vector<void*>{&host});
    EXPECT_TRUE(logged(std::string("interface \"") + vr::IVRServerDriverHost_Version
                       + "\": hooking"));
}

TEST_F(ObserverTest, AnUnhookableVersionOfAnInterfaceWeNeedIsReportedLoudlyAndNotHooked)
{
    // The dangerous silent case: a version we cannot hook means every device behind it
    // is invisible to us, and the pointer must not be touched (its vtable layout is
    // exactly what we do not know).
    int dummy = 0;
    observer_.onInterfaceRequested("IVRServerDriverHost_005", &dummy);

    EXPECT_TRUE(hooks_.hostHooks.empty());
    EXPECT_TRUE(logged("interface \"IVRServerDriverHost_005\": NOT HOOKED"));
}

TEST_F(ObserverTest, InterfacesWeDoNotNeedAndNullResultsAreJustRecorded)
{
    int dummy = 0;
    observer_.onInterfaceRequested(vr::IVRResources_Version, &dummy);
    // IVRDriverInput is not hooked: input capture moved out of the driver DLL to a
    // separate background client app, so the input interface is "not needed" like
    // IVRResources.
    observer_.onInterfaceRequested(vr::IVRDriverInput_Version, &dummy);
    observer_.onInterfaceRequested("IVRCameraComponent_004", nullptr);
    observer_.onInterfaceRequested(nullptr, nullptr);

    EXPECT_TRUE(logged("interface \"IVRResources_001\": seen, not hooked (not needed)"));
    EXPECT_TRUE(logged(std::string("interface \"") + vr::IVRDriverInput_Version
                       + "\": seen, not hooked (not needed)"));
    EXPECT_TRUE(logged("interface \"IVRCameraComponent_004\": requested, vrserver returned NULL"));
    EXPECT_TRUE(logged("interface \"(null)\": requested, vrserver returned NULL"));
    EXPECT_TRUE(hooks_.hostHooks.empty());
}

TEST_F(ObserverTest, EachVersionStringIsReportedOnce)
{
    // Every driver in the session asks for the same interfaces; the log has to stay
    // readable, and the hook must not be reinstalled per request.
    int host = 0;
    for (int i = 0; i < 3; ++i)
    {
        observer_.onInterfaceRequested(vr::IVRServerDriverHost_Version, &host);
        observer_.onInterfaceRequested(vr::IVRResources_Version, &host);
        observer_.onInterfaceRequested("IVRCameraComponent_004", nullptr);
    }
    EXPECT_EQ(hooks_.hostHooks.size(), 1u);
    EXPECT_EQ(lines_.size(), 3u);
}

// ------------------------------------------------------------ device / pose ----

TEST_F(ObserverTest, PosesForImpossibleDeviceIndicesAreDropped)
{
    observer_.onPose(vr::k_unMaxTrackedDeviceCount, makePose(), sizeof(vr::DriverPose_t));
    observer_.onPose(vr::k_unMaxTrackedDeviceCount + 7, makePose(), sizeof(vr::DriverPose_t));
    EXPECT_TRUE(lines_.empty());
}

// ---------------------------------------------------- RunFrame housekeeping ----

TEST_F(ObserverTest, HousekeepingRunsOnTheFirstFrameThenOncePerSecond)
{
    properties_.add(kTracker, {"LHR-TESTTRACKER", "Vive Tracker", "lighthouse",
                               vr::TrackedDeviceClass_GenericTracker,
                               vr::TrackedControllerRole_Invalid});
    observer_.setProperties(&properties_);

    observer_.onRunFrame();
    EXPECT_TRUE(logged("device 3: class=tracker(3) serial=\"LHR-TESTTRACKER\" container=5003"));

    const uint32_t lookupsAfterFirst = properties_.containerLookups;
    now_ += driver::kHousekeepingSeconds / 2.0;
    observer_.onRunFrame();
    EXPECT_EQ(properties_.containerLookups, lookupsAfterFirst);

    now_ += driver::kHousekeepingSeconds;
    observer_.onRunFrame();
    EXPECT_GT(properties_.containerLookups, lookupsAfterFirst);
}

TEST_F(ObserverTest, WithoutAPropertyReaderNothingIsEnumerated)
{
    observer_.setProperties(nullptr);
    observer_.onRunFrame();
    EXPECT_EQ(properties_.containerLookups, 0u);
    EXPECT_FALSE(logged("device 3:"));
}

TEST_F(ObserverTest, ContainersWithoutADeviceAreSkipped)
{
    properties_.add(1, {"", "", "", vr::TrackedDeviceClass_Invalid,
                        vr::TrackedControllerRole_Invalid, /*readable=*/false});
    observer_.setProperties(&properties_);
    observer_.onRunFrame();

    EXPECT_GT(properties_.containerLookups, 0u);
    EXPECT_FALSE(logged("device 1:"));
}

TEST_F(ObserverTest, MetadataIsReReadOnlyWhenTheSerialChanges)
{
    bringUpTracker();
    lines_.clear();

    now_ += driver::kHousekeepingSeconds;
    observer_.onRunFrame();
    EXPECT_FALSE(logged("device 3: class="));

    properties_.devices[kTracker].serial = "LHR-OTHER";
    properties_.devices[kTracker].deviceClass = vr::TrackedDeviceClass_Controller;
    now_ += driver::kHousekeepingSeconds;
    observer_.onRunFrame();
    EXPECT_TRUE(logged("device 3: class=controller(2) serial=\"LHR-OTHER\""));
}

// -------------------------------------------------------- pose forwarding ----

// Parses the first frame from `written` (8-byte header + DevicePose payload)
// and returns it, or fails the test if the wire does not match.
link::DevicePose parseFirstPose(const std::vector<std::uint8_t>& written)
{
    constexpr std::size_t kHeader = 8;
    EXPECT_GE(written.size(), kHeader + sizeof(link::DevicePose));
    link::DevicePose restored;
    std::memcpy(&restored, written.data() + kHeader, sizeof(restored));
    return restored;
}

TEST_F(ObserverTest, ForwardsAPoseAsTheBCompositionWithClassAndSerial)
{
    bringUpTracker();  // metadata known: tracker, "LHR-TESTTRACKER"; pose stored
    lines_.clear();

    std::vector<link::Message> dummy;
    channel_.receive(dummy);  // construct the pipe

    observer_.onPose(kTracker, makePose(), sizeof(vr::DriverPose_t));

    // One frame on the wire: header (8) + DevicePose (68).
    ASSERT_EQ(pipe_.written.size(), 8u + sizeof(link::DevicePose));
    const link::DevicePose wire = parseFirstPose(pipe_.written);
    EXPECT_EQ(wire.deviceId, kTracker);
    EXPECT_EQ(wire.deviceKind, link::DeviceKind::Tracker);
    EXPECT_EQ(wire.tracking, link::TrackingState::Tracking);
    EXPECT_STREQ(wire.serial, "LHR-TESTTRACKER");

    // The world-space pose is the "B" composition BothCandidateCompositions
    // logs above: with makePose() (x=1), B.pos = {9, 2.5, -3},
    // B.rot = {w=-1, x=0, y=0, z=0} (xyzw: 0, 0, 0, -1).
    EXPECT_FLOAT_EQ(wire.position[0], 9.0f);
    EXPECT_FLOAT_EQ(wire.position[1], 2.5f);
    EXPECT_FLOAT_EQ(wire.position[2], -3.0f);
    EXPECT_FLOAT_EQ(wire.rotation[0], 0.0f);  // x
    EXPECT_FLOAT_EQ(wire.rotation[1], 0.0f);  // y
    EXPECT_FLOAT_EQ(wire.rotation[2], 0.0f);  // z
    EXPECT_FLOAT_EQ(wire.rotation[3], -1.0f); // w
}

TEST_F(ObserverTest, ForwardsAnInvalidPoseAsTrackingLost)
{
    // No pose validation: an invalid pose is still forwarded, but with
    // tracking=Lost so the app drops the device this frame.
    bringUpTracker();
    lines_.clear();

    std::vector<link::Message> dummy;
    channel_.receive(dummy);  // construct the pipe

    vr::DriverPose_t untracked = makePose();
    untracked.poseIsValid = false;
    observer_.onPose(kTracker, untracked, sizeof(vr::DriverPose_t));

    const link::DevicePose wire = parseFirstPose(pipe_.written);
    EXPECT_EQ(wire.tracking, link::TrackingState::Lost);
    EXPECT_EQ(wire.deviceKind, link::DeviceKind::Tracker);
}

TEST_F(ObserverTest, ATruncatedPoseIsNotForwarded)
{
    bringUpTracker();
    lines_.clear();

    std::vector<link::Message> dummy;
    channel_.receive(dummy);  // construct the pipe

    observer_.onPose(kTracker, makePose(), sizeof(vr::DriverPose_t) - 8);
    EXPECT_TRUE(pipe_.written.empty());
}

// -------------------------------------------------------- names and guards ----

TEST(Names, EveryDeviceClassHasALabel)
{
    EXPECT_STREQ(driver::deviceClassName(vr::TrackedDeviceClass_Invalid), "invalid");
    EXPECT_STREQ(driver::deviceClassName(vr::TrackedDeviceClass_HMD), "hmd");
    EXPECT_STREQ(driver::deviceClassName(vr::TrackedDeviceClass_Controller), "controller");
    EXPECT_STREQ(driver::deviceClassName(vr::TrackedDeviceClass_GenericTracker), "tracker");
    EXPECT_STREQ(driver::deviceClassName(vr::TrackedDeviceClass_TrackingReference), "reference");
    EXPECT_STREQ(driver::deviceClassName(vr::TrackedDeviceClass_DisplayRedirect),
                 "display_redirect");
    EXPECT_STREQ(driver::deviceClassName(9999), "unknown");
}

TEST(Names, EveryControllerRoleHasALabel)
{
    EXPECT_STREQ(driver::roleHintName(vr::TrackedControllerRole_Invalid), "invalid");
    EXPECT_STREQ(driver::roleHintName(vr::TrackedControllerRole_LeftHand), "left_hand");
    EXPECT_STREQ(driver::roleHintName(vr::TrackedControllerRole_RightHand), "right_hand");
    EXPECT_STREQ(driver::roleHintName(vr::TrackedControllerRole_OptOut), "opt_out");
    EXPECT_STREQ(driver::roleHintName(vr::TrackedControllerRole_Treadmill), "treadmill");
    EXPECT_STREQ(driver::roleHintName(vr::TrackedControllerRole_Stylus), "stylus");
    EXPECT_STREQ(driver::roleHintName(9999), "unknown");
}

TEST(Guard, ExceptionsNeverLeaveAHookBoundary)
{
    // An exception escaping a detour kills SteamVR outright, so this is the one thing
    // the driver must get right even when everything else is broken.
    bool ran = false;
    driver::runGuarded([&] { ran = true; });
    EXPECT_TRUE(ran);

    driver::runGuarded([] { throw std::runtime_error("inside a hook thread"); });
    driver::runGuarded([] { throw 42; });
}

TEST_F(ObserverTest, AThrowingPropertyReaderCannotEscapeTheGuard)
{
    // The realistic version of the above: vrserver's property reader failing while we
    // enumerate, on the RunFrame thread.
    properties_.throwOnRead = true;
    observer_.setProperties(&properties_);
    driver::runGuarded([&] { observer_.onRunFrame(); });
}
} // namespace
