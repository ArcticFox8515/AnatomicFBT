// Tests for the throwaway step-1 spike's observation logic (doc/driver-plan.md).
//
// SpikeObserver is the whole decision surface of the spike, and it is driven here
// directly: no DLL, no MinHook, no vrserver, no clock, no filesystem. Its four
// dependencies (logger, hook installer, property reader, clock) are fakes, so every
// branch — including the ones a live SteamVR session would only show us by accident
// (a truncated pose struct, a device whose serial changed, an unhookable interface
// version, an update for a component we never saw created) — is reachable and
// deterministic.

#include "spike/SpikeGuard.h"
#include "spike/SpikeLog.h"
#include "spike/SpikeNames.h"
#include "spike/SpikeObserver.h"
#include "spike/SpikePoseMath.h"

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
constexpr vr::VRInputComponentHandle_t kTriggerHandle = 100;

class FakeInterfaceHooks : public spike::InterfaceHooks
{
public:
    void hookServerDriverHost(void* host) override { hostHooks.push_back(host); }
    void hookDriverInput(void* input) override { inputHooks.push_back(input); }

    std::vector<void*> hostHooks;
    std::vector<void*> inputHooks;
};

class FakeProperties : public spike::DeviceProperties
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

class SpikeObserverTest : public ::testing::Test
{
protected:
    SpikeObserverTest() : observer_(logger_, hooks_, [this] { return now_; })
    {
        logger_.setTimestampSource([] { return std::string("00:00:00.000"); });
        logger_.setSink([this](const char* message) { lines_.emplace_back(message); });
    }

    bool logged(const std::string& needle) const
    {
        for (const std::string& line : lines_)
            if (line.find(needle) != std::string::npos)
                return true;
        return false;
    }

    size_t countLogged(const std::string& needle) const
    {
        size_t count = 0;
        for (const std::string& line : lines_)
            if (line.find(needle) != std::string::npos)
                ++count;
        return count;
    }

    // The rest pose of the fixture: one tracker known, one trigger component created
    // and resolved to it, one pose seen.
    void bringUpTracker()
    {
        properties_.add(kTracker, {"LHR-TESTTRACKER", "Vive Tracker", "lighthouse",
                                   vr::TrackedDeviceClass_GenericTracker,
                                   vr::TrackedControllerRole_Invalid});
        observer_.setProperties(&properties_);
        observer_.onBooleanComponentCreated(kContainerBase + kTracker, "/input/trigger/click",
                                           kTriggerHandle);
        observer_.onPose(kTracker, makePose(), sizeof(vr::DriverPose_t));
        observer_.onInit();
        observer_.onRunFrame();
    }

    double now_ = 100.0;
    std::vector<std::string> lines_;
    spike::Logger logger_;
    FakeInterfaceHooks hooks_;
    FakeProperties properties_;
    spike::SpikeObserver observer_;
};

// ------------------------------------------------- GetGenericInterface dispatch ----

TEST_F(SpikeObserverTest, TheInterfacesWeBuildAgainstAreHooked)
{
    int host = 0;
    int input = 0;
    observer_.onInterfaceRequested(vr::IVRServerDriverHost_Version, &host);
    observer_.onInterfaceRequested(vr::IVRDriverInput_Version, &input);

    EXPECT_EQ(hooks_.hostHooks, std::vector<void*>{&host});
    EXPECT_EQ(hooks_.inputHooks, std::vector<void*>{&input});
    EXPECT_TRUE(logged(std::string("interface \"") + vr::IVRServerDriverHost_Version
                       + "\": hooking"));
    EXPECT_TRUE(logged(std::string("interface \"") + vr::IVRDriverInput_Version + "\": hooking"));
}

TEST_F(SpikeObserverTest, AnUnhookableVersionOfAnInterfaceWeNeedIsReportedLoudlyAndNotHooked)
{
    // The dangerous silent case: a version we cannot hook means every device behind it
    // is invisible to us, and the pointer must not be touched (its vtable layout is
    // exactly what we do not know).
    int dummy = 0;
    observer_.onInterfaceRequested("IVRServerDriverHost_005", &dummy);
    observer_.onInterfaceRequested("IVRDriverInput_002", &dummy);

    EXPECT_TRUE(hooks_.hostHooks.empty());
    EXPECT_TRUE(hooks_.inputHooks.empty());
    EXPECT_TRUE(logged("interface \"IVRServerDriverHost_005\": NOT HOOKED"));
    EXPECT_TRUE(logged("interface \"IVRDriverInput_002\": NOT HOOKED"));
}

TEST_F(SpikeObserverTest, InterfacesWeDoNotNeedAndNullResultsAreJustRecorded)
{
    int dummy = 0;
    observer_.onInterfaceRequested(vr::IVRResources_Version, &dummy);
    observer_.onInterfaceRequested("IVRCameraComponent_004", nullptr);
    observer_.onInterfaceRequested(nullptr, nullptr);

    EXPECT_TRUE(logged("interface \"IVRResources_001\": seen, not hooked (not needed)"));
    EXPECT_TRUE(logged("interface \"IVRCameraComponent_004\": requested, vrserver returned NULL"));
    EXPECT_TRUE(logged("interface \"(null)\": requested, vrserver returned NULL"));
    EXPECT_TRUE(hooks_.hostHooks.empty());
}

TEST_F(SpikeObserverTest, EachVersionStringIsReportedOnce)
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

TEST_F(SpikeObserverTest, DeviceAdditionsAreLoggedWithTheirClass)
{
    observer_.onDeviceAdded("LHR-1", vr::TrackedDeviceClass_GenericTracker);
    observer_.onDeviceAdded(nullptr, vr::TrackedDeviceClass_HMD);
    EXPECT_TRUE(logged("TrackedDeviceAdded: serial=\"LHR-1\" class=tracker(3)"));
    EXPECT_TRUE(logged("TrackedDeviceAdded: serial=\"(null)\" class=hmd(1)"));
}

TEST_F(SpikeObserverTest, FirstPosePerDeviceIsLoggedOnce)
{
    observer_.onPose(kTracker, makePose(), sizeof(vr::DriverPose_t));
    observer_.onPose(kTracker, makePose(), sizeof(vr::DriverPose_t));
    EXPECT_EQ(countLogged("first pose update from device 3 (poseIsValid=1 connected=1)"), 1u);
}

TEST_F(SpikeObserverTest, PosesForImpossibleDeviceIndicesAreDropped)
{
    // A hook thread hands us whatever vrserver hands it; indexing our array with it
    // unchecked would be a write past the end inside vrserver.
    observer_.onPose(vr::k_unMaxTrackedDeviceCount, makePose(), sizeof(vr::DriverPose_t));
    observer_.onPose(vr::k_unMaxTrackedDeviceCount + 7, makePose(), sizeof(vr::DriverPose_t));
    EXPECT_TRUE(lines_.empty());
}

TEST_F(SpikeObserverTest, ATruncatedPoseStructIsWarnedAboutOnceAndNeverRead)
{
    // A caller built against an older openvr header: the fields beyond its struct size
    // do not exist, so the pose must not be stored, let alone composed.
    observer_.onPose(kTracker, makePose(), sizeof(vr::DriverPose_t) - 8);
    observer_.onPose(4, makePose(), sizeof(vr::DriverPose_t) - 8);

    EXPECT_EQ(countLogged("pose contents NOT read for such callers"), 1u);
    EXPECT_TRUE(logged("first pose update from device 3 (truncated pose struct)"));
    EXPECT_TRUE(logged("first pose update from device 4 (truncated pose struct)"));

    observer_.setProperties(&properties_);
    observer_.onInit();
    observer_.onRunFrame();
    EXPECT_FALSE(logged("A = wFd o local"));
}

// --------------------------------------------------------- input components ----

TEST_F(SpikeObserverTest, BooleanComponentsAreRecordedAndTriggersMarked)
{
    observer_.onBooleanComponentCreated(kContainerBase, "/input/trigger/click", 100);
    observer_.onBooleanComponentCreated(kContainerBase, "/input/grip/click", 101);
    observer_.onBooleanComponentCreated(kContainerBase, nullptr, 102);

    EXPECT_TRUE(logged("CreateBooleanComponent: container=5000 handle=100 "
                       "name=\"/input/trigger/click\" <-- trigger click"));
    EXPECT_TRUE(logged("name=\"/input/grip/click\""));
    EXPECT_FALSE(logged("name=\"/input/grip/click\" <-- trigger click"));
    EXPECT_TRUE(logged("handle=102 name=\"(null)\""));
}

TEST_F(SpikeObserverTest, ScalarComponentsAreOnlyReported)
{
    // Never hooked for updates — logged to find out whether a controller exposes a
    // scalar-only trigger (the documented fallback in doc/driver-plan.md).
    observer_.onScalarComponentCreated(kContainerBase, "/input/trigger/value", 200);
    observer_.onScalarComponentCreated(kContainerBase, nullptr, 201);
    EXPECT_TRUE(logged("CreateScalarComponent: container=5000 handle=200 "
                       "name=\"/input/trigger/value\""));
    EXPECT_TRUE(logged("handle=201 name=\"(null)\""));
}

TEST_F(SpikeObserverTest, OnlyTriggerEdgesAreLogged)
{
    bringUpTracker();
    lines_.clear();

    observer_.onBooleanComponentUpdated(kTriggerHandle, true);
    observer_.onBooleanComponentUpdated(kTriggerHandle, true); // repeat, not an edge
    observer_.onBooleanComponentUpdated(kTriggerHandle, false);
    observer_.onBooleanComponentUpdated(kTriggerHandle, false);

    EXPECT_EQ(lines_.size(), 2u);
    EXPECT_TRUE(logged("trigger DOWN: device 3 (LHR-TESTTRACKER) component "
                       "\"/input/trigger/click\""));
    EXPECT_TRUE(logged("trigger up  : device 3 (LHR-TESTTRACKER)"));
}

TEST_F(SpikeObserverTest, NonTriggerComponentsAndUnresolvedDevicesStayQuiet)
{
    observer_.onBooleanComponentCreated(kContainerBase, "/input/grip/click", 101);
    observer_.onBooleanComponentCreated(kContainerBase, "/input/trigger/click", kTriggerHandle);
    lines_.clear();

    observer_.onBooleanComponentUpdated(101, true);
    EXPECT_TRUE(lines_.empty());

    // A trigger whose container never matched a device: still logged, with "?" instead
    // of a serial, because losing the edge would hide the calibration gesture.
    observer_.onBooleanComponentUpdated(kTriggerHandle, true);
    EXPECT_TRUE(logged("trigger DOWN: device -1 (?) component \"/input/trigger/click\""));
}

TEST_F(SpikeObserverTest, UpdatesForUnknownHandlesAreCountedNotDropped)
{
    // Components created before our hook went in: a coverage gap we have to be able to
    // see, since it is the one thing that would silently cost us a trigger.
    bringUpTracker();
    now_ += spike::kStatsSeconds;

    observer_.onBooleanComponentUpdated(987654321, true);
    observer_.onBooleanComponentUpdated(987654321, false);
    lines_.clear();
    observer_.onRunFrame();

    EXPECT_TRUE(logged("UpdateBooleanComponent: 2 updates for components created before our "
                       "hook was installed"));

    // Reported as a delta, so an idle interval says nothing.
    lines_.clear();
    now_ += spike::kStatsSeconds;
    observer_.onRunFrame();
    EXPECT_FALSE(logged("created before our"));
}

// ---------------------------------------------------- RunFrame housekeeping ----

TEST_F(SpikeObserverTest, HousekeepingRunsOnTheFirstFrameThenOncePerSecond)
{
    properties_.add(kTracker, {"LHR-TESTTRACKER", "Vive Tracker", "lighthouse",
                               vr::TrackedDeviceClass_GenericTracker,
                               vr::TrackedControllerRole_Invalid});
    observer_.setProperties(&properties_);
    observer_.onInit();

    // Deliberately on the *first* frame, so devices show up immediately rather than a
    // second late.
    observer_.onRunFrame();
    EXPECT_TRUE(logged("first RunFrame call"));
    EXPECT_TRUE(logged("device 3: class=tracker(3) role=invalid serial=\"LHR-TESTTRACKER\" "
                       "model=\"Vive Tracker\" trackingSystem=\"lighthouse\" container=5003"));

    const uint32_t lookupsAfterFirst = properties_.containerLookups;
    now_ += spike::kHousekeepingSeconds / 2.0;
    observer_.onRunFrame();
    EXPECT_EQ(properties_.containerLookups, lookupsAfterFirst);

    now_ += spike::kHousekeepingSeconds;
    observer_.onRunFrame();
    EXPECT_GT(properties_.containerLookups, lookupsAfterFirst);
    EXPECT_EQ(countLogged("first RunFrame call"), 1u);
}

TEST_F(SpikeObserverTest, WithoutAPropertyReaderNothingIsEnumerated)
{
    // The state before InitServerDriverContext succeeds, and after Cleanup.
    observer_.setProperties(nullptr);
    observer_.onInit();
    observer_.onRunFrame();
    EXPECT_EQ(properties_.containerLookups, 0u);
    EXPECT_FALSE(logged("device 3:"));
}

TEST_F(SpikeObserverTest, ContainersWithoutADeviceAreSkipped)
{
    // vrserver answers TrackedDeviceToPropertyContainer for every index; only some of
    // them have a device, and the rest must not become empty rows in the table.
    properties_.add(1, {"", "", "", vr::TrackedDeviceClass_Invalid,
                        vr::TrackedControllerRole_Invalid, /*readable=*/false});
    observer_.setProperties(&properties_);
    observer_.onInit();
    observer_.onRunFrame();

    EXPECT_GT(properties_.containerLookups, 0u);
    EXPECT_FALSE(logged("device 1:"));
    observer_.onCleanup();
    EXPECT_FALSE(logged("summary: device 1"));
}

TEST_F(SpikeObserverTest, MetadataIsReReadOnlyWhenTheSerialChanges)
{
    bringUpTracker();
    lines_.clear();

    now_ += spike::kHousekeepingSeconds;
    observer_.onRunFrame();
    EXPECT_FALSE(logged("device 3: class="));

    // A tracker powered off and a different one powered on in the same slot.
    properties_.devices[kTracker].serial = "LHR-OTHER";
    properties_.devices[kTracker].deviceClass = vr::TrackedDeviceClass_Controller;
    properties_.devices[kTracker].roleHint = vr::TrackedControllerRole_LeftHand;
    now_ += spike::kHousekeepingSeconds;
    observer_.onRunFrame();
    EXPECT_TRUE(logged("device 3: class=controller(2) role=left_hand serial=\"LHR-OTHER\""));
}

TEST_F(SpikeObserverTest, ComponentsResolveToTheDeviceSharingTheirContainer)
{
    // The mapping the real driver needs to attribute a trigger to a device.
    bringUpTracker();
    EXPECT_TRUE(logged("component \"/input/trigger/click\" resolved to device 3 "
                       "(\"LHR-TESTTRACKER\")"));

    lines_.clear();
    now_ += spike::kHousekeepingSeconds;
    observer_.onRunFrame();
    EXPECT_FALSE(logged("resolved to device"));
}

TEST_F(SpikeObserverTest, AComponentWithNoMatchingDeviceStaysUnresolved)
{
    observer_.onBooleanComponentCreated(999999, "/input/trigger/click", kTriggerHandle);
    properties_.add(kTracker, {"LHR-TESTTRACKER", "", "", vr::TrackedDeviceClass_GenericTracker,
                               vr::TrackedControllerRole_Invalid});
    observer_.setProperties(&properties_);
    observer_.onInit();
    observer_.onRunFrame();

    EXPECT_FALSE(logged("resolved to device"));
    observer_.onCleanup();
    EXPECT_TRUE(logged("summary: component \"/input/trigger/click\" device=-1 updates=0"));
}

TEST_F(SpikeObserverTest, BothCandidateCompositionsAreLoggedForEveryPosedDevice)
{
    // The entire point of the spike: these two lines are what the live run compares
    // against spike_client's raw pose.
    const vr::DriverPose_t pose = makePose();
    bringUpTracker();

    const spike::RigidPose worldFromDriver{{10.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}};
    const spike::RigidPose local{{1.0, 2.0, 3.0}, {0.0, 0.0, 1.0, 0.0}};
    const spike::RigidPose driverFromHead{{0.0, 0.5, 0.0}, {}};
    const spike::RigidPose a = spike::compose(worldFromDriver, local);
    const spike::RigidPose b = spike::compose(a, driverFromHead);

    EXPECT_TRUE(logged("pose dev 3 tracker \"LHR-TESTTRACKER\" valid=1 connected=1 result=200 "
                       "timeOffset=-0.01100"));
    EXPECT_TRUE(logged("local            " + spike::formatPose(local)));
    EXPECT_TRUE(logged("worldFromDriver  " + spike::formatPose(worldFromDriver)));
    EXPECT_TRUE(logged("driverFromHead   " + spike::formatPose(driverFromHead)));
    EXPECT_TRUE(logged("A = wFd o local  " + spike::formatPose(a)));
    EXPECT_TRUE(logged("B = A o dFh      " + spike::formatPose(b)));
    EXPECT_NE(spike::formatPose(a), spike::formatPose(b)) << "DriverFromHead must matter here";
    EXPECT_EQ(pose.vecPosition[0], 1.0);
}

TEST_F(SpikeObserverTest, APoseSteamVRReportsAsInvalidIsNotComposed)
{
    // What lighthouse actually sends for a device that is not tracking, taken from
    // driver-spike-vrserver.log at 18:27:09.364: the 9001 sentinel, a pose time offset
    // 80 seconds stale (the headset had entered standby), and - for the base stations -
    // an all-zero quaternion, which is not a rotation at all.
    //
    // Composing that produces numbers, not information, and they are printed in the same
    // format as the A/B lines the whole spike exists to compare. Same contract as
    // ATruncatedPoseStructIsWarnedAboutOnceAndNeverRead above: what cannot be trusted is
    // not composed.
    bringUpTracker();

    vr::DriverPose_t untracked{};
    untracked.poseIsValid = false;
    untracked.deviceIsConnected = false;
    untracked.result = vr::TrackingResult_Uninitialized;
    untracked.poseTimeOffset = -80.92440;
    untracked.vecPosition[1] = 9001.0;
    untracked.qRotation = {0.0, 0.0, 0.0, 0.0};
    observer_.onPose(kTracker, untracked, sizeof(vr::DriverPose_t));

    // Only the housekeeping frame that follows the invalid pose is under test; the valid
    // pose bringUpTracker() already logged is not.
    lines_.clear();
    now_ += spike::kHousekeepingSeconds;
    observer_.onRunFrame();

    EXPECT_FALSE(logged("A = wFd o local"));
    EXPECT_FALSE(logged("B = A o dFh"));
    EXPECT_FALSE(logged("worldFromDriver"));
    EXPECT_FALSE(logged("driverFromHead"));
}

// ------------------------------------------------------------------- rates ----

TEST_F(SpikeObserverTest, RatesAreReportedEveryFiveSecondsAsDeltas)
{
    bringUpTracker();
    lines_.clear();

    // Not yet due.
    now_ += spike::kStatsSeconds - 0.1;
    observer_.onRunFrame();
    EXPECT_FALSE(logged("RunFrame:"));

    for (int i = 0; i < 9; ++i)
        observer_.onPose(kTracker, makePose(), sizeof(vr::DriverPose_t));
    now_ += 0.1;
    observer_.onRunFrame();

    // 3 RunFrame calls and 10 poses over the 5 s window (the first pose came from
    // bringUpTracker, before the window).
    EXPECT_TRUE(logged("RunFrame: 0.6 Hz (3 calls total)"));
    EXPECT_TRUE(logged("pose rate: device 3 tracker \"LHR-TESTTRACKER\": 2.0 Hz (10 total)"));

    // Silent for devices that did not move on.
    lines_.clear();
    now_ += spike::kStatsSeconds;
    observer_.onRunFrame();
    EXPECT_TRUE(logged("RunFrame:"));
    EXPECT_FALSE(logged("pose rate:"));
}

// ----------------------------------------------------------------- cleanup ----

TEST_F(SpikeObserverTest, CleanupSummarizesWhatWasSeenAndNothingElse)
{
    bringUpTracker();
    observer_.onBooleanComponentUpdated(kTriggerHandle, true);
    now_ += 12.5;
    lines_.clear();

    observer_.onCleanup();
    EXPECT_TRUE(logged("summary: 1 RunFrame calls in 12.5 s"));
    EXPECT_TRUE(logged("summary: device 3 tracker \"LHR-TESTTRACKER\": 1 pose updates"));
    EXPECT_TRUE(logged("summary: component \"/input/trigger/click\" device=3 updates=1"));
    // 22 of the 64 device slots would otherwise be logged as empty rows.
    EXPECT_EQ(countLogged("summary: device"), 1u);
}

TEST_F(SpikeObserverTest, ADeviceKnownByMetadataAloneIsStillSummarized)
{
    // No pose ever arrived (a base station, or a tracker that never tracked): the row
    // is exactly what tells us the difference.
    properties_.add(2, {"LHR-BASE", "", "", vr::TrackedDeviceClass_TrackingReference,
                        vr::TrackedControllerRole_Invalid});
    observer_.setProperties(&properties_);
    observer_.onInit();
    observer_.onRunFrame();
    observer_.onCleanup();
    EXPECT_TRUE(logged("summary: device 2 reference \"LHR-BASE\": 0 pose updates"));
}

// -------------------------------------------------------- names and guards ----

TEST(SpikeNames, EveryDeviceClassHasALabel)
{
    EXPECT_STREQ(spike::deviceClassName(vr::TrackedDeviceClass_Invalid), "invalid");
    EXPECT_STREQ(spike::deviceClassName(vr::TrackedDeviceClass_HMD), "hmd");
    EXPECT_STREQ(spike::deviceClassName(vr::TrackedDeviceClass_Controller), "controller");
    EXPECT_STREQ(spike::deviceClassName(vr::TrackedDeviceClass_GenericTracker), "tracker");
    EXPECT_STREQ(spike::deviceClassName(vr::TrackedDeviceClass_TrackingReference), "reference");
    EXPECT_STREQ(spike::deviceClassName(vr::TrackedDeviceClass_DisplayRedirect),
                 "display_redirect");
    EXPECT_STREQ(spike::deviceClassName(9999), "unknown");
}

TEST(SpikeNames, EveryControllerRoleHasALabel)
{
    EXPECT_STREQ(spike::roleHintName(vr::TrackedControllerRole_Invalid), "invalid");
    EXPECT_STREQ(spike::roleHintName(vr::TrackedControllerRole_LeftHand), "left_hand");
    EXPECT_STREQ(spike::roleHintName(vr::TrackedControllerRole_RightHand), "right_hand");
    EXPECT_STREQ(spike::roleHintName(vr::TrackedControllerRole_OptOut), "opt_out");
    EXPECT_STREQ(spike::roleHintName(vr::TrackedControllerRole_Treadmill), "treadmill");
    EXPECT_STREQ(spike::roleHintName(vr::TrackedControllerRole_Stylus), "stylus");
    EXPECT_STREQ(spike::roleHintName(9999), "unknown");
}

TEST(SpikeNames, TriggerClickIsRecognizedBySuffixOnly)
{
    EXPECT_TRUE(spike::isTriggerClick("/input/trigger/click"));
    EXPECT_TRUE(spike::isTriggerClick("/user/hand/left/input/trigger/click"));
    EXPECT_FALSE(spike::isTriggerClick("/input/trigger/value"));
    EXPECT_FALSE(spike::isTriggerClick("/input/trigger/click/extra"));
    EXPECT_FALSE(spike::isTriggerClick("click"));
    EXPECT_FALSE(spike::isTriggerClick(""));
}

TEST(SpikeNames, EndsWithHandlesShortAndEqualStrings)
{
    EXPECT_TRUE(spike::endsWith("abc", "abc"));
    EXPECT_TRUE(spike::endsWith("abc", ""));
    EXPECT_FALSE(spike::endsWith("ab", "abc"));
}

TEST(SpikeGuard, ExceptionsNeverLeaveAHookBoundary)
{
    // An exception escaping a detour kills SteamVR outright, so this is the one thing
    // the spike must get right even when everything else is broken.
    bool ran = false;
    spike::runGuarded([&] { ran = true; });
    EXPECT_TRUE(ran);

    spike::runGuarded([] { throw std::runtime_error("inside a hook thread"); });
    spike::runGuarded([] { throw 42; });
}

TEST_F(SpikeObserverTest, AThrowingPropertyReaderCannotEscapeTheGuard)
{
    // The realistic version of the above: vrserver's property reader failing while we
    // enumerate, on the RunFrame thread.
    properties_.throwOnRead = true;
    observer_.setProperties(&properties_);
    observer_.onInit();
    spike::runGuarded([&] { observer_.onRunFrame(); });
    EXPECT_TRUE(logged("first RunFrame call"));
}
} // namespace
