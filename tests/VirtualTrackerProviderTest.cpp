// Tests for the driver's virtual-tracker emitter (doc/virtual-trackers-plan.md step 6).
//
// VirtualTrackerProvider is the decision surface for device creation, property-writing
// and pose-pushing. Its three dependencies (logger, property reader/writer, host) are
// fakes, so every branch — including the ones a live SteamVR session would only show
// by accident (a device Activate that hasn't happened yet, a stale tracker, a pipe drop)
// — is reachable and deterministic.

#include "driver/Guard.h"
#include "driver/PoseMath.h"
#include "link/Log.h"
#include "driver/VirtualTrackers.h"

#include "link/Protocol.h"

#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

namespace
{
// Builds a wire VirtualTracker frame from a bone name and pose components.
link::Message vtMessage(const char* boneName, float x, float y, float z,
                        float qw, float qx, float qy, float qz)
{
    link::Message m;
    m.size = sizeof(link::VirtualTracker);
    m.type = link::MessageType::VirtualTracker;
    std::memset(m.virtualTracker.name, 0, sizeof(m.virtualTracker.name));
    std::strncpy(m.virtualTracker.name, boneName, sizeof(m.virtualTracker.name) - 1);
    m.virtualTracker.tracking = link::TrackingState::Tracking;
    m.virtualTracker.position = {x, y, z};
    m.virtualTracker.rotation = {qx, qy, qz, qw};
    return m;
}

class FakeServerDriverHost : public driver::ServerDriverHost
{
public:
    // When true (default), the host calls Activate on the device synchronously, as
    // SteamVR does. When false, the device stays unactivated — testing the "no
    // props / no poses before Activate" path.
    bool callActivate = true;

    bool trackedDeviceAdded(const char* serial, vr::ETrackedDeviceClass deviceClass,
                            vr::ITrackedDeviceServerDriver* device) override
    {
        addedSerials.emplace_back(serial ? serial : "");
        addedClasses.push_back(deviceClass);
        addedDrivers.push_back(device);
        if (callActivate && device)
            device->Activate(static_cast<uint32_t>(100 + addedSerials.size()));
        return true;
    }

    void poseUpdated(uint32_t index, const vr::DriverPose_t& pose,
                     uint32_t poseStructSize) override
    {
        pushedPoses.push_back({index, pose, poseStructSize});
    }

    struct PushedPose
    {
        uint32_t index;
        vr::DriverPose_t pose;
        uint32_t structSize;
    };

    std::vector<std::string> addedSerials;
    std::vector<vr::ETrackedDeviceClass> addedClasses;
    std::vector<vr::ITrackedDeviceServerDriver*> addedDrivers;
    std::vector<PushedPose> pushedPoses;
};

class FakeDeviceProperties : public driver::DeviceProperties
{
public:
    vr::PropertyContainerHandle_t container(uint32_t deviceIndex) override
    {
        return 2000 + deviceIndex;
    }

    bool stringProperty(vr::PropertyContainerHandle_t, vr::ETrackedDeviceProperty,
                        std::string&) override
    {
        return false;
    }

    int32_t int32Property(vr::PropertyContainerHandle_t,
                          vr::ETrackedDeviceProperty) override
    {
        return 0;
    }

    bool setStringProperty(vr::PropertyContainerHandle_t container,
                           vr::ETrackedDeviceProperty property,
                           const std::string& value) override
    {
        writtenProps.push_back({container, property, value});
        return true;
    }

    struct WrittenProp
    {
        vr::PropertyContainerHandle_t container;
        vr::ETrackedDeviceProperty property;
        std::string value;
    };

    std::vector<WrittenProp> writtenProps;
};

class VirtualTrackerProviderTest : public ::testing::Test
{
protected:
    VirtualTrackerProviderTest()
        : provider_(logger_, properties_, host_, [this] { return now_; })
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

    double now_ = 100.0;
    std::vector<std::string> lines_;
    link::Logger logger_;
    FakeDeviceProperties properties_;
    FakeServerDriverHost host_;
    driver::VirtualTrackerProvider provider_{logger_, properties_, host_,
                                              [this] { return now_; }};
};

// ---- serialForBone ----

TEST(SerialForBone, IsAPrefixOfTheBoneName)
{
    EXPECT_EQ(driver::serialForBone("LeftUpperLeg"), "TC-LeftUpperLeg");
    EXPECT_EQ(driver::serialForBone("Chest"), "TC-Chest");
}

TEST(SerialForBone, IsStableForTheSameInput)
{
    EXPECT_EQ(driver::serialForBone("Spine"), driver::serialForBone("Spine"));
    EXPECT_NE(driver::serialForBone("Spine"), driver::serialForBone("Chest"));
}

// ---- one device per roster entry ----

TEST_F(VirtualTrackerProviderTest, OneDevicePerUniqueBoneName)
{
    std::vector<link::Message> messages;
    messages.push_back(vtMessage("LeftUpperLeg", 1, 0, 0, 1, 0, 0, 0));
    messages.push_back(vtMessage("RightUpperLeg", 2, 0, 0, 1, 0, 0, 0));

    provider_.onMessages(messages);

    ASSERT_EQ(host_.addedSerials.size(), 2u);
    EXPECT_EQ(host_.addedSerials[0], "TC-LeftUpperLeg");
    EXPECT_EQ(host_.addedSerials[1], "TC-RightUpperLeg");
    EXPECT_EQ(host_.addedClasses[0], vr::TrackedDeviceClass_GenericTracker);
    EXPECT_EQ(host_.addedClasses[1], vr::TrackedDeviceClass_GenericTracker);
}

TEST_F(VirtualTrackerProviderTest, DuplicateRosterEntryAddsNothingTwice)
{
    std::vector<link::Message> messages;
    messages.push_back(vtMessage("Chest", 1, 0, 0, 1, 0, 0, 0));
    messages.push_back(vtMessage("Chest", 2, 0, 0, 1, 0, 0, 0));

    provider_.onMessages(messages);

    EXPECT_EQ(host_.addedSerials.size(), 1u);
    EXPECT_EQ(host_.addedSerials[0], "TC-Chest");
}

// ---- props and poses gated on Activate ----

TEST_F(VirtualTrackerProviderTest, NoPropsOrPosesBeforeActivate)
{
    host_.callActivate = false;

    std::vector<link::Message> messages;
    messages.push_back(vtMessage("Chest", 1, 0, 0, 1, 0, 0, 0));

    provider_.onMessages(messages);

    // The device was registered but not yet activated — no props written, no pose pushed.
    EXPECT_EQ(host_.addedSerials.size(), 1u);
    EXPECT_TRUE(properties_.writtenProps.empty());
    EXPECT_TRUE(host_.pushedPoses.empty());
}

TEST_F(VirtualTrackerProviderTest, PropsWrittenOnceAfterActivate)
{
    host_.callActivate = false;

    std::vector<link::Message> messages;
    messages.push_back(vtMessage("Chest", 1, 0, 0, 1, 0, 0, 0));
    provider_.onMessages(messages);

    // The device is registered but unactivated. Simulate SteamVR calling Activate
    // later (as it does asynchronously after TrackedDeviceAdded). The provider must
    // write props on the next frame once isActivated() is true.
    ASSERT_EQ(host_.addedDrivers.size(), 1u);
    host_.addedDrivers[0]->Activate(101);

    std::vector<link::Message> second;
    second.push_back(vtMessage("Chest", 1.5f, 0, 0, 1, 0, 0, 0));
    provider_.onMessages(second);

    // Four properties: Manufacturer, ModelNumber, RenderModel, TrackingSystem.
    ASSERT_EQ(properties_.writtenProps.size(), 4u);
    EXPECT_EQ(properties_.writtenProps[0].value, "TrackingCorrector");
    EXPECT_EQ(properties_.writtenProps[1].value, "TrackingCorrector Virtual Tracker");
    EXPECT_EQ(properties_.writtenProps[2].value, "{htc}/rendermodels/vr_tracker_vive_1_0");
    EXPECT_EQ(properties_.writtenProps[3].value, "00trackingcorrector");

    // Feeding again must not re-write props.
    std::vector<link::Message> third;
    third.push_back(vtMessage("Chest", 2.0f, 0, 0, 1, 0, 0, 0));
    provider_.onMessages(third);
    EXPECT_EQ(properties_.writtenProps.size(), 4u);
}

TEST_F(VirtualTrackerProviderTest, PosePushedOnlyAfterActivate)
{
    host_.callActivate = false;

    std::vector<link::Message> messages;
    messages.push_back(vtMessage("Chest", 1, 0, 0, 1, 0, 0, 0));
    provider_.onMessages(messages);
    EXPECT_TRUE(host_.pushedPoses.empty());

    // After SteamVR calls Activate, the next frame pushes the pose.
    ASSERT_EQ(host_.addedDrivers.size(), 1u);
    host_.addedDrivers[0]->Activate(101);
    std::vector<link::Message> second;
    second.push_back(vtMessage("Chest", 1.5f, 0, 0, 1, 0, 0, 0));
    provider_.onMessages(second);

    ASSERT_EQ(host_.pushedPoses.size(), 1u);
    EXPECT_EQ(host_.pushedPoses[0].index, 101u);
}

// ---- pose values: identity transforms, zero velocities, position/rotation from wire ----

TEST_F(VirtualTrackerProviderTest, PoseHasIdentityTransformsAndZeroVelocities)
{
    std::vector<link::Message> messages;
    messages.push_back(vtMessage("Spine", 1.5f, 2.5f, 3.5f, 0.0f, 0.0f, 1.0f, 0.0f));
    provider_.onMessages(messages);

    ASSERT_EQ(host_.pushedPoses.size(), 1u);
    const vr::DriverPose_t& pose = host_.pushedPoses[0].pose;

    EXPECT_TRUE(pose.poseIsValid);
    EXPECT_TRUE(pose.deviceIsConnected);
    EXPECT_EQ(pose.result, vr::TrackingResult_Running_OK);

    // Identity world-from-driver and driver-from-head.
    EXPECT_EQ(pose.qWorldFromDriverRotation.w, 1.0);
    EXPECT_EQ(pose.qWorldFromDriverRotation.x, 0.0);
    EXPECT_EQ(pose.qWorldFromDriverRotation.y, 0.0);
    EXPECT_EQ(pose.qWorldFromDriverRotation.z, 0.0);
    EXPECT_EQ(pose.qDriverFromHeadRotation.w, 1.0);
    EXPECT_EQ(pose.qDriverFromHeadRotation.x, 0.0);
    EXPECT_EQ(pose.qDriverFromHeadRotation.y, 0.0);
    EXPECT_EQ(pose.qDriverFromHeadRotation.z, 0.0);

    // Zero translation offsets and zero velocities/accelerations.
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_EQ(pose.vecWorldFromDriverTranslation[i], 0.0);
        EXPECT_EQ(pose.vecDriverFromHeadTranslation[i], 0.0);
        EXPECT_EQ(pose.vecVelocity[i], 0.0);
        EXPECT_EQ(pose.vecAcceleration[i], 0.0);
        EXPECT_EQ(pose.vecAngularVelocity[i], 0.0);
        EXPECT_EQ(pose.vecAngularAcceleration[i], 0.0);
    }

    // Position and rotation from the wire pose.
    EXPECT_EQ(pose.vecPosition[0], 1.5);
    EXPECT_EQ(pose.vecPosition[1], 2.5);
    EXPECT_EQ(pose.vecPosition[2], 3.5);
    // Wire quaternion (x=0, y=1, z=0, w=0): 180 deg about Y.
    EXPECT_EQ(pose.qRotation.w, 0.0);
    EXPECT_EQ(pose.qRotation.x, 0.0);
    EXPECT_EQ(pose.qRotation.y, 1.0);
    EXPECT_EQ(pose.qRotation.z, 0.0);
}

// ---- staleness: a device that goes stale is marked disconnected, pushed once ----

TEST_F(VirtualTrackerProviderTest, StaleDeviceIsMarkedDisconnected)
{
    std::vector<link::Message> messages;
    messages.push_back(vtMessage("Hips", 1, 0, 0, 1, 0, 0, 0));
    provider_.onMessages(messages);

    ASSERT_EQ(host_.pushedPoses.size(), 1u);
    EXPECT_TRUE(host_.pushedPoses[0].pose.deviceIsConnected);

    // Advance time past the staleness window without sending another frame.
    now_ += driver::kVirtualTrackerStaleSeconds + 0.01;
    provider_.onRunFrame();

    // A disconnected pose was pushed, and only once.
    ASSERT_EQ(host_.pushedPoses.size(), 2u);
    EXPECT_FALSE(host_.pushedPoses[1].pose.deviceIsConnected);
    EXPECT_FALSE(host_.pushedPoses[1].pose.poseIsValid);
    EXPECT_EQ(host_.pushedPoses[1].pose.result, vr::TrackingResult_Uninitialized);

    // A second runFrame must not push another disconnect.
    provider_.onRunFrame();
    EXPECT_EQ(host_.pushedPoses.size(), 2u);
}

TEST_F(VirtualTrackerProviderTest, ADeviceWithinTheStalenessWindowStaysConnected)
{
    std::vector<link::Message> messages;
    messages.push_back(vtMessage("Hips", 1, 0, 0, 1, 0, 0, 0));
    provider_.onMessages(messages);

    now_ += driver::kVirtualTrackerStaleSeconds - 0.01;
    provider_.onRunFrame();

    // No disconnect pose pushed.
    EXPECT_EQ(host_.pushedPoses.size(), 1u);
}

TEST_F(VirtualTrackerProviderTest, ResumingAfterStalenessReconnects)
{
    std::vector<link::Message> messages;
    messages.push_back(vtMessage("Hips", 1, 0, 0, 1, 0, 0, 0));
    provider_.onMessages(messages);

    now_ += driver::kVirtualTrackerStaleSeconds + 0.01;
    provider_.onRunFrame();
    ASSERT_EQ(host_.pushedPoses.size(), 2u);

    // The app resumes sending: the device reconnects and a connected pose is pushed.
    std::vector<link::Message> resume;
    resume.push_back(vtMessage("Hips", 2, 0, 0, 1, 0, 0, 0));
    provider_.onMessages(resume);

    ASSERT_EQ(host_.pushedPoses.size(), 3u);
    EXPECT_TRUE(host_.pushedPoses[2].pose.deviceIsConnected);
    EXPECT_TRUE(host_.pushedPoses[2].pose.poseIsValid);
}

// ---- markAllDisconnected ----

TEST_F(VirtualTrackerProviderTest, MarkAllDisconnectedPushesForEveryActiveDevice)
{
    std::vector<link::Message> messages;
    messages.push_back(vtMessage("Chest", 1, 0, 0, 1, 0, 0, 0));
    messages.push_back(vtMessage("Spine", 2, 0, 0, 1, 0, 0, 0));
    messages.push_back(vtMessage("Hips", 3, 0, 0, 1, 0, 0, 0));
    provider_.onMessages(messages);

    // Two poses per device: one connected (from onMessages), then disconnected.
    // Wait — onMessages pushes one connected pose each = 3 total. Then markAllDisconnected
    // pushes 3 disconnected poses = 6 total.
    ASSERT_EQ(host_.pushedPoses.size(), 3u);

    provider_.markAllDisconnected();

    ASSERT_EQ(host_.pushedPoses.size(), 6u);
    for (std::size_t i = 3; i < 6; ++i)
    {
        EXPECT_FALSE(host_.pushedPoses[i].pose.deviceIsConnected);
        EXPECT_FALSE(host_.pushedPoses[i].pose.poseIsValid);
    }

    // A second markAllDisconnected must not re-push (already disconnected).
    provider_.markAllDisconnected();
    EXPECT_EQ(host_.pushedPoses.size(), 6u);
}

// ---- non-VirtualTracker messages are ignored ----

TEST_F(VirtualTrackerProviderTest, NonVirtualTrackerMessagesAreIgnored)
{
    std::vector<link::Message> messages;
    link::Message poseOverride;
    poseOverride.size = sizeof(link::PoseOverride);
    poseOverride.type = link::MessageType::PoseOverride;
    poseOverride.poseOverride.deviceId = 5;
    messages.push_back(poseOverride);

    provider_.onMessages(messages);

    EXPECT_TRUE(host_.addedSerials.empty());
    EXPECT_TRUE(host_.pushedPoses.empty());
}

// ---- GetPose returns the last pose atomically ----

TEST_F(VirtualTrackerProviderTest, GetPoseReturnsTheLastPushedPose)
{
    std::vector<link::Message> messages;
    messages.push_back(vtMessage("Chest", 1.0f, 2.0f, 3.0f, 1, 0, 0, 0));
    provider_.onMessages(messages);

    // The device is the first one added; Activate was called with index 101.
    ASSERT_EQ(host_.addedDrivers.size(), 1u);
    auto* device = static_cast<driver::VirtualTracker*>(host_.addedDrivers[0]);
    ASSERT_NE(device, nullptr);
    EXPECT_EQ(device->deviceIndex(), 101u);

    const vr::DriverPose_t pose = device->GetPose();
    EXPECT_EQ(pose.vecPosition[0], 1.0);
    EXPECT_EQ(pose.vecPosition[1], 2.0);
    EXPECT_EQ(pose.vecPosition[2], 3.0);
    EXPECT_TRUE(pose.poseIsValid);
}

// ---- ITrackedDeviceServerDriver entry points do not throw ----

TEST_F(VirtualTrackerProviderTest, ActivateReturnsNoneOnSuccess)
{
    driver::VirtualTracker device("TC-Chest", "Chest");
    EXPECT_EQ(device.Activate(42), vr::VRInitError_None);
    EXPECT_EQ(device.deviceIndex(), 42u);
    EXPECT_TRUE(device.isActivated());
}

TEST_F(VirtualTrackerProviderTest, DeactivateInvalidatesTheIndex)
{
    driver::VirtualTracker device("TC-Chest", "Chest");
    device.Activate(42);
    EXPECT_TRUE(device.isActivated());

    device.Deactivate();
    EXPECT_FALSE(device.isActivated());
    EXPECT_EQ(device.deviceIndex(), vr::k_unTrackedDeviceIndexInvalid);
}

TEST_F(VirtualTrackerProviderTest, GetComponentReturnsNull)
{
    driver::VirtualTracker device("TC-Chest", "Chest");
    EXPECT_EQ(device.GetComponent("anything"), nullptr);
}

TEST_F(VirtualTrackerProviderTest, DebugRequestAndEnterStandbyDoNotThrow)
{
    driver::VirtualTracker device("TC-Chest", "Chest");
    char buffer[16] = {};
    device.DebugRequest("test", buffer, sizeof(buffer));
    device.EnterStandby();
    SUCCEED();
}

TEST_F(VirtualTrackerProviderTest, GetPoseBeforeAnyPoseIsPushedReturnsDisconnectedDefault)
{
    driver::VirtualTracker device("TC-Chest", "Chest");
    const vr::DriverPose_t pose = device.GetPose();
    // Default-constructed pose: valid=false, connected=false (value-init to zero).
    EXPECT_FALSE(pose.poseIsValid);
    EXPECT_FALSE(pose.deviceIsConnected);
}
} // namespace
