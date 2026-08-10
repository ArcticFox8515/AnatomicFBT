// Tests for the app-side pose source (doc/driver-plan.md phase A, step 4).
//
// OpenVrTracking owns a link::MessageChannel driven by the test's FakePipe:
// the channel is created and connected on init(), framed DevicePose bytes fed
// into the fake's read queue are reassembled and folded into the TrackedDevice
// snapshot, and the reconnect throttle is exercised with an injected clock.
// Nothing touches Win32 or openvr.

#include "FakePipe.h"
#include "link/Log.h"
#include "link/Protocol.h"
#include "model/Error.h"
#include "model/OpenVrTracking.h"
#include "model/Pose.h"
#include "model/TrackerCorrection.h"
#include "model/VirtualTrackers.h"

#include <gtest/gtest.h>

#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <cstring>
#include <vector>

namespace
{
link::Logger& testLogger()
{
    static link::Logger logger;
    return logger;
}

// One length-prefixed frame: u32 size, u16 type, 2 pad, then the DevicePose.
std::vector<std::uint8_t> poseFrame(const link::DevicePose& pose)
{
    const std::uint8_t* p = reinterpret_cast<const std::uint8_t*>(&pose);
    std::vector<std::uint8_t> out;
    const std::uint32_t len = static_cast<std::uint32_t>(sizeof(link::DevicePose));
    out.push_back(static_cast<std::uint8_t>(len & 0xFF));
    out.push_back(static_cast<std::uint8_t>((len >> 8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((len >> 16) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((len >> 24) & 0xFF));
    const std::uint16_t t = static_cast<std::uint16_t>(link::MessageType::DevicePose);
    out.push_back(static_cast<std::uint8_t>(t & 0xFF));
    out.push_back(static_cast<std::uint8_t>((t >> 8) & 0xFF));
    out.push_back(0);
    out.push_back(0);
    out.insert(out.end(), p, p + sizeof(pose));
    return out;
}

link::DevicePose trackingPose(std::uint32_t id, link::DeviceKind kind,
                              float x = 0.0f, float y = 0.0f, float z = 0.0f)
{
    link::DevicePose pose;
    pose.deviceId = id;
    pose.tracking = link::TrackingState::Tracking;
    pose.deviceKind = kind;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.rotation.w = 1.0f;  // identity (xyzw)
    return pose;
}

// A connected OpenVrTracking backed by `pipe`, with an injected clock.
struct TrackingFixture
{
    double clock = 0.0;
    link_test::FakePipe pipe;
    OpenVrTracking vr;

    TrackingFixture()
        : vr(testLogger(), link_test::borrowPipeFactory(pipe),
             [this] { return clock; })
    {
        vr.init();
    }
};
} // namespace

// ---- init ------------------------------------------------------------------

TEST(OpenVrTracking, InitThrowsWhenTheDriverPipeIsAbsent)
{
    link_test::FakePipe pipe;
    pipe.createPipeResult = nullptr;  // CreateFileA fails: no server listening
    double clock = 0.0;
    OpenVrTracking vr(testLogger(), link_test::borrowPipeFactory(pipe),
                      [&] { return clock; });
    EXPECT_THROW(vr.init(), Error);
    EXPECT_FALSE(vr.isInitialized());
}

TEST(OpenVrTracking, InitSucceedsWhenThePipeConnects)
{
    TrackingFixture f;
    EXPECT_TRUE(f.vr.isInitialized());
}

// ---- pose mapping ----------------------------------------------------------

TEST(OpenVrTracking, MapsDevicePoseToTrackedDevice)
{
    TrackingFixture f;
    f.pipe.feedRead(poseFrame(trackingPose(11, link::DeviceKind::Controller, 0.5f, 1.0f, -2.0f)));

    std::vector<TrackedDevice> devices = f.vr.pollPoses();
    ASSERT_EQ(devices.size(), 1u);
    EXPECT_EQ(devices[0].id, 11);
    EXPECT_EQ(devices[0].kind, TrackedDeviceKind::Controller);
    EXPECT_FLOAT_EQ(devices[0].pose.position.x, 0.5f);
    EXPECT_FLOAT_EQ(devices[0].pose.position.y, 1.0f);
    EXPECT_FLOAT_EQ(devices[0].pose.position.z, -2.0f);
    EXPECT_FLOAT_EQ(devices[0].pose.rotation.w, 1.0f);
}

TEST(OpenVrTracking, MapsEachDeviceKind)
{
    TrackingFixture f;
    f.pipe.feedRead(poseFrame(trackingPose(0, link::DeviceKind::Hmd)));
    f.pipe.feedRead(poseFrame(trackingPose(1, link::DeviceKind::Controller)));
    f.pipe.feedRead(poseFrame(trackingPose(2, link::DeviceKind::Tracker)));

    std::vector<TrackedDevice> devices = f.vr.pollPoses();
    ASSERT_EQ(devices.size(), 3u);
    EXPECT_EQ(devices[0].kind, TrackedDeviceKind::Hmd);
    EXPECT_EQ(devices[1].kind, TrackedDeviceKind::Controller);
    EXPECT_EQ(devices[2].kind, TrackedDeviceKind::Tracker);
}

TEST(OpenVrTracking, SkipsOtherKindDevices)
{
    TrackingFixture f;
    f.pipe.feedRead(poseFrame(trackingPose(0, link::DeviceKind::Hmd)));
    f.pipe.feedRead(poseFrame(trackingPose(1, link::DeviceKind::Other)));

    std::vector<TrackedDevice> devices = f.vr.pollPoses();
    ASSERT_EQ(devices.size(), 1u);
    EXPECT_EQ(devices[0].id, 0);
}

TEST(OpenVrTracking, LostFrameRemovesTheDevice)
{
    TrackingFixture f;
    f.pipe.feedRead(poseFrame(trackingPose(5, link::DeviceKind::Tracker)));
    ASSERT_EQ(f.vr.pollPoses().size(), 1u);

    link::DevicePose lost = trackingPose(5, link::DeviceKind::Tracker);
    lost.tracking = link::TrackingState::Lost;
    f.pipe.feedRead(poseFrame(lost));

    EXPECT_TRUE(f.vr.pollPoses().empty());
}

TEST(OpenVrTracking, SnapshotIsOrderedByDeviceIdAscending)
{
    TrackingFixture f;
    f.pipe.feedRead(poseFrame(trackingPose(8, link::DeviceKind::Tracker)));
    f.pipe.feedRead(poseFrame(trackingPose(2, link::DeviceKind::Tracker)));
    f.pipe.feedRead(poseFrame(trackingPose(5, link::DeviceKind::Tracker)));

    std::vector<TrackedDevice> devices = f.vr.pollPoses();
    ASSERT_EQ(devices.size(), 3u);
    EXPECT_EQ(devices[0].id, 2);
    EXPECT_EQ(devices[1].id, 5);
    EXPECT_EQ(devices[2].id, 8);
}

TEST(OpenVrTracking, DeviceSurvivesFramesWithNoUpdate)
{
    TrackingFixture f;
    f.pipe.feedRead(poseFrame(trackingPose(3, link::DeviceKind::Hmd, 1.0f)));
    ASSERT_EQ(f.vr.pollPoses().size(), 1u);

    // No new bytes this frame — the device stays in the snapshot at its pose.
    std::vector<TrackedDevice> devices = f.vr.pollPoses();
    ASSERT_EQ(devices.size(), 1u);
    EXPECT_EQ(devices[0].id, 3);
    EXPECT_FLOAT_EQ(devices[0].pose.position.x, 1.0f);
}

TEST(OpenVrTracking, UpdateOverwritesAnExistingDevice)
{
    TrackingFixture f;
    f.pipe.feedRead(poseFrame(trackingPose(7, link::DeviceKind::Hmd, 1.0f)));
    f.vr.pollPoses();

    f.pipe.feedRead(poseFrame(trackingPose(7, link::DeviceKind::Hmd, 9.0f)));
    std::vector<TrackedDevice> devices = f.vr.pollPoses();
    ASSERT_EQ(devices.size(), 1u);
    EXPECT_FLOAT_EQ(devices[0].pose.position.x, 9.0f);
}

// ---- reconnect throttle ----------------------------------------------------

TEST(OpenVrTracking, ReconnectsAtMostOncePerSecond)
{
    TrackingFixture f;

    // Drop the live pipe (peer closed), then make createPipe fail so every
    // reconnect attempt stays disconnected — the throttle is what matters.
    f.pipe.readEmptyErr = link::errBrokenPipe;
    f.pipe.readQueue.clear();
    f.vr.pollPoses();  // drains -> detects the drop -> disconnected
    ASSERT_FALSE(f.vr.isInitialized());
    const int callsAfterDrop = f.pipe.createPipeCallCount;
    f.pipe.createPipeResult = nullptr;

    // First poll after the drop: not throttled (nextAttemptAt_ == 0), attempts.
    f.clock = 0.0;
    f.vr.pollPoses();
    EXPECT_EQ(f.pipe.createPipeCallCount, callsAfterDrop + 1);

    // Half a second later: throttled, no new attempt.
    f.clock = 0.5;
    f.vr.pollPoses();
    EXPECT_EQ(f.pipe.createPipeCallCount, callsAfterDrop + 1);

    // Past the 1 s deadline: attempts again.
    f.clock = 1.5;
    f.vr.pollPoses();
    EXPECT_EQ(f.pipe.createPipeCallCount, callsAfterDrop + 2);
}

TEST(OpenVrTracking, IsInitializedFalseAfterADrop)
{
    TrackingFixture f;
    ASSERT_TRUE(f.vr.isInitialized());

    f.pipe.readEmptyErr = link::errBrokenPipe;
    f.pipe.readQueue.clear();
    f.vr.pollPoses();
    EXPECT_FALSE(f.vr.isInitialized());
}

// ---- sendOffsets: upstream PoseOverride frames -----------------------------

TEST(OpenVrTracking, SendOffsetsWritesOneFramePerDevice)
{
    TrackingFixture f;

    std::vector<DeviceOffset> offsets;
    offsets.push_back({1, {glm::vec3(0.1f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}});
    offsets.push_back({2, {glm::vec3(0.0f, 0.2f, 0.0f), glm::angleAxis(0.5f, glm::vec3(0.0f, 1.0f, 0.0f))}});
    f.vr.sendOffsets(offsets);

    const std::size_t oneFrame = 8u + sizeof(link::PoseOverride);
    ASSERT_EQ(f.pipe.written.size(), oneFrame * 2);

    link::PoseOverride first;
    std::memcpy(&first, f.pipe.written.data() + 8, sizeof(first));
    EXPECT_EQ(first.deviceId, 1u);
    EXPECT_FLOAT_EQ(first.position.x, 0.1f);

    link::PoseOverride second;
    std::memcpy(&second, f.pipe.written.data() + oneFrame + 8, sizeof(second));
    EXPECT_EQ(second.deviceId, 2u);
    EXPECT_FLOAT_EQ(second.position.y, 0.2f);
}

// ---- sendVirtualTrackers: upstream VirtualTracker frames (step 5) -----------

TEST(OpenVrTracking, SendVirtualTrackersWritesOneFramePerBone)
{
    TrackingFixture f;

    std::vector<VirtualTrackerPose> trackers;
    trackers.push_back({"Chest", {glm::vec3(0.1f, 1.2f, 0.0f),
                                  glm::angleAxis(0.5f, glm::vec3(0.0f, 1.0f, 0.0f))}});
    trackers.push_back({"Spine", {glm::vec3(0.0f, 0.9f, -0.1f),
                                  glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}});
    f.vr.sendVirtualTrackers(trackers);

    const std::size_t oneFrame = 8u + sizeof(link::VirtualTracker);
    ASSERT_EQ(f.pipe.written.size(), oneFrame * 2);

    link::VirtualTracker first;
    std::memcpy(&first, f.pipe.written.data() + 8, sizeof(first));
    EXPECT_EQ(std::string(first.name), "Chest");
    EXPECT_EQ(first.tracking, link::TrackingState::Tracking);
    EXPECT_FLOAT_EQ(first.position.x, 0.1f);
    EXPECT_FLOAT_EQ(first.position.y, 1.2f);

    link::VirtualTracker second;
    std::memcpy(&second, f.pipe.written.data() + oneFrame + 8, sizeof(second));
    EXPECT_EQ(std::string(second.name), "Spine");
    EXPECT_FLOAT_EQ(second.position.y, 0.9f);
    EXPECT_FLOAT_EQ(second.position.z, -0.1f);
}

TEST(OpenVrTracking, SendVirtualTrackersTruncatesLongBoneNames)
{
    TrackingFixture f;

    VirtualTrackerPose tracker;
    tracker.name = std::string(link::kMaxBoneNameBytes, 'X');  // one past the field
    tracker.pose = {glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    f.vr.sendVirtualTrackers({tracker});

    link::VirtualTracker wire;
    std::memcpy(&wire, f.pipe.written.data() + 8, sizeof(wire));
    EXPECT_EQ(wire.name[link::kMaxBoneNameBytes - 1], '\0');
    EXPECT_EQ(std::string(wire.name), std::string(link::kMaxBoneNameBytes - 1, 'X'));
}

TEST(OpenVrTracking, InboundPoseOverrideDoesNotBecomeATrackedDevice)
{
    TrackingFixture f;

    link::PoseOverride ov;
    ov.deviceId = 5;
    ov.position.x = 1.0f;

    std::vector<std::uint8_t> bytes;
    const std::uint32_t len = static_cast<std::uint32_t>(sizeof(ov));
    bytes.push_back(static_cast<std::uint8_t>(len & 0xFF));
    bytes.push_back(static_cast<std::uint8_t>((len >> 8) & 0xFF));
    bytes.push_back(static_cast<std::uint8_t>((len >> 16) & 0xFF));
    bytes.push_back(static_cast<std::uint8_t>((len >> 24) & 0xFF));
    const std::uint16_t t = static_cast<std::uint16_t>(link::MessageType::PoseOverride);
    bytes.push_back(static_cast<std::uint8_t>(t & 0xFF));
    bytes.push_back(static_cast<std::uint8_t>((t >> 8) & 0xFF));
    bytes.push_back(0);
    bytes.push_back(0);
    const std::uint8_t* p = reinterpret_cast<const std::uint8_t*>(&ov);
    bytes.insert(bytes.end(), p, p + sizeof(ov));

    f.pipe.feedRead(bytes);
    const auto devices = f.vr.pollPoses();
    EXPECT_TRUE(devices.empty());
}