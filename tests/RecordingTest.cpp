#include <gtest/gtest.h>

#include <glm/gtc/quaternion.hpp>

#include <sstream>
#include <string>

#include "model/Error.h"
#include "model/Recording.h"

namespace
{
TrackedDevice makeDevice(int id, TrackedDeviceKind kind, const glm::vec3& position,
                         const glm::quat& rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f))
{
    return {id, kind, {position, rotation}};
}

void expectSameDevice(const TrackedDevice& actual, const TrackedDevice& expected)
{
    EXPECT_EQ(actual.id, expected.id);
    EXPECT_EQ(actual.kind, expected.kind);
    // Byte-exact round trip: raw float equality is intentional.
    EXPECT_EQ(actual.pose.position.x, expected.pose.position.x);
    EXPECT_EQ(actual.pose.position.y, expected.pose.position.y);
    EXPECT_EQ(actual.pose.position.z, expected.pose.position.z);
    EXPECT_EQ(actual.pose.rotation.x, expected.pose.rotation.x);
    EXPECT_EQ(actual.pose.rotation.y, expected.pose.rotation.y);
    EXPECT_EQ(actual.pose.rotation.z, expected.pose.rotation.z);
    EXPECT_EQ(actual.pose.rotation.w, expected.pose.rotation.w);
}

std::vector<TrackedDevice> frame0Devices()
{
    return {
        makeDevice(0, TrackedDeviceKind::Hmd, {0.0f, 1.7f, 0.0f},
                   glm::quat(0.9f, 0.1f, 0.2f, 0.3f)),
        makeDevice(3, TrackedDeviceKind::Controller, {0.6f, 1.2f, -0.1f}),
        makeDevice(7, TrackedDeviceKind::Tracker, {0.1f, 0.1f, 0.05f}),
    };
}
} // namespace

TEST(Recording, RoundTripPreservesRosterTimesAndPoses)
{
    std::stringstream stream(std::ios::in | std::ios::out | std::ios::binary);
    RecordingWriter writer(stream);

    const std::vector<TrackedDevice> frame0 = frame0Devices();
    std::vector<TrackedDevice> frame1 = frame0;
    for (TrackedDevice& device : frame1)
        device.pose.position += glm::vec3(0.1f, -0.2f, 0.3f);

    writer.writeFrame(0.0f, frame0);
    writer.writeFrame(0.011f, frame1);

    const Recording recording = loadRecording(stream);
    ASSERT_EQ(recording.frames.size(), 2u);
    EXPECT_EQ(recording.frames[0].time, 0.0f);
    EXPECT_EQ(recording.frames[1].time, 0.011f);
    EXPECT_EQ(recording.duration(), 0.011f);
    ASSERT_EQ(recording.frames[0].devices.size(), frame0.size());
    ASSERT_EQ(recording.frames[1].devices.size(), frame1.size());
    for (size_t i = 0; i < frame0.size(); ++i)
    {
        expectSameDevice(recording.frames[0].devices[i], frame0[i]);
        expectSameDevice(recording.frames[1].devices[i], frame1[i]);
    }
}

TEST(Recording, DroppedDeviceRepeatsItsLastPose)
{
    std::stringstream stream(std::ios::in | std::ios::out | std::ios::binary);
    RecordingWriter writer(stream);

    const std::vector<TrackedDevice> frame0 = frame0Devices();
    writer.writeFrame(0.0f, frame0);

    // Device 3 loses tracking: absent from the snapshot, like pollPoses.
    std::vector<TrackedDevice> frame1 = {frame0[0], frame0[2]};
    frame1[0].pose.position.y = 1.8f;
    writer.writeFrame(0.011f, frame1);

    // It comes back with a new pose.
    std::vector<TrackedDevice> frame2 = frame0;
    frame2[1].pose.position = {0.7f, 1.3f, -0.2f};
    writer.writeFrame(0.022f, frame2);

    const Recording recording = loadRecording(stream);
    ASSERT_EQ(recording.frames.size(), 3u);
    // Every loaded frame carries the full roster.
    for (const RecordingFrame& frame : recording.frames)
        ASSERT_EQ(frame.devices.size(), frame0.size());
    // Frame 1: the dropped device repeats its frame-0 pose (the live effect
    // of a dropout — the target does not move), others move normally.
    expectSameDevice(recording.frames[1].devices[0], frame1[0]);
    expectSameDevice(recording.frames[1].devices[1], frame0[1]);
    // Frame 2: the returned device's new pose is picked up again.
    expectSameDevice(recording.frames[2].devices[1], frame2[1]);
}

TEST(Recording, DeviceUnknownToTheRosterIsIgnored)
{
    std::stringstream stream(std::ios::in | std::ios::out | std::ios::binary);
    RecordingWriter writer(stream);

    const std::vector<TrackedDevice> frame0 = frame0Devices();
    writer.writeFrame(0.0f, frame0);

    std::vector<TrackedDevice> frame1 = frame0;
    frame1.push_back(makeDevice(42, TrackedDeviceKind::Tracker, {5.0f, 5.0f, 5.0f}));
    writer.writeFrame(0.011f, frame1);

    const Recording recording = loadRecording(stream);
    ASSERT_EQ(recording.frames.size(), 2u);
    ASSERT_EQ(recording.frames[1].devices.size(), frame0.size());
    for (const TrackedDevice& device : recording.frames[1].devices)
        EXPECT_NE(device.id, 42);
}

TEST(Recording, LoadRejectsBadMagicAndEmptyStreams)
{
    std::stringstream empty(std::ios::in | std::ios::out | std::ios::binary);
    EXPECT_THROW(loadRecording(empty), Error);

    std::stringstream garbage(std::ios::in | std::ios::out | std::ios::binary);
    garbage << "this is not a recording file";
    EXPECT_THROW(loadRecording(garbage), Error);
}

TEST(Recording, LoadRejectsRecordingWithoutACompleteFrame)
{
    std::stringstream stream(std::ios::in | std::ios::out | std::ios::binary);
    RecordingWriter writer(stream);
    writer.writeFrame(0.0f, frame0Devices());

    // Header (4+2) + roster count (4) + 3 entries (5 each) = 25 bytes; keep
    // only those, cutting the first frame off entirely.
    std::stringstream truncated(stream.str().substr(0, 25),
                                std::ios::in | std::ios::out | std::ios::binary);
    EXPECT_THROW(loadRecording(truncated), Error);
}

TEST(Recording, TruncatedTrailingFrameIsDropped)
{
    std::stringstream stream(std::ios::in | std::ios::out | std::ios::binary);
    RecordingWriter writer(stream);
    const std::vector<TrackedDevice> frame0 = frame0Devices();
    writer.writeFrame(0.0f, frame0);
    writer.writeFrame(0.011f, frame0);

    // Chop a few bytes off the second frame, as a crashed session would.
    const std::string bytes = stream.str();
    std::stringstream truncated(bytes.substr(0, bytes.size() - 5),
                                std::ios::in | std::ios::out | std::ios::binary);

    const Recording recording = loadRecording(truncated);
    ASSERT_EQ(recording.frames.size(), 1u);
    EXPECT_EQ(recording.frames[0].time, 0.0f);
}

TEST(Recording, NearestFrameIndexSnapsToClosestTime)
{
    Recording recording;
    recording.frames.resize(3);
    recording.frames[0].time = 0.0f;
    recording.frames[1].time = 1.0f;
    recording.frames[2].time = 2.0f;

    EXPECT_EQ(nearestFrameIndex(recording, -1.0f), 0u);
    EXPECT_EQ(nearestFrameIndex(recording, 0.0f), 0u);
    EXPECT_EQ(nearestFrameIndex(recording, 0.4f), 0u);
    EXPECT_EQ(nearestFrameIndex(recording, 0.5f), 0u);  // tie -> earlier frame
    EXPECT_EQ(nearestFrameIndex(recording, 0.6f), 1u);
    EXPECT_EQ(nearestFrameIndex(recording, 1.6f), 2u);
    EXPECT_EQ(nearestFrameIndex(recording, 5.0f), 2u);

    EXPECT_THROW(nearestFrameIndex(Recording{}, 0.0f), Error);
}
