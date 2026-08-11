#include <gtest/gtest.h>

#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>

#include "model/Error.h"
#include "model/GripOffsets.h"
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

    writer.writeFrame(0.0f, frame0, {});
    writer.writeFrame(0.011f, frame1, {});

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
    writer.writeFrame(0.0f, frame0, {});

    // Device 3 loses tracking: absent from the snapshot, like pollPoses.
    std::vector<TrackedDevice> frame1 = {frame0[0], frame0[2]};
    frame1[0].pose.position.y = 1.8f;
    writer.writeFrame(0.011f, frame1, {});

    // It comes back with a new pose.
    std::vector<TrackedDevice> frame2 = frame0;
    frame2[1].pose.position = {0.7f, 1.3f, -0.2f};
    writer.writeFrame(0.022f, frame2, {});

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
    writer.writeFrame(0.0f, frame0, {});

    std::vector<TrackedDevice> frame1 = frame0;
    frame1.push_back(makeDevice(42, TrackedDeviceKind::Tracker, {5.0f, 5.0f, 5.0f}));
    writer.writeFrame(0.011f, frame1, {});

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
    writer.writeFrame(0.0f, frame0Devices(), {});

    // Header (4+2) + roster count (4) + 3 entries (33 each: id, kind, grip
    // pose) = 113 bytes; keep only those, cutting the first frame off entirely.
    std::stringstream truncated(stream.str().substr(0, 113),
                                std::ios::in | std::ios::out | std::ios::binary);
    EXPECT_THROW(loadRecording(truncated), Error);
}

TEST(Recording, TruncatedTrailingFrameIsDropped)
{
    std::stringstream stream(std::ios::in | std::ios::out | std::ios::binary);
    RecordingWriter writer(stream);
    const std::vector<TrackedDevice> frame0 = frame0Devices();
    writer.writeFrame(0.0f, frame0, {});
    writer.writeFrame(0.011f, frame0, {});

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

// --- v2: grip offsets in the roster -----------------------------------------

// A controller at the origin with identity rotation; its grip offset is a
// pure translation of [0, 0.003, 0.097] (the knuckles grip origin). The loaded
// frame's controller pose should be shifted by that offset; the HMD and
// tracker in the same roster stay put.
TEST(Recording, V2StoresGripOffsetsAndLoadsGripAppliedFrames)
{
    std::stringstream stream(std::ios::in | std::ios::out | std::ios::binary);
    RecordingWriter writer(stream);

    const std::vector<TrackedDevice> frame0 = {
        makeDevice(0, TrackedDeviceKind::Hmd, {0.0f, 1.7f, 0.0f}),
        makeDevice(3, TrackedDeviceKind::Controller, {0.0f, 0.0f, 0.0f}),
        makeDevice(7, TrackedDeviceKind::Tracker, {0.1f, 0.1f, 0.05f}),
    };
    const std::vector<GripOffset> grips = {
        {3, {glm::vec3(0.0f, 0.003f, 0.097f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}},
    };

    writer.writeFrame(0.0f, frame0, grips);

    const Recording recording = loadRecording(stream);
    ASSERT_EQ(recording.frames.size(), 1u);
    ASSERT_EQ(recording.frames[0].devices.size(), 3u);

    // The roster carries the grip offset keyed by the controller's id.
    ASSERT_EQ(recording.gripOffsets.size(), 3u);
    EXPECT_EQ(recording.gripOffsets[1].deviceId, 3);
    EXPECT_FLOAT_EQ(recording.gripOffsets[1].deviceToGrip.position.z, 0.097f);
    // Non-controllers get identity in the roster.
    EXPECT_EQ(recording.gripOffsets[0].deviceId, 0);
    EXPECT_EQ(recording.gripOffsets[0].deviceToGrip.rotation,
              glm::quat(1.0f, 0.0f, 0.0f, 0.0f));

    // The loaded controller pose is grip-applied (composed with the offset
    // in the controller's local frame = identity rotation, so plain add).
    EXPECT_FLOAT_EQ(recording.frames[0].devices[1].pose.position.x, 0.0f);
    EXPECT_FLOAT_EQ(recording.frames[0].devices[1].pose.position.y, 0.003f);
    EXPECT_FLOAT_EQ(recording.frames[0].devices[1].pose.position.z, 0.097f);
    // The HMD and tracker are untouched.
    EXPECT_EQ(recording.frames[0].devices[0].pose.position, glm::vec3(0.0f, 1.7f, 0.0f));
    EXPECT_EQ(recording.frames[0].devices[2].pose.position, glm::vec3(0.1f, 0.1f, 0.05f));
}

// The grip offset's rotation composes onto the device rotation, not just the
// translation. A controller with a non-identity rotation and a grip offset
// carrying a rotation must land at the rotated offset.
TEST(Recording, V2GripOffsetRotationComposesOnLoad)
{
    std::stringstream stream(std::ios::in | std::ios::out | std::ios::binary);
    RecordingWriter writer(stream);

    const glm::quat rotY = glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.0f, 1.0f, 0.0f));
    const std::vector<TrackedDevice> frame0 = {
        makeDevice(3, TrackedDeviceKind::Controller, {0.0f, 0.0f, 0.0f}, rotY),
    };
    // Grip offset = translate [0,0,0.1] in the controller's local frame.
    // With the controller rotated 90 deg about Y, the grip point lands at
    // world [0.1, 0, 0].
    const std::vector<GripOffset> grips = {
        {3, {glm::vec3(0.0f, 0.0f, 0.1f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}},
    };

    writer.writeFrame(0.0f, frame0, grips);

    const Recording recording = loadRecording(stream);
    ASSERT_EQ(recording.frames.size(), 1u);
    EXPECT_NEAR(recording.frames[0].devices[0].pose.position.x, 0.1f, 1e-6f);
    EXPECT_NEAR(recording.frames[0].devices[0].pose.position.y, 0.0f, 1e-6f);
    EXPECT_NEAR(recording.frames[0].devices[0].pose.position.z, 0.0f, 1e-6f);
}

// Later frames in a session keep the roster's grip offsets — only the poses
// refresh, the offsets are fixed at the roster freeze (frame 0).
TEST(Recording, V2LaterFramesKeepFrame0GripOffsets)
{
    std::stringstream stream(std::ios::in | std::ios::out | std::ios::binary);
    RecordingWriter writer(stream);

    const std::vector<TrackedDevice> frame0 = {
        makeDevice(3, TrackedDeviceKind::Controller, {0.0f, 0.0f, 0.0f}),
    };
    const std::vector<GripOffset> grips = {
        {3, {glm::vec3(0.0f, 0.0f, 0.097f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}},
    };
    std::vector<TrackedDevice> frame1 = frame0;
    frame1[0].pose.position = {1.0f, 0.0f, 0.0f};  // controller moves in X

    writer.writeFrame(0.0f, frame0, grips);
    writer.writeFrame(0.011f, frame1, grips);

    const Recording recording = loadRecording(stream);
    ASSERT_EQ(recording.frames.size(), 2u);
    // Frame 1: raw position [1,0,0] + grip [0,0,0.097] (identity rotation) =
    // [1, 0, 0.097].
    EXPECT_FLOAT_EQ(recording.frames[1].devices[0].pose.position.x, 1.0f);
    EXPECT_FLOAT_EQ(recording.frames[1].devices[0].pose.position.z, 0.097f);
}

// --- v1 backward compatibility ---------------------------------------------

// A hand-built v1 byte buffer (roster = id + kind, no grip offsets) loads
// with identity offsets and untouched poses — the same behavior a recording
// written by the previous version has.
TEST(Recording, LoadsV1RecordingsWithIdentityGripOffsets)
{
    constexpr std::uint32_t kMagic = 0x31524354;  // "TCR1"
    constexpr std::uint16_t kVersion1 = 1;
    constexpr std::uint32_t kRosterSize = 2;

    const std::vector<TrackedDevice> frame0 = {
        makeDevice(0, TrackedDeviceKind::Hmd, {0.0f, 1.7f, 0.0f}),
        makeDevice(3, TrackedDeviceKind::Controller, {0.6f, 1.2f, -0.1f}),
    };

    std::string bytes;
    auto append = [&](const void* p, std::size_t n) {
        bytes.append(static_cast<const char*>(p), n);
    };
    append(&kMagic, 4);
    append(&kVersion1, 2);
    append(&kRosterSize, 4);
    for (const TrackedDevice& d : frame0)
    {
        const std::int32_t id = d.id;
        const std::uint8_t kind = static_cast<std::uint8_t>(d.kind);
        append(&id, 4);
        append(&kind, 1);
    }
    const float time = 0.0f;
    append(&time, 4);
    for (const TrackedDevice& d : frame0)
    {
        append(&d.pose.position.x, 4);
        append(&d.pose.position.y, 4);
        append(&d.pose.position.z, 4);
        append(&d.pose.rotation.x, 4);
        append(&d.pose.rotation.y, 4);
        append(&d.pose.rotation.z, 4);
        append(&d.pose.rotation.w, 4);
    }

    std::stringstream stream(bytes, std::ios::in | std::ios::out | std::ios::binary);
    const Recording recording = loadRecording(stream);

    ASSERT_EQ(recording.frames.size(), 1u);
    ASSERT_EQ(recording.frames[0].devices.size(), 2u);
    // Poses are untouched (identity grip offsets — no shift applied).
    expectSameDevice(recording.frames[0].devices[0], frame0[0]);
    expectSameDevice(recording.frames[0].devices[1], frame0[1]);
    // Grip offsets are all identity (v1 has none on the wire).
    ASSERT_EQ(recording.gripOffsets.size(), 2u);
    for (const GripOffset& o : recording.gripOffsets)
    {
        EXPECT_EQ(o.deviceToGrip.position, glm::vec3(0.0f, 0.0f, 0.0f));
        EXPECT_EQ(o.deviceToGrip.rotation, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    }
}
