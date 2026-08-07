// Wire-protocol round-trip tests for the driver link (doc/driver-plan.md phase A, step 2).
//
// There is no codec: a message is a packed POD, (de)serialization is a memcpy.
// What is tested here is that the POD layout is stable and that a memcpy
// round-trip preserves every field bit-for-bit, plus the pinned wire values of
// the enums (an existing driver speaks the old numbers) and the exact on-wire
// size the framing layer will see.

#include "link/Protocol.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

namespace
{
TEST(LinkProtocol, DeviceMetadataRoundTripsThroughMemcpy)
{
    link::DeviceMetadata original;
    original.deviceId = 7;
    original.kind = link::DeviceKind::Tracker;

    std::vector<std::uint8_t> wire(sizeof(link::DeviceMetadata));
    std::memcpy(wire.data(), &original, sizeof(link::DeviceMetadata));

    link::DeviceMetadata restored;
    std::memcpy(&restored, wire.data(), sizeof(link::DeviceMetadata));
    EXPECT_EQ(restored.deviceId, 7u);
    EXPECT_EQ(restored.kind, link::DeviceKind::Tracker);
}

TEST(LinkProtocol, DevicePoseRoundTripsThroughMemcpy)
{
    link::DevicePose original;
    original.deviceId = 3;
    original.tracking = link::TrackingState::Tracking;
    original.position[0] = 1.5f;
    original.position[1] = -0.25f;
    original.position[2] = 2.0f;
    original.rotation[0] = 0.1f;  // x
    original.rotation[1] = 0.2f;  // y
    original.rotation[2] = 0.3f;  // z
    original.rotation[3] = 0.9f;  // w

    std::vector<std::uint8_t> wire(sizeof(link::DevicePose));
    std::memcpy(wire.data(), &original, sizeof(link::DevicePose));

    link::DevicePose restored;
    std::memcpy(&restored, wire.data(), sizeof(link::DevicePose));
    EXPECT_EQ(restored.deviceId, 3u);
    EXPECT_EQ(restored.tracking, link::TrackingState::Tracking);
    EXPECT_FLOAT_EQ(restored.position[0], 1.5f);
    EXPECT_FLOAT_EQ(restored.position[1], -0.25f);
    EXPECT_FLOAT_EQ(restored.position[2], 2.0f);
    EXPECT_FLOAT_EQ(restored.rotation[0], 0.1f);
    EXPECT_FLOAT_EQ(restored.rotation[1], 0.2f);
    EXPECT_FLOAT_EQ(restored.rotation[2], 0.3f);
    EXPECT_FLOAT_EQ(restored.rotation[3], 0.9f);
}

TEST(LinkProtocol, DevicePoseDefaultsToIdentityRotation)
{
    link::DevicePose pose;
    EXPECT_EQ(pose.tracking, link::TrackingState::Lost);
    EXPECT_FLOAT_EQ(pose.rotation[3], 1.0f);  // w
    EXPECT_FLOAT_EQ(pose.rotation[0], 0.0f);  // x
}

// ---- pinned wire values: an existing driver speaks these numbers -----------

TEST(LinkProtocol, DeviceKindValuesArePinned)
{
    EXPECT_EQ(static_cast<std::uint8_t>(link::DeviceKind::Hmd), 0);
    EXPECT_EQ(static_cast<std::uint8_t>(link::DeviceKind::Controller), 1);
    EXPECT_EQ(static_cast<std::uint8_t>(link::DeviceKind::Tracker), 2);
    EXPECT_EQ(static_cast<std::uint8_t>(link::DeviceKind::Other), 3);
}

TEST(LinkProtocol, TrackingStateValuesArePinned)
{
    EXPECT_EQ(static_cast<std::uint8_t>(link::TrackingState::Lost), 0);
    EXPECT_EQ(static_cast<std::uint8_t>(link::TrackingState::Tracking), 1);
}

TEST(LinkProtocol, MessageTypeValuesArePinned)
{
    EXPECT_EQ(static_cast<std::uint16_t>(link::MessageType::DeviceMetadata), 1);
    EXPECT_EQ(static_cast<std::uint16_t>(link::MessageType::DevicePose), 2);
}

// ---- the framing layer keys on sizeof(POD); pin those sizes ---------------

static_assert(sizeof(link::DeviceMetadata) == 8, "metadata POD size drifted");
static_assert(sizeof(link::DevicePose) == 36, "pose POD size drifted");
} // namespace