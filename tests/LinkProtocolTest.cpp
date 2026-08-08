// Wire-protocol round-trip tests for the driver link (doc/driver-plan.md phase A, step 3).
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
#include <string>
#include <vector>

namespace
{
TEST(LinkProtocol, DevicePoseRoundTripsThroughMemcpy)
{
    link::DevicePose original;
    original.deviceId = 3;
    original.tracking = link::TrackingState::Tracking;
    original.deviceKind = link::DeviceKind::Tracker;
    original.position[0] = 1.5f;
    original.position[1] = -0.25f;
    original.position[2] = 2.0f;
    original.rotation[0] = 0.1f;  // x
    original.rotation[1] = 0.2f;  // y
    original.rotation[2] = 0.3f;  // z
    original.rotation[3] = 0.9f;  // w
    const std::string serial = "LHR-ABC1234567";
    std::memcpy(original.serial, serial.c_str(), serial.size());

    std::vector<std::uint8_t> wire(sizeof(link::DevicePose));
    std::memcpy(wire.data(), &original, sizeof(link::DevicePose));

    link::DevicePose restored;
    std::memcpy(&restored, wire.data(), sizeof(link::DevicePose));
    EXPECT_EQ(restored.deviceId, 3u);
    EXPECT_EQ(restored.tracking, link::TrackingState::Tracking);
    EXPECT_EQ(restored.deviceKind, link::DeviceKind::Tracker);
    EXPECT_FLOAT_EQ(restored.position[0], 1.5f);
    EXPECT_FLOAT_EQ(restored.position[1], -0.25f);
    EXPECT_FLOAT_EQ(restored.position[2], 2.0f);
    EXPECT_FLOAT_EQ(restored.rotation[0], 0.1f);
    EXPECT_FLOAT_EQ(restored.rotation[1], 0.2f);
    EXPECT_FLOAT_EQ(restored.rotation[2], 0.3f);
    EXPECT_FLOAT_EQ(restored.rotation[3], 0.9f);
    EXPECT_STREQ(restored.serial, "LHR-ABC1234567");
}

TEST(LinkProtocol, DevicePoseDefaultsToIdentityRotationAndOtherKind)
{
    link::DevicePose pose;
    EXPECT_EQ(pose.tracking, link::TrackingState::Lost);
    EXPECT_EQ(pose.deviceKind, link::DeviceKind::Other);
    EXPECT_FLOAT_EQ(pose.rotation[3], 1.0f);  // w
    EXPECT_FLOAT_EQ(pose.rotation[0], 0.0f);  // x
    EXPECT_EQ(pose.serial[0], '\0');
}

TEST(LinkProtocol, ASerialLongerThanTheFieldIsTruncatedByTheSenderNotTheCodec)
{
    // The wire field is char[kMaxSerialBytes]; the sender is responsible for
    // truncating. The codec memcpy's the whole struct, so a full-width serial
    // (31 chars + NUL) round-trips exactly.
    link::DevicePose original;
    original.deviceId = 7;
    const std::string full(link::kMaxSerialBytes - 1, 'X');
    std::memcpy(original.serial, full.c_str(), full.size());

    std::vector<std::uint8_t> wire(sizeof(link::DevicePose));
    std::memcpy(wire.data(), &original, sizeof(link::DevicePose));

    link::DevicePose restored;
    std::memcpy(&restored, wire.data(), sizeof(link::DevicePose));
    EXPECT_EQ(std::string(restored.serial), full);
    EXPECT_EQ(restored.serial[link::kMaxSerialBytes - 1], '\0');
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
    // DeviceMetadata (1) was removed in step 3; DevicePose keeps its wire value
    // so an older driver/app that spoke type 2 still interoperates.
    EXPECT_EQ(static_cast<std::uint16_t>(link::MessageType::DevicePose), 2);
    EXPECT_EQ(static_cast<std::uint16_t>(link::MessageType::PoseOverride), 3);
}

// ---- the framing layer keys on sizeof(POD); pin those sizes ---------------

static_assert(sizeof(link::DevicePose) == 68, "pose POD size drifted");

TEST(LinkProtocol, PoseOverrideRoundTripsThroughMemcpy)
{
    link::PoseOverride original;
    original.deviceId = 5;
    original.position[0] = 0.1f;
    original.position[1] = -0.2f;
    original.position[2] = 0.3f;
    original.rotation[0] = 0.4f;  // x
    original.rotation[1] = 0.5f;  // y
    original.rotation[2] = 0.6f;  // z
    original.rotation[3] = 0.7f;  // w

    std::vector<std::uint8_t> wire(sizeof(link::PoseOverride));
    std::memcpy(wire.data(), &original, sizeof(link::PoseOverride));

    link::PoseOverride restored;
    std::memcpy(&restored, wire.data(), sizeof(link::PoseOverride));
    EXPECT_EQ(restored.deviceId, 5u);
    EXPECT_FLOAT_EQ(restored.position[0], 0.1f);
    EXPECT_FLOAT_EQ(restored.position[1], -0.2f);
    EXPECT_FLOAT_EQ(restored.position[2], 0.3f);
    EXPECT_FLOAT_EQ(restored.rotation[0], 0.4f);
    EXPECT_FLOAT_EQ(restored.rotation[1], 0.5f);
    EXPECT_FLOAT_EQ(restored.rotation[2], 0.6f);
    EXPECT_FLOAT_EQ(restored.rotation[3], 0.7f);
}

TEST(LinkProtocol, PoseOverrideDefaultsToIdentityRotation)
{
    link::PoseOverride pose;
    EXPECT_EQ(pose.deviceId, 0u);
    EXPECT_FLOAT_EQ(pose.position[0], 0.0f);
    EXPECT_FLOAT_EQ(pose.position[1], 0.0f);
    EXPECT_FLOAT_EQ(pose.position[2], 0.0f);
    EXPECT_FLOAT_EQ(pose.rotation[3], 1.0f);  // w
    EXPECT_FLOAT_EQ(pose.rotation[0], 0.0f);  // x
}

static_assert(sizeof(link::PoseOverride) == 32, "poseOverride POD size drifted");
} // namespace
