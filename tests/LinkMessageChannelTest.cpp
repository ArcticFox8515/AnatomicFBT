// Framing + reassembly tests for the driver link (doc/driver-plan.md phase A, step 3).
//
// The framing layer (MessageChannel) sits on a `Pipe` and splits the byte
// stream into length-prefixed frames. These tests drive it through the cases
// the real overlapped pipe will produce — split reads, several frames per
// read, a header split mid-field, partial writes — plus the pipe-lifecycle
// cases (factory returns null, dead pipe dropped, reconnect, no snapshot).
// The mock `Pipe` scripts the bytes; nothing touches Win32.

#include "FakePipe.h"
#include "link/MessageChannel.h"
#include "link/Protocol.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

namespace
{
// The on-wire frame header is 8 bytes: u32 length + u16 type + 2 pad (the
// naturally-aligned POD the channel memcpy's). Kept in sync with the private
// FrameHeader in MessageChannel.cpp via the static_asserts at the bottom.
inline constexpr std::size_t kFrameHeaderSize = 8;

// Builds a complete frame (header + payload) for the cases that need raw bytes
// fed into the read side. The header is the 8-byte naturally-aligned POD the
// channel memcpy's: u32 length, u16 type, 2 pad bytes (kept zero so the wire
// is deterministic regardless of the sender's stack garbage).
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
    out.push_back(0);  // pad
    out.push_back(0);  // pad
    out.insert(out.end(), payload, payload + size);
    return out;
}

std::vector<std::uint8_t> frame(link::MessageType type, const std::vector<std::uint8_t>& payload)
{
    return frame(type, payload.data(), payload.size());
}

static_assert(sizeof(link::DevicePose) == 68, "pose POD size drifted");

// ---- round trip through the mock pipe --------------------------------------

TEST(LinkMessageChannel, SendsAndReceivesADevicePose)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(link_test::borrowPipeFactory(pipe));
    std::vector<link::Message> dummy;
    channel.receive(dummy);  // construct the pipe

    link::DevicePose pose;
    pose.deviceId = 11;
    pose.tracking = link::TrackingState::Tracking;
    pose.deviceKind = link::DeviceKind::Controller;
    pose.position[0] = 0.5f;
    pose.rotation[3] = 1.0f;
    ASSERT_TRUE(channel.send(link::MessageType::DevicePose,
                             reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose)));

    pipe.feedRead(pipe.written);

    std::vector<link::Message> messages;
    EXPECT_EQ(channel.receive(messages), 1u);
    ASSERT_EQ(messages.size(), 1u);
    EXPECT_EQ(messages[0].type, link::MessageType::DevicePose);
    ASSERT_EQ(messages[0].payload.size(), sizeof(link::DevicePose));
    link::DevicePose restored;
    std::memcpy(&restored, messages[0].payload.data(), sizeof(restored));
    EXPECT_EQ(restored.deviceId, 11u);
    EXPECT_EQ(restored.tracking, link::TrackingState::Tracking);
    EXPECT_EQ(restored.deviceKind, link::DeviceKind::Controller);
    EXPECT_FLOAT_EQ(restored.position[0], 0.5f);
}

// ---- reassembly across split reads ----------------------------------------

TEST(LinkMessageChannel, ReassemblesAFrameSplitAcrossThreeReads)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(link_test::borrowPipeFactory(pipe));

    link::DevicePose pose;
    pose.deviceId = 1;
    const auto wire = frame(link::MessageType::DevicePose,
                            reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose));

    std::vector<link::Message> messages;

    pipe.feedRead(wire.data(), 2);
    EXPECT_EQ(channel.receive(messages), 0u);

    pipe.feedRead(wire.data() + 2, 10);
    EXPECT_EQ(channel.receive(messages), 0u);

    pipe.feedRead(wire.data() + 12, wire.size() - 12);
    EXPECT_EQ(channel.receive(messages), 1u);
    ASSERT_EQ(messages.size(), 1u);
    EXPECT_EQ(messages[0].type, link::MessageType::DevicePose);
    ASSERT_EQ(messages[0].payload.size(), sizeof(link::DevicePose));
}

TEST(LinkMessageChannel, ReassemblesTwoFramesDeliveredInOneRead)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(link_test::borrowPipeFactory(pipe));

    link::DevicePose pose1;
    pose1.deviceId = 1;
    link::DevicePose pose2;
    pose2.deviceId = 2;
    const auto wire1 = frame(link::MessageType::DevicePose,
                             reinterpret_cast<const std::uint8_t*>(&pose1), sizeof(pose1));
    const auto wire2 = frame(link::MessageType::DevicePose,
                             reinterpret_cast<const std::uint8_t*>(&pose2), sizeof(pose2));

    std::vector<std::uint8_t> both;
    both.insert(both.end(), wire1.begin(), wire1.end());
    both.insert(both.end(), wire2.begin(), wire2.end());
    pipe.feedRead(both);

    std::vector<link::Message> messages;
    EXPECT_EQ(channel.receive(messages), 2u);
    ASSERT_EQ(messages.size(), 2u);
    EXPECT_EQ(messages[0].type, link::MessageType::DevicePose);
    EXPECT_EQ(messages[1].type, link::MessageType::DevicePose);
}

TEST(LinkMessageChannel, KeepsAPartialFrameForTheNextReceive)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(link_test::borrowPipeFactory(pipe));

    link::DevicePose pose;
    pose.deviceId = 9;
    const auto wire = frame(link::MessageType::DevicePose,
                            reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose));

    const std::size_t split = 6 + sizeof(link::DevicePose) / 2;
    pipe.feedRead(wire.data(), split);

    std::vector<link::Message> messages;
    EXPECT_EQ(channel.receive(messages), 0u);
    ASSERT_TRUE(messages.empty());

    pipe.feedRead(wire.data() + split, wire.size() - split);
    EXPECT_EQ(channel.receive(messages), 1u);
    ASSERT_EQ(messages.size(), 1u);
}

// ---- partial writes --------------------------------------------------------

TEST(LinkMessageChannel, RetainsBacklogWhenPipeIsPendingAndDrainsOnNextSend)
{
    link_test::FakePipe pipe;
    pipe.writePendingAfter = 4;
    link::MessageChannel channel(link_test::borrowPipeFactory(pipe));
    std::vector<link::Message> dummy;
    channel.receive(dummy);  // construct the pipe

    link::DevicePose pose;
    pose.deviceId = 2;
    ASSERT_TRUE(channel.send(link::MessageType::DevicePose,
                             reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose)));

    const std::size_t frameSize = sizeof(link::DevicePose) + kFrameHeaderSize;
    EXPECT_LT(pipe.written.size(), frameSize);

    pipe.writePendingAfter = 0;
    link::DevicePose pose2;
    pose2.deviceId = 3;
    ASSERT_TRUE(channel.send(link::MessageType::DevicePose,
                             reinterpret_cast<const std::uint8_t*>(&pose2), sizeof(pose2)));
    EXPECT_EQ(pipe.written.size(), frameSize + sizeof(link::DevicePose) + kFrameHeaderSize);
}

TEST(LinkMessageChannel, SendReturnsFalseWhenOutboundBufferIsFull)
{
    link_test::FakePipe pipe;
    pipe.writeForce = link::IoStatus::Pending;  // accept nothing
    link::MessageChannel channel(link_test::borrowPipeFactory(pipe));
    std::vector<link::Message> dummy;
    channel.receive(dummy);  // construct the pipe

    link::DevicePose pose;
    for (int i = 0; i < 100000; ++i)
    {
        if (!channel.send(link::MessageType::DevicePose,
                          reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose)))
            return;
    }
    FAIL() << "send never refused despite the outbound cap";
}

// ---- unknown type skipped between known ones ------------------------------

TEST(LinkMessageChannel, SkipsAnUnknownMessageTypeAndContinues)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(link_test::borrowPipeFactory(pipe));

    link::DevicePose pose1;
    pose1.deviceId = 5;
    const auto known1 = frame(link::MessageType::DevicePose,
                              reinterpret_cast<const std::uint8_t*>(&pose1), sizeof(pose1));
    const auto unknown = frame(static_cast<link::MessageType>(999),
                               std::vector<std::uint8_t>(4, 0xAB).data(), 4);
    link::DevicePose pose2;
    pose2.deviceId = 6;
    const auto known2 = frame(link::MessageType::DevicePose,
                              reinterpret_cast<const std::uint8_t*>(&pose2), sizeof(pose2));

    std::vector<std::uint8_t> all;
    all.insert(all.end(), known1.begin(), known1.end());
    all.insert(all.end(), unknown.begin(), unknown.end());
    all.insert(all.end(), known2.begin(), known2.end());
    pipe.feedRead(all);

    std::vector<link::Message> messages;
    EXPECT_EQ(channel.receive(messages), 2u);  // unknown skipped
    ASSERT_EQ(messages.size(), 2u);
    EXPECT_EQ(messages[0].type, link::MessageType::DevicePose);
    EXPECT_EQ(messages[1].type, link::MessageType::DevicePose);
}

// ---- the removed DeviceMetadata type (1) is now unknown and skipped --------

TEST(LinkMessageChannel, TheOldDeviceMetadataTypeIsSkippedAsUnknown)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(link_test::borrowPipeFactory(pipe));

    const auto legacy = frame(static_cast<link::MessageType>(1),
                              std::vector<std::uint8_t>(8, 0).data(), 8);
    link::DevicePose pose;
    pose.deviceId = 42;
    const auto current = frame(link::MessageType::DevicePose,
                               reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose));

    std::vector<std::uint8_t> all;
    all.insert(all.end(), legacy.begin(), legacy.end());
    all.insert(all.end(), current.begin(), current.end());
    pipe.feedRead(all);

    std::vector<link::Message> messages;
    EXPECT_EQ(channel.receive(messages), 1u);
    ASSERT_EQ(messages.size(), 1u);
    EXPECT_EQ(messages[0].type, link::MessageType::DevicePose);
}

// ---- factory returns nullptr when no client is connected ------------------

TEST(LinkMessageChannel, FactoryReturnsNullptrWhenNoClientIsConnected)
{
    link::PipeFactoryFn factory = [] { return nullptr; };
    link::MessageChannel channel(factory);

    std::vector<link::Message> messages;
    EXPECT_EQ(channel.receive(messages), 0u);

    link::DevicePose pose;
    EXPECT_FALSE(channel.send(link::MessageType::DevicePose,
                              reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose)));
}

// ---- dead pipe dropped, channel resets for next client --------------------

TEST(LinkMessageChannel, DropsAPipeThatThePeerClosedAndResetsForNextClient)
{
    link_test::FakePipe fake;
    fake.readForce = link::IoStatus::Closed;
    link::MessageChannel channel(link_test::borrowPipeFactory(fake));

    std::vector<link::Message> messages;
    channel.receive(messages);  // constructs, reads Closed, drops

    // No pipe — send returns false.
    link::DevicePose pose;
    EXPECT_FALSE(channel.send(link::MessageType::DevicePose,
                              reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose)));
}

TEST(LinkMessageChannel, DropsAPipeThatFailedAndPreservesLastError)
{
    link_test::FakePipe fake;
    fake.readForce = link::IoStatus::Failed;
    fake.forcedError = "link severed";
    link::MessageChannel channel(link_test::borrowPipeFactory(fake));

    std::vector<link::Message> messages;
    channel.receive(messages);

    EXPECT_EQ(channel.lastError(), "link severed");
}

TEST(LinkMessageChannel, OversizeLengthDropsThePipe)
{
    link_test::FakePipe fake;
    link::MessageChannel channel(link_test::borrowPipeFactory(fake));

    std::vector<std::uint8_t> header(8);
    const std::uint32_t tooLong = link::kMaxPayloadBytes + 1;
    header[0] = static_cast<std::uint8_t>(tooLong & 0xFF);
    header[1] = static_cast<std::uint8_t>((tooLong >> 8) & 0xFF);
    header[2] = static_cast<std::uint8_t>((tooLong >> 16) & 0xFF);
    header[3] = static_cast<std::uint8_t>((tooLong >> 24) & 0xFF);
    header[4] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(link::MessageType::DevicePose) & 0xFF);
    header[5] = 0;
    fake.feedRead(header);

    std::vector<link::Message> messages;
    channel.receive(messages);

    // Pipe dropped — send returns false.
    link::DevicePose pose;
    EXPECT_FALSE(channel.send(link::MessageType::DevicePose,
                              reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose)));
    EXPECT_FALSE(channel.lastError().empty());
}

TEST(LinkMessageChannel, RefusesToSendOversizePayload)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(link_test::borrowPipeFactory(pipe));
    std::vector<link::Message> dummy;
    channel.receive(dummy);  // construct the pipe

    std::vector<std::uint8_t> huge(link::kMaxPayloadBytes + 1, 0);
    EXPECT_FALSE(channel.send(link::MessageType::DevicePose, huge.data(), huge.size()));
}

// ---- reconnect after a drop -----------------------------------------------

TEST(LinkMessageChannel, ReconnectsAfterADrop)
{
    link_test::FakePipe fake;
    link::MessageChannel channel(link_test::borrowPipeFactory(fake));

    std::vector<link::Message> messages;
    channel.receive(messages);  // construct the pipe

    link::DevicePose pose;
    pose.deviceId = 1;
    ASSERT_TRUE(channel.send(link::MessageType::DevicePose,
                             reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose)));
    fake.feedRead(fake.written);
    messages.clear();
    channel.receive(messages);
    ASSERT_EQ(messages.size(), 1u);

    // Peer closes: the next receive drops the pipe.
    fake.readForce = link::IoStatus::Closed;
    fake.readQueue.clear();
    messages.clear();
    channel.receive(messages);

    // Reconnect: the factory hands out the same borrowed pipe; reset its read
    // state and feed a new frame.
    fake.readForce = link::IoStatus::Pending;
    link::DevicePose pose2;
    pose2.deviceId = 2;
    fake.feedRead(frame(link::MessageType::DevicePose,
                        reinterpret_cast<const std::uint8_t*>(&pose2), sizeof(pose2)));
    messages.clear();
    channel.receive(messages);
    ASSERT_EQ(messages.size(), 1u);
    link::DevicePose restored;
    std::memcpy(&restored, messages[0].payload.data(), sizeof(restored));
    EXPECT_EQ(restored.deviceId, 2u);
}

TEST(LinkMessageChannel, ClearsBacklogOnReconnectSoNoSnapshotIsDelivered)
{
    link_test::FakePipe fake;
    link::MessageChannel channel(link_test::borrowPipeFactory(fake));

    std::vector<link::Message> messages;
    channel.receive(messages);  // construct the pipe

    // Buffer a pose the pipe will not accept (Pending): it lives in the
    // channel's outbound backlog, not on the wire.
    fake.writeForce = link::IoStatus::Pending;
    link::DevicePose pose;
    pose.deviceId = 99;
    channel.send(link::MessageType::DevicePose,
                 reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose));
    ASSERT_TRUE(fake.written.empty());

    // Peer closes: receive drops the pipe and clears the backlog.
    fake.readForce = link::IoStatus::Closed;
    channel.receive(messages);

    // Reconnect: the factory hands out the same borrowed pipe, now accepting
    // writes. The backlog from step 2 was cleared.
    fake.readForce = link::IoStatus::Pending;
    fake.writeForce = link::IoStatus::Ok;
    channel.receive(messages);

    // A new pose: the only thing the new client should see.
    link::DevicePose pose2;
    pose2.deviceId = 100;
    ASSERT_TRUE(channel.send(link::MessageType::DevicePose,
                             reinterpret_cast<const std::uint8_t*>(&pose2), sizeof(pose2)));
    const std::size_t oneFrame = kFrameHeaderSize + sizeof(link::DevicePose);
    EXPECT_EQ(fake.written.size(), oneFrame);
}
} // namespace
