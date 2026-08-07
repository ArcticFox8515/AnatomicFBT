// Framing + reassembly tests for the driver link (doc/driver-plan.md phase A, step 2).
//
// The framing layer (MessageChannel) sits on a `Pipe` and splits the byte
// stream into length-prefixed frames. These tests drive it through the cases
// the real overlapped pipe will produce — split reads, several frames per
// read, a header split mid-field, partial writes — plus the two fatal
// conditions (oversize length, peer closed). The mock `Pipe` scripts the
// bytes; nothing touches Win32.

#include "FakePipe.h"
#include "link/MessageChannel.h"
#include "link/Protocol.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
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

// A frame header on the wire is 8 bytes: u32 length + u16 type + 2 pad (the
// naturally-aligned POD the channel memcpy's). The framer reads the whole
// struct; only length and type carry information.
static_assert(sizeof(link::DeviceMetadata) == 8, "metadata POD size drifted");
static_assert(sizeof(link::DevicePose) == 36, "pose POD size drifted");

// ---- round trip through the mock pipe -------------------------------------

TEST(LinkMessageChannel, SendsAndReceivesADevicePose)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(pipe);

    link::DevicePose pose;
    pose.deviceId = 11;
    pose.tracking = link::TrackingState::Tracking;
    pose.position[0] = 0.5f;
    pose.rotation[3] = 1.0f;
    ASSERT_TRUE(channel.send(link::MessageType::DevicePose,
                             reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose)));

    // The bytes the driver wrote are what the app reads.
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
    EXPECT_FLOAT_EQ(restored.position[0], 0.5f);
}

TEST(LinkMessageChannel, SendsAndReceivesDeviceMetadata)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(pipe);

    link::DeviceMetadata meta;
    meta.deviceId = 4;
    meta.kind = link::DeviceKind::Controller;
    ASSERT_TRUE(channel.send(link::MessageType::DeviceMetadata,
                             reinterpret_cast<const std::uint8_t*>(&meta), sizeof(meta)));

    pipe.feedRead(pipe.written);
    std::vector<link::Message> messages;
    channel.receive(messages);
    ASSERT_EQ(messages.size(), 1u);
    EXPECT_EQ(messages[0].type, link::MessageType::DeviceMetadata);
    ASSERT_EQ(messages[0].payload.size(), sizeof(link::DeviceMetadata));
    link::DeviceMetadata restored;
    std::memcpy(&restored, messages[0].payload.data(), sizeof(restored));
    EXPECT_EQ(restored.deviceId, 4u);
    EXPECT_EQ(restored.kind, link::DeviceKind::Controller);
}

// ---- reassembly across split reads ----------------------------------------

TEST(LinkMessageChannel, ReassemblesAFrameSplitAcrossThreeReads)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(pipe);

    link::DevicePose pose;
    pose.deviceId = 1;
    const auto wire = frame(link::MessageType::DevicePose,
                            reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose));

    // Three arbitrary chunks: header split mid-field, then mid-payload, then
    // tail. Each chunk is fed then drained before the next, so the mock
    // delivers exactly that fragment per receive call.
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
    link::MessageChannel channel(pipe);

    link::DeviceMetadata meta;
    meta.deviceId = 0;
    link::DevicePose pose;
    pose.deviceId = 1;
    const auto wire1 = frame(link::MessageType::DeviceMetadata,
                             reinterpret_cast<const std::uint8_t*>(&meta), sizeof(meta));
    const auto wire2 = frame(link::MessageType::DevicePose,
                             reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose));

    std::vector<std::uint8_t> both;
    both.insert(both.end(), wire1.begin(), wire1.end());
    both.insert(both.end(), wire2.begin(), wire2.end());
    pipe.feedRead(both);

    std::vector<link::Message> messages;
    EXPECT_EQ(channel.receive(messages), 2u);
    ASSERT_EQ(messages.size(), 2u);
    EXPECT_EQ(messages[0].type, link::MessageType::DeviceMetadata);
    EXPECT_EQ(messages[1].type, link::MessageType::DevicePose);
}

TEST(LinkMessageChannel, KeepsAPartialFrameForTheNextReceive)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(pipe);

    link::DevicePose pose;
    pose.deviceId = 9;
    const auto wire = frame(link::MessageType::DevicePose,
                            reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose));

    // First read: header + half payload. Second read: the rest.
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
    // Accept only 4 bytes total, then go Pending: the first send flushes a
    // fragment and leaves the rest buffered.
    pipe.writePendingAfter = 4;
    link::MessageChannel channel(pipe);

    link::DeviceMetadata meta;
    meta.deviceId = 2;
    ASSERT_TRUE(channel.send(link::MessageType::DeviceMetadata,
                             reinterpret_cast<const std::uint8_t*>(&meta), sizeof(meta)));

    const std::size_t frameSize = sizeof(link::DeviceMetadata) + kFrameHeaderSize;
    EXPECT_LT(pipe.written.size(), frameSize);

    // Release the budget; the next send drains the old backlog first, then
    // appends + writes its own frame.
    pipe.writePendingAfter = 0;
    link::DevicePose pose;
    pose.deviceId = 3;
    ASSERT_TRUE(channel.send(link::MessageType::DevicePose,
                             reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose)));
    EXPECT_EQ(pipe.written.size(), frameSize + sizeof(link::DevicePose) + kFrameHeaderSize);
}

TEST(LinkMessageChannel, SendReturnsFalseWhenOutboundBufferIsFull)
{
    link_test::FakePipe pipe;
    pipe.writeForce = link::IoStatus::Pending;  // accept nothing
    link::MessageChannel channel(pipe);

    link::DevicePose pose;
    // Fill the outbound buffer to the cap. Each frame is header + payload;
    // once pending > kMaxPayloadBytes - frameSize, send refuses.
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
    link::MessageChannel channel(pipe);

    link::DeviceMetadata meta;
    meta.deviceId = 5;
    const auto known1 = frame(link::MessageType::DeviceMetadata,
                              reinterpret_cast<const std::uint8_t*>(&meta), sizeof(meta));
    const auto unknown = frame(static_cast<link::MessageType>(999),
                               std::vector<std::uint8_t>(4, 0xAB).data(), 4);
    link::DevicePose pose;
    pose.deviceId = 6;
    const auto known2 = frame(link::MessageType::DevicePose,
                              reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose));

    std::vector<std::uint8_t> all;
    all.insert(all.end(), known1.begin(), known1.end());
    all.insert(all.end(), unknown.begin(), unknown.end());
    all.insert(all.end(), known2.begin(), known2.end());
    pipe.feedRead(all);

    std::vector<link::Message> messages;
    EXPECT_EQ(channel.receive(messages), 2u);  // unknown skipped
    ASSERT_EQ(messages.size(), 2u);
    EXPECT_EQ(messages[0].type, link::MessageType::DeviceMetadata);
    EXPECT_EQ(messages[1].type, link::MessageType::DevicePose);
}

// ---- fatal conditions ------------------------------------------------------

TEST(LinkMessageChannel, OversizeLengthFailsTheChannel)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(pipe);

    // A header claiming a payload larger than kMaxPayloadBytes (8-byte POD: 4
    // length + 2 type + 2 pad).
    std::vector<std::uint8_t> header(8);
    const std::uint32_t tooLong = link::kMaxPayloadBytes + 1;
    header[0] = static_cast<std::uint8_t>(tooLong & 0xFF);
    header[1] = static_cast<std::uint8_t>((tooLong >> 8) & 0xFF);
    header[2] = static_cast<std::uint8_t>((tooLong >> 16) & 0xFF);
    header[3] = static_cast<std::uint8_t>((tooLong >> 24) & 0xFF);
    header[4] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(link::MessageType::DevicePose) & 0xFF);
    header[5] = 0;
    pipe.feedRead(header);

    std::vector<link::Message> messages;
    channel.receive(messages);
    EXPECT_EQ(channel.state(), link::MessageChannel::State::Failed);
    EXPECT_FALSE(channel.lastError().empty());

    // Even good bytes after the failure are ignored.
    link::DevicePose pose;
    pipe.feedRead(frame(link::MessageType::DevicePose,
                        reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose)));
    EXPECT_EQ(channel.receive(messages), 0u);
}

TEST(LinkMessageChannel, PeerClosedStopsReceive)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(pipe);
    pipe.readForce = link::IoStatus::Closed;

    std::vector<link::Message> messages;
    channel.receive(messages);
    EXPECT_EQ(channel.state(), link::MessageChannel::State::Closed);
}

TEST(LinkMessageChannel, PipeErrorFailsTheChannel)
{
    link_test::FakePipe pipe;
    pipe.readForce = link::IoStatus::Failed;
    pipe.forcedError = "broken pipe";
    link::MessageChannel channel(pipe);

    std::vector<link::Message> messages;
    channel.receive(messages);
    EXPECT_EQ(channel.state(), link::MessageChannel::State::Failed);
    EXPECT_EQ(channel.lastError(), "broken pipe");
}

TEST(LinkMessageChannel, RefusesToSendOversizePayload)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(pipe);
    std::vector<std::uint8_t> huge(link::kMaxPayloadBytes + 1, 0);
    EXPECT_FALSE(channel.send(link::MessageType::DevicePose, huge.data(), huge.size()));
    EXPECT_EQ(channel.state(), link::MessageChannel::State::Open);
}

TEST(LinkMessageChannel, RefusesToSendWhenNotOpen)
{
    link_test::FakePipe pipe;
    pipe.readForce = link::IoStatus::Closed;
    link::MessageChannel channel(pipe);

    std::vector<link::Message> messages;
    channel.receive(messages);  // drives Closed
    ASSERT_EQ(channel.state(), link::MessageChannel::State::Closed);

    link::DevicePose pose;
    EXPECT_FALSE(channel.send(link::MessageType::DevicePose,
                              reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose)));
}
} // namespace