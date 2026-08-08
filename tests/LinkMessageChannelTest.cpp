// Framing + reassembly tests for the driver link (doc/driver-plan.md phase A, step 5).
//
// The framing layer (MessageChannel) sits on a `Pipe` and splits the byte
// stream into length-prefixed frames. These tests drive it through the cases
// the real overlapped pipe will produce — split reads, several frames per
// read, a header split mid-field, partial writes, pending-then-complete on
// both directions — plus the pipe-lifecycle cases (factory returns null, dead
// pipe dropped, reconnect, no snapshot). The mock `Pipe` scripts the bytes;
// nothing touches Win32.

#include "FakePipe.h"
#include "link/Log.h"
#include "link/MessageChannel.h"
#include "link/Pipe.h"
#include "link/Protocol.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace
{
inline constexpr std::size_t kFrameHeaderSize = 8;

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
    out.push_back(0);
    out.push_back(0);
    out.insert(out.end(), payload, payload + size);
    return out;
}

std::vector<std::uint8_t> frame(link::MessageType type, const std::vector<std::uint8_t>& payload)
{
    return frame(type, payload.data(), payload.size());
}

link::Message message(const link::DevicePose& pose)
{
    link::Message m;
    m.size = sizeof(link::DevicePose);
    m.type = link::MessageType::DevicePose;
    m.pose = pose;
    return m;
}

link::Logger& testLogger()
{
    static link::Logger logger;
    return logger;
}

static_assert(sizeof(link::DevicePose) == 68, "pose POD size drifted");

// ---- classifyIo ------------------------------------------------------------

TEST(LinkClassifyIo, NonzeroReturnIsOk)
{
    EXPECT_EQ(link::classifyIo(1, 0), link::IoStatus::Ok);
}

TEST(LinkClassifyIo, IoPendingIsPending)
{
    EXPECT_EQ(link::classifyIo(0, link::errIoPending), link::IoStatus::Pending);
}

TEST(LinkClassifyIo, IoIncompleteIsPending)
{
    EXPECT_EQ(link::classifyIo(0, link::errIoIncomplete), link::IoStatus::Pending);
}

TEST(LinkClassifyIo, PipeConnectedIsOk)
{
    EXPECT_EQ(link::classifyIo(0, link::errPipeConnected), link::IoStatus::Ok);
}

TEST(LinkClassifyIo, BrokenPipeIsClosed)
{
    EXPECT_EQ(link::classifyIo(0, link::errBrokenPipe), link::IoStatus::Closed);
}

TEST(LinkClassifyIo, PipeNotConnectedIsClosed)
{
    EXPECT_EQ(link::classifyIo(0, link::errPipeNotConnected), link::IoStatus::Closed);
}

TEST(LinkClassifyIo, NoDataIsClosed)
{
    EXPECT_EQ(link::classifyIo(0, link::errNoData), link::IoStatus::Closed);
}

TEST(LinkClassifyIo, OperationAbortedIsClosed)
{
    EXPECT_EQ(link::classifyIo(0, link::errOperationAborted), link::IoStatus::Closed);
}

TEST(LinkClassifyIo, UnknownErrorIsFailed)
{
    EXPECT_EQ(link::classifyIo(0, 5), link::IoStatus::Failed);
}

// ---- round trip through the mock pipe --------------------------------------

TEST(LinkMessageChannel, SendsAndReceivesADevicePose)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(pipe));
    std::vector<link::Message> dummy;
    channel.receive(dummy);

    link::DevicePose pose;
    pose.deviceId = 11;
    pose.tracking = link::TrackingState::Tracking;
    pose.deviceKind = link::DeviceKind::Controller;
    pose.position[0] = 0.5f;
    pose.rotation[3] = 1.0f;
    channel.send(message(pose));

    pipe.feedRead(pipe.written);

    std::vector<link::Message> messages;
    EXPECT_EQ(channel.receive(messages), 1u);
    ASSERT_EQ(messages.size(), 1u);
    EXPECT_EQ(messages[0].type, link::MessageType::DevicePose);
    ASSERT_EQ(messages[0].size, sizeof(link::DevicePose));
    EXPECT_EQ(messages[0].pose.deviceId, 11u);
    EXPECT_EQ(messages[0].pose.tracking, link::TrackingState::Tracking);
    EXPECT_EQ(messages[0].pose.deviceKind, link::DeviceKind::Controller);
    EXPECT_FLOAT_EQ(messages[0].pose.position[0], 0.5f);
}

// ---- reassembly across split reads ----------------------------------------

TEST(LinkMessageChannel, ReassemblesAFrameSplitAcrossThreeReads)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(pipe));

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
    ASSERT_EQ(messages[0].size, sizeof(link::DevicePose));
}

TEST(LinkMessageChannel, ReassemblesTwoFramesDeliveredInOneRead)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(pipe));

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
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(pipe));

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

// ---- pending read completes on a later poll --------------------------------

TEST(LinkMessageChannel, PendingReadCompletesOnNextReceive)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(pipe));
    std::vector<link::Message> dummy;
    channel.receive(dummy);

    link::DevicePose pose;
    pose.deviceId = 7;
    const auto wire = frame(link::MessageType::DevicePose,
                            reinterpret_cast<const std::uint8_t*>(&pose), sizeof(pose));

    std::vector<link::Message> messages;
    EXPECT_EQ(channel.receive(messages), 0u);

    pipe.feedRead(wire);
    EXPECT_EQ(channel.receive(messages), 1u);
    ASSERT_EQ(messages.size(), 1u);
    ASSERT_EQ(messages[0].size, sizeof(link::DevicePose));
    EXPECT_EQ(messages[0].pose.deviceId, 7u);
}

// ---- partial / pending writes ----------------------------------------------

TEST(LinkMessageChannel, RetainsBacklogWhenPipeIsPendingAndDrainsOnNextSend)
{
    link_test::FakePipe pipe;
    pipe.writePendingAfter = 4;
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(pipe));
    std::vector<link::Message> dummy;
    channel.receive(dummy);

    link::DevicePose pose;
    pose.deviceId = 2;
    channel.send(message(pose));

    const std::size_t frameSize = sizeof(link::DevicePose) + kFrameHeaderSize;
    EXPECT_LT(pipe.written.size(), frameSize);

    pipe.writePendingAfter = 0;
    link::DevicePose pose2;
    pose2.deviceId = 3;
    channel.send(message(pose2));
    EXPECT_EQ(pipe.written.size(), frameSize + sizeof(link::DevicePose) + kFrameHeaderSize);
}

TEST(LinkMessageChannel, PendingWriteCompletesOnNextReceive)
{
    link_test::FakePipe pipe;
    pipe.writeForceErr = link::errIoPending;
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(pipe));
    std::vector<link::Message> dummy;
    channel.receive(dummy);

    link::DevicePose pose;
    pose.deviceId = 42;
    channel.send(message(pose));
    ASSERT_TRUE(pipe.written.empty());

    pipe.writeForceErr = 0;
    std::vector<link::Message> messages;
    channel.receive(messages);
    ASSERT_EQ(pipe.written.size(), kFrameHeaderSize + sizeof(link::DevicePose));

    pipe.feedRead(pipe.written);
    messages.clear();
    EXPECT_EQ(channel.receive(messages), 1u);
    ASSERT_EQ(messages.size(), 1u);
    EXPECT_EQ(messages[0].pose.deviceId, 42u);
}

// ---- unknown type skipped between known ones ------------------------------

TEST(LinkMessageChannel, SkipsAnUnknownMessageTypeAndContinues)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(pipe));

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
    EXPECT_EQ(channel.receive(messages), 2u);
    ASSERT_EQ(messages.size(), 2u);
    EXPECT_EQ(messages[0].type, link::MessageType::DevicePose);
    EXPECT_EQ(messages[1].type, link::MessageType::DevicePose);
}

TEST(LinkMessageChannel, TheOldDeviceMetadataTypeIsSkippedAsUnknown)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(pipe));

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
    link::MessageChannel channel(testLogger(), factory);

    std::vector<link::Message> messages;
    EXPECT_EQ(channel.receive(messages), 0u);

    link::DevicePose pose;
    channel.send(message(pose));
}

// ---- dead pipe dropped, channel resets for next client --------------------

TEST(LinkMessageChannel, DropsAPipeThatThePeerClosedAndResetsForNextClient)
{
    link_test::FakePipe fake;
    fake.readEmptyErr = link::errBrokenPipe;
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(fake));

    std::vector<link::Message> messages;
    channel.receive(messages);

    link::DevicePose pose;
    channel.send(message(pose));
}

TEST(LinkMessageChannel, DropsAPipeThatFailedAndPreservesLastError)
{
    link_test::FakePipe fake;
    fake.readEmptyErr = 5;
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(fake));

    std::vector<link::Message> messages;
    channel.receive(messages);

    EXPECT_EQ(channel.lastError(), std::to_string(5u));
}

TEST(LinkMessageChannel, OversizeLengthDropsThePipe)
{
    link_test::FakePipe fake;
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(fake));

    std::vector<std::uint8_t> header(8);
    const std::uint32_t tooLong = sizeof(link::DevicePose) + 1;
    header[0] = static_cast<std::uint8_t>(tooLong & 0xFF);
    header[1] = static_cast<std::uint8_t>((tooLong >> 8) & 0xFF);
    header[2] = static_cast<std::uint8_t>((tooLong >> 16) & 0xFF);
    header[3] = static_cast<std::uint8_t>((tooLong >> 24) & 0xFF);
    header[4] = static_cast<std::uint8_t>(static_cast<std::uint16_t>(link::MessageType::DevicePose) & 0xFF);
    header[5] = 0;
    fake.feedRead(header);

    std::vector<link::Message> messages;
    channel.receive(messages);

    EXPECT_FALSE(channel.lastError().empty());
}

// ---- reconnect after a drop -----------------------------------------------

TEST(LinkMessageChannel, ReconnectsAfterADrop)
{
    link_test::FakePipe fake;
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(fake));

    std::vector<link::Message> messages;
    channel.receive(messages);

    link::DevicePose pose;
    pose.deviceId = 1;
    channel.send(message(pose));
    fake.feedRead(fake.written);
    messages.clear();
    channel.receive(messages);
    ASSERT_EQ(messages.size(), 1u);

    fake.readEmptyErr = link::errBrokenPipe;
    fake.readQueue.clear();
    messages.clear();
    channel.receive(messages);

    fake.readEmptyErr = link::errIoPending;
    link::DevicePose pose2;
    pose2.deviceId = 2;
    fake.feedRead(frame(link::MessageType::DevicePose,
                        reinterpret_cast<const std::uint8_t*>(&pose2), sizeof(pose2)));
    messages.clear();
    channel.receive(messages);
    ASSERT_EQ(messages.size(), 1u);
    EXPECT_EQ(messages[0].pose.deviceId, 2u);
}

TEST(LinkMessageChannel, ClearsBacklogOnReconnectSoNoSnapshotIsDelivered)
{
    link_test::FakePipe fake;
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(fake));

    std::vector<link::Message> messages;
    channel.receive(messages);

    fake.writeForceErr = link::errIoPending;
    link::DevicePose pose;
    pose.deviceId = 99;
    channel.send(message(pose));
    ASSERT_TRUE(fake.written.empty());

    fake.readEmptyErr = link::errBrokenPipe;
    channel.receive(messages);

    fake.readEmptyErr = link::errIoPending;
    fake.writeForceErr = 0;
    channel.receive(messages);

    link::DevicePose pose2;
    pose2.deviceId = 100;
    channel.send(message(pose2));
    const std::size_t oneFrame = kFrameHeaderSize + sizeof(link::DevicePose);
    EXPECT_EQ(fake.written.size(), oneFrame);
}

// ---- overlapped buffer sizing ------------------------------------------------

TEST(LinkMessageChannel, OverlappedBuffersHaveExpectedSize)
{
    link_test::FakePipe pipe;
    pipe.overlappedSizeResult = 24;
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(pipe));
    std::vector<link::Message> dummy;
    channel.receive(dummy);

    ASSERT_EQ(pipe.createEventOverlappedSizes.size(), 3u);
    EXPECT_EQ(pipe.createEventOverlappedSizes[0], 24u);
    EXPECT_EQ(pipe.createEventOverlappedSizes[1], 24u);
    EXPECT_EQ(pipe.createEventOverlappedSizes[2], 24u);
}

// ---- handle is passed correctly to all methods ------------------------------

TEST(LinkMessageChannel, PassesCorrectHandleToAllPipeMethods)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(pipe));
    std::vector<link::Message> dummy;
    channel.receive(dummy);

    ASSERT_NE(pipe.createdHandle, nullptr);
    for (void* h : pipe.handlesPassedToConnect)
        EXPECT_EQ(h, pipe.createdHandle);

    link::DevicePose pose;
    pose.deviceId = 1;
    channel.send(message(pose));
    for (void* h : pipe.handlesPassedToWrite)
        EXPECT_EQ(h, pipe.createdHandle);

    pipe.feedRead(pipe.written);
    std::vector<link::Message> messages;
    channel.receive(messages);
    for (void* h : pipe.handlesPassedToRead)
        EXPECT_EQ(h, pipe.createdHandle);
}

// ---- full lifecycle: create, connect, IO, close on drop ---------------------

TEST(LinkMessageChannel, FullLifecycleCreatesAndClosesEverything)
{
    link_test::FakePipe pipe;
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(pipe));

    std::vector<link::Message> messages;
    channel.receive(messages);

    EXPECT_EQ(pipe.createEventPtrs.size(), 3u);
    EXPECT_EQ(pipe.createPipeCallCount, 1);
    EXPECT_EQ(pipe.createPipeInBufferSize, link::pipeBufferSize);
    EXPECT_EQ(pipe.createPipeOutBufferSize, link::pipeBufferSize);

    pipe.readEmptyErr = link::errBrokenPipe;
    pipe.readQueue.clear();
    messages.clear();
    channel.receive(messages);

    EXPECT_EQ(pipe.closeEventPtrs.size(), 3u);
    EXPECT_TRUE(pipe.closeCalled);
    EXPECT_EQ(pipe.closeHandle, pipe.createdHandle);
}

// ---- third event creation fails: first two de-allocated, third untouched ----

TEST(LinkMessageChannel, DropsPipeWhenThirdEventCreationFails)
{
    link_test::FakePipe fake;
    fake.createEventResults[2] = 0;
    fake.forcedErr = 8;
    link::MessageChannel channel(testLogger(), link_test::borrowPipeFactory(fake));

    std::vector<link::Message> messages;
    channel.receive(messages);

    ASSERT_EQ(fake.createEventPtrs.size(), 3u);
    ASSERT_EQ(fake.closeEventPtrs.size(), 2u);

    EXPECT_NE(fake.closeEventPtrs[0], fake.createEventPtrs[2]);
    EXPECT_NE(fake.closeEventPtrs[1], fake.createEventPtrs[2]);

    EXPECT_FALSE(channel.lastError().empty());

    fake.createEventResults[2] = 1;
    fake.createEventCallCount = 0;
    fake.readEmptyErr = link::errIoPending;
    channel.receive(messages);

    ASSERT_EQ(fake.createEventPtrs.size(), 6u);

    link::DevicePose pose;
    channel.send(message(pose));
    const std::size_t oneFrame = kFrameHeaderSize + sizeof(link::DevicePose);
    EXPECT_EQ(fake.written.size(), oneFrame);
}
} // namespace
