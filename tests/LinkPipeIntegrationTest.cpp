// One real-pipe integration test for the driver link (doc/messagechannel-rework-plan.md
// §8). A server channel and a client channel run in one process on a private pipe
// name and are pumped alternately from the test thread within a bounded iteration
// budget: the server sends a DevicePose the client receives intact, and the client
// sends one the server receives intact.

#include "link/Log.h"
#include "link/MessageChannel.h"
#include "link/Pipe.h"
#include "link/Protocol.h"
#include "pipe/Win32Pipe.h"

#include <gtest/gtest.h>

#include <memory>
#include <random>
#include <string>
#include <vector>

namespace
{
std::string privatePipeName()
{
    std::random_device rd;
    return std::string(link::kDriverPipeName) + "-" + std::to_string(rd());
}

link::Message poseMessage(std::uint32_t id)
{
    link::DevicePose pose;
    pose.deviceId = id;
    pose.tracking = link::TrackingState::Tracking;
    pose.deviceKind = link::DeviceKind::Tracker;
    pose.position.x = static_cast<float>(id);
    pose.rotation.w = 1.0f;
    link::Message m;
    m.size = sizeof(link::DevicePose);
    m.type = link::MessageType::DevicePose;
    m.devicePose = pose;
    return m;
}

TEST(LinkPipeIntegration, RoundTripsADevicePoseInBothDirections)
{
    const std::string name = privatePipeName();
    link::Logger logger;
    link::MessageChannel server(logger, [&name] {
        return std::make_shared<link::Win32ServerPipe>(name);
    });
    link::MessageChannel client(logger, [&name] {
        return std::make_shared<link::Win32ClientPipe>(name);
    });

    for (int i = 0; i < 20; ++i)
    {
        std::vector<link::Message> dummy;
        server.receive(dummy);
        client.receive(dummy);
    }

    server.send(poseMessage(7));

    std::vector<link::Message> clientMessages;
    for (int i = 0; i < 1000 && clientMessages.empty(); ++i)
    {
        std::vector<link::Message> dummy;
        server.receive(dummy);
        client.receive(clientMessages);
    }
    ASSERT_EQ(clientMessages.size(), 1u);
    EXPECT_EQ(clientMessages[0].devicePose.deviceId, 7u);
    EXPECT_EQ(clientMessages[0].devicePose.deviceKind, link::DeviceKind::Tracker);
    EXPECT_FLOAT_EQ(clientMessages[0].devicePose.position.x, 7.0f);

    client.send(poseMessage(13));

    std::vector<link::Message> serverMessages;
    for (int i = 0; i < 1000 && serverMessages.empty(); ++i)
    {
        std::vector<link::Message> dummy;
        client.receive(dummy);
        server.receive(serverMessages);
    }
    ASSERT_EQ(serverMessages.size(), 1u);
    EXPECT_EQ(serverMessages[0].devicePose.deviceId, 13u);
}
} // namespace
