#include <gtest/gtest.h>

#include <glm/gtc/quaternion.hpp>

#include "model/TrackedDevice.h"

namespace
{
TrackedDevice makeDevice(int id, TrackedDeviceKind kind, glm::vec3 position = glm::vec3(0.0f))
{
    return {id, kind, {position, glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}};
}
} // namespace

TEST(FindHmd, ReturnsNullptrWithoutHmd)
{
    EXPECT_EQ(findHmd({}), nullptr);

    const std::vector<TrackedDevice> devices = {
        makeDevice(1, TrackedDeviceKind::Controller),
        makeDevice(2, TrackedDeviceKind::Tracker)};
    EXPECT_EQ(findHmd(devices), nullptr);
}

TEST(FindHmd, ReturnsFirstHmd)
{
    const std::vector<TrackedDevice> devices = {
        makeDevice(1, TrackedDeviceKind::Controller),
        makeDevice(2, TrackedDeviceKind::Hmd, {1.0f, 1.7f, 0.0f}),
        makeDevice(3, TrackedDeviceKind::Hmd, {0.0f, 1.8f, 0.0f})};

    const TrackedDevice* hmd = findHmd(devices);

    ASSERT_NE(hmd, nullptr);
    EXPECT_EQ(hmd->id, 2);
}

TEST(DeviceKindName, LabelsAllKinds)
{
    EXPECT_STREQ(deviceKindName(TrackedDeviceKind::Hmd), "HMD");
    EXPECT_STREQ(deviceKindName(TrackedDeviceKind::Controller), "controller");
    EXPECT_STREQ(deviceKindName(TrackedDeviceKind::Tracker), "tracker");
    EXPECT_STREQ(deviceKindName(TrackedDeviceKind::Other), "device");
}

TEST(DevicePosePairs, PreservesIdsAndPosesInOrder)
{
    const glm::quat rot = glm::angleAxis(0.5f, glm::vec3(0.0f, 1.0f, 0.0f));
    const std::vector<TrackedDevice> devices = {
        {7, TrackedDeviceKind::Hmd, {{0.1f, 1.7f, 0.0f}, rot}},
        {3, TrackedDeviceKind::Tracker, {{0.5f, 0.9f, 0.2f}, glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}}};

    const std::vector<std::pair<int, Pose>> pairs = devicePosePairs(devices);

    ASSERT_EQ(pairs.size(), 2u);
    EXPECT_EQ(pairs[0].first, 7);
    EXPECT_EQ(pairs[0].second.position, glm::vec3(0.1f, 1.7f, 0.0f));
    EXPECT_EQ(pairs[0].second.rotation, rot);
    EXPECT_EQ(pairs[1].first, 3);
    EXPECT_EQ(pairs[1].second.position, glm::vec3(0.5f, 0.9f, 0.2f));
}
