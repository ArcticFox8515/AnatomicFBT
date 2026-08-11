#include <gtest/gtest.h>

#include <glm/gtc/epsilon.hpp>
#include <glm/gtc/quaternion.hpp>

#include <filesystem>
#include <fstream>

#include "model/Error.h"
#include "model/IkRig.h"
#include "model/IkRigConfig.h"
#include "model/ModeController.h"
#include "model/Recording.h"
#include "model/ReplaySession.h"
#include "model/Skeleton.h"

namespace
{
IkRig makeRig()
{
    IkRig rig(Skeleton::makeDefault());
    rig.loadConfig(IkRigConfig::makeDefault());
    return rig;
}

// One device exactly on each target's rest world position (identity
// rotation) — a perfect T-pose, so calibration offsets are identity and
// capture goals equal the raw device poses.
std::vector<TrackedDevice> tPoseDevices(const IkRig& rig)
{
    const WorldTransforms rest = computeWorldTransforms(rig.skeleton);
    std::vector<TrackedDevice> devices;
    for (size_t i = 0; i < rig.targets.size(); ++i)
        devices.push_back({static_cast<int>(100 + i), TrackedDeviceKind::Tracker,
                           {rest.positions[rig.targets[i].jointIndex],
                            glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}});
    return devices;
}

std::vector<TrackedDevice> movedDevices(std::vector<TrackedDevice> devices, const glm::vec3& delta)
{
    for (TrackedDevice& device : devices)
        device.pose.position += delta;
    return devices;
}

// Temp directory per test; recordings are written with the real writer, so
// scan/load run against exactly what a live session produces.
class ReplaySessionTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        dir_ = std::filesystem::temp_directory_path() / "TrackingCorrectorReplaySessionTest";
        std::filesystem::remove_all(dir_);
        std::filesystem::create_directories(dir_);
    }

    void TearDown() override { std::filesystem::remove_all(dir_); }

    void writeRecording(const std::string& name,
                        const std::vector<std::vector<TrackedDevice>>& frames)
    {
        std::ofstream file(dir_ / name, std::ios::binary | std::ios::trunc);
        RecordingWriter writer(file);
        for (size_t i = 0; i < frames.size(); ++i)
            writer.writeFrame(static_cast<float>(i) * 0.5f, frames[i], {});
    }

    void writeGarbage(const std::string& name)
    {
        std::ofstream file(dir_ / name, std::ios::binary | std::ios::trunc);
        file << "certainly not a recording";
    }

    std::filesystem::path dir_;
};
} // namespace

TEST_F(ReplaySessionTest, ScanFindsRecordingsSortedByName)
{
    IkRig rig = makeRig();
    writeRecording("b.tcrec", {tPoseDevices(rig)});
    writeRecording("a.tcrec", {tPoseDevices(rig)});
    writeGarbage("notes.txt");

    ReplaySession session;
    session.scan(dir_);

    ASSERT_EQ(session.files().size(), 2u);
    EXPECT_EQ(session.files()[0], "a.tcrec");
    EXPECT_EQ(session.files()[1], "b.tcrec");
    EXPECT_FALSE(session.hasRecording());  // scanning does not load
    EXPECT_EQ(session.loadedIndex(), -1);
}

TEST_F(ReplaySessionTest, ScanMissingDirectoryThrowsAndLeavesEmptyList)
{
    ReplaySession session;
    EXPECT_THROW(session.scan(dir_ / "does-not-exist"), Error);
    EXPECT_TRUE(session.files().empty());
}

TEST_F(ReplaySessionTest, LoadRecalibratesFromFrameZeroAndReproducesLiveGoals)
{
    // Live session: T-pose calibration, then one moved capture frame.
    IkRig liveRig = makeRig();
    ModeController live(Mode::Calibration);
    const std::vector<TrackedDevice> tPose = tPoseDevices(liveRig);
    const std::vector<TrackedDevice> moved = movedDevices(tPose, {0.1f, -0.2f, 0.05f});
    const FramePlan liveFrame0 = live.update(liveRig, tPose, true);
    const FramePlan liveFrame1 = live.update(liveRig, moved, false);
    writeRecording("session.tcrec", {tPose, moved});

    IkRig replayRig = makeRig();
    ModeController controller;
    controller.switchToReplay();
    ReplaySession session;
    session.scan(dir_);
    session.load(0, controller, replayRig);

    EXPECT_EQ(session.loadedIndex(), 0);
    ASSERT_TRUE(session.hasRecording());
    EXPECT_EQ(session.frameIndex(), 0u);

    // Frame 0 devices drive the solver to the live transition-frame goals...
    FramePlan plan = controller.update(replayRig, session.currentDevices(), false);
    ASSERT_EQ(plan.solve, SolveMode::Goals);
    ASSERT_EQ(plan.goals.size(), liveFrame0.goals.size());
    for (size_t i = 0; i < plan.goals.size(); ++i)
        EXPECT_TRUE(glm::all(glm::epsilonEqual(plan.goals[i].position,
                                               liveFrame0.goals[i].position, 1e-5f)));

    // ...and seeking to the second frame reproduces the live capture frame.
    session.seek(session.recording().duration());
    EXPECT_EQ(session.frameIndex(), 1u);
    plan = controller.update(replayRig, session.currentDevices(), false);
    ASSERT_EQ(plan.goals.size(), liveFrame1.goals.size());
    for (size_t i = 0; i < plan.goals.size(); ++i)
        EXPECT_TRUE(glm::all(glm::epsilonEqual(plan.goals[i].position,
                                               liveFrame1.goals[i].position, 1e-5f)));
}

TEST_F(ReplaySessionTest, LoadCorruptFileThrowsAndKeepsTheFileList)
{
    writeGarbage("bad.tcrec");
    IkRig rig = makeRig();
    ModeController controller;
    ReplaySession session;
    session.scan(dir_);
    ASSERT_EQ(session.files().size(), 1u);

    EXPECT_THROW(session.load(0, controller, rig), Error);

    EXPECT_FALSE(session.hasRecording());
    EXPECT_EQ(session.loadedIndex(), -1);
    EXPECT_EQ(session.files().size(), 1u);  // the list survives for the UI
    EXPECT_TRUE(session.currentDevices().empty());
}

TEST_F(ReplaySessionTest, LoadOutOfRangeThrows)
{
    IkRig rig = makeRig();
    ModeController controller;
    ReplaySession session;
    session.scan(dir_);
    EXPECT_THROW(session.load(0, controller, rig), Error);
}

TEST_F(ReplaySessionTest, SeekSnapsToNearestFrameTime)
{
    IkRig rig = makeRig();
    const std::vector<TrackedDevice> tPose = tPoseDevices(rig);
    writeRecording("session.tcrec", {tPose, tPose, tPose});  // times 0, 0.5, 1.0
    ModeController controller;
    ReplaySession session;
    session.scan(dir_);
    session.load(0, controller, rig);

    session.seek(0.2f);
    EXPECT_EQ(session.frameIndex(), 0u);
    EXPECT_EQ(session.frameTime(), 0.0f);
    session.seek(0.4f);
    EXPECT_EQ(session.frameIndex(), 1u);
    EXPECT_EQ(session.frameTime(), 0.5f);
    session.seek(99.0f);
    EXPECT_EQ(session.frameIndex(), 2u);
    EXPECT_EQ(session.frameTime(), 1.0f);
}

TEST_F(ReplaySessionTest, WithoutARecordingSeekAndDevicesAreInert)
{
    ReplaySession session;
    session.seek(1.0f);  // no-op, must not throw
    EXPECT_EQ(session.frameIndex(), 0u);
    EXPECT_EQ(session.frameTime(), 0.0f);
    EXPECT_TRUE(session.currentDevices().empty());
}

TEST_F(ReplaySessionTest, ResetDropsEverything)
{
    IkRig rig = makeRig();
    writeRecording("session.tcrec", {tPoseDevices(rig)});
    ModeController controller;
    ReplaySession session;
    session.scan(dir_);
    session.load(0, controller, rig);
    ASSERT_TRUE(session.hasRecording());

    session.reset();

    EXPECT_TRUE(session.files().empty());
    EXPECT_FALSE(session.hasRecording());
    EXPECT_EQ(session.loadedIndex(), -1);
    EXPECT_EQ(session.frameIndex(), 0u);
}
