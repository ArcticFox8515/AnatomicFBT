#include <gtest/gtest.h>

#include <memory>
#include <sstream>

#include "model/Recording.h"
#include "model/SessionRecorder.h"

namespace
{
std::vector<TrackedDevice> someDevices()
{
    return {
        {0, TrackedDeviceKind::Hmd, {{0.0f, 1.7f, 0.0f}, glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}},
        {5, TrackedDeviceKind::Tracker, {{0.1f, 0.9f, 0.0f}, glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}},
    };
}

std::vector<TrackedDevice> movedDevices(std::vector<TrackedDevice> devices, float dy)
{
    for (TrackedDevice& device : devices)
        device.pose.position.y += dy;
    return devices;
}

// Stream factory that hands out shared string streams and remembers them so
// tests can inspect (and load) the written bytes afterwards.
struct StreamLog
{
    std::vector<std::shared_ptr<std::stringstream>> streams;

    SessionRecorder::StreamFactory factory()
    {
        return [this]() -> std::shared_ptr<std::ostream> {
            streams.push_back(std::make_shared<std::stringstream>(
                std::ios::in | std::ios::out | std::ios::binary));
            return streams.back();
        };
    }
};
} // namespace

TEST(SessionRecorder, IdlesOutsideCapture)
{
    StreamLog log;
    SessionRecorder recorder(log.factory());

    for (const Mode mode : {Mode::ManualPose, Mode::Calibration, Mode::Replay})
    {
        const SessionRecorder::Event event = recorder.update(mode, false, 1.0, someDevices(), {});
        EXPECT_FALSE(event.started);
        EXPECT_FALSE(event.stopped);
        EXPECT_TRUE(event.error.empty());
    }
    EXPECT_FALSE(recorder.isRecording());
    EXPECT_TRUE(log.streams.empty());  // no stream was ever opened
}

TEST(SessionRecorder, RecordsSessionWithCalibrationFrameAtTimeZero)
{
    StreamLog log;
    SessionRecorder recorder(log.factory());
    const std::vector<TrackedDevice> frame0 = someDevices();
    const std::vector<TrackedDevice> frame1 = movedDevices(frame0, 0.1f);

    // Calibration -> capture transition at absolute clock 100s.
    SessionRecorder::Event event = recorder.update(Mode::Capture, true, 100.0, frame0, {});
    EXPECT_TRUE(event.started);
    EXPECT_TRUE(recorder.isRecording());

    event = recorder.update(Mode::Capture, false, 100.5, frame1, {});
    EXPECT_FALSE(event.started);
    EXPECT_FALSE(event.stopped);

    // Leaving capture stops the recording; the stop frame is not recorded.
    event = recorder.update(Mode::ManualPose, false, 101.0, {}, {});
    EXPECT_TRUE(event.stopped);
    EXPECT_FALSE(recorder.isRecording());

    ASSERT_EQ(log.streams.size(), 1u);
    const Recording recording = loadRecording(*log.streams[0]);
    ASSERT_EQ(recording.frames.size(), 2u);
    EXPECT_EQ(recording.frames[0].time, 0.0f);   // session time, not wall clock
    EXPECT_EQ(recording.frames[1].time, 0.5f);
    ASSERT_EQ(recording.frames[0].devices.size(), frame0.size());
    EXPECT_EQ(recording.frames[0].devices[1].pose.position.y, frame0[1].pose.position.y);
    EXPECT_EQ(recording.frames[1].devices[1].pose.position.y, frame1[1].pose.position.y);
}

TEST(SessionRecorder, EachSessionGetsAFreshStream)
{
    StreamLog log;
    SessionRecorder recorder(log.factory());
    const std::vector<TrackedDevice> devices = someDevices();

    recorder.update(Mode::Capture, true, 10.0, devices, {});
    recorder.update(Mode::Calibration, false, 11.0, devices, {});  // stop (recalibrating)
    recorder.update(Mode::Capture, true, 12.0, devices, {});
    recorder.update(Mode::Capture, false, 12.5, devices, {});
    recorder.update(Mode::ManualPose, false, 13.0, devices, {});   // stop

    ASSERT_EQ(log.streams.size(), 2u);
    EXPECT_EQ(loadRecording(*log.streams[0]).frames.size(), 1u);
    EXPECT_EQ(loadRecording(*log.streams[1]).frames.size(), 2u);
}

TEST(SessionRecorder, FailedStreamOpenReportsErrorAndDoesNotRecord)
{
    SessionRecorder recorder([]() -> std::shared_ptr<std::ostream> { return nullptr; });

    const SessionRecorder::Event event = recorder.update(Mode::Capture, true, 0.0, someDevices(), {});

    EXPECT_FALSE(event.started);
    EXPECT_FALSE(event.error.empty());
    EXPECT_FALSE(recorder.isRecording());

    // Subsequent capture frames stay silent — the failure was reported once.
    const SessionRecorder::Event next = recorder.update(Mode::Capture, false, 0.5, someDevices(), {});
    EXPECT_TRUE(next.error.empty());
    EXPECT_FALSE(next.started);
    EXPECT_FALSE(next.stopped);
}

TEST(SessionRecorder, ThrowingFactoryReportsErrorAndDoesNotRecord)
{
    SessionRecorder recorder([]() -> std::shared_ptr<std::ostream> {
        throw std::runtime_error("disk on fire");
    });

    const SessionRecorder::Event event = recorder.update(Mode::Capture, true, 0.0, someDevices(), {});

    EXPECT_FALSE(event.started);
    EXPECT_NE(event.error.find("disk on fire"), std::string::npos);
    EXPECT_FALSE(recorder.isRecording());
}

TEST(SessionRecorder, MidSessionWriteFailureStopsRecordingOnce)
{
    StreamLog log;
    SessionRecorder recorder(log.factory());
    const std::vector<TrackedDevice> devices = someDevices();

    EXPECT_TRUE(recorder.update(Mode::Capture, true, 0.0, devices, {}).started);
    log.streams[0]->setstate(std::ios::badbit);  // the disk fills up

    const SessionRecorder::Event failure = recorder.update(Mode::Capture, false, 0.5, devices, {});
    EXPECT_FALSE(failure.error.empty());
    EXPECT_FALSE(recorder.isRecording());

    // Capture continues; the recorder stays silent and opens nothing new.
    const SessionRecorder::Event next = recorder.update(Mode::Capture, false, 1.0, devices, {});
    EXPECT_TRUE(next.error.empty());
    EXPECT_FALSE(next.stopped);
    EXPECT_EQ(log.streams.size(), 1u);
}
