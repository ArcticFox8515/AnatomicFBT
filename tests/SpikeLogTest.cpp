// Tests for the throwaway step-1 spike's logging (doc/driver-plan.md).
//
// No filesystem, no clock, no environment variables: the spike runs inside
// vrserver.exe, so its logging must be provably harmless, and a test that depends on
// the machine it runs on proves nothing. Every decision is driven through a plain
// ostringstream — including the "stream failed to open" case, which is a stream in a
// bad state here rather than an unwritable path.

#include "spike/SpikeLog.h"

#include <gtest/gtest.h>

#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace
{
TEST(SpikeLogPaths, ProcessBaseNameStripsDirectoryAndExtension)
{
    EXPECT_EQ(spike::processBaseName("C:\\Program Files\\SteamVR\\vrserver.exe"), "vrserver");
    EXPECT_EQ(spike::processBaseName("C:/mixed/separators/vrwatchdog.exe"), "vrwatchdog");
    EXPECT_EQ(spike::processBaseName("vrserver.exe"), "vrserver");   // no separator
    EXPECT_EQ(spike::processBaseName("C:\\dir\\noextension"), "noextension");
    EXPECT_EQ(spike::processBaseName("noextension"), "noextension"); // neither
    EXPECT_EQ(spike::processBaseName(""), "unknown");
}

TEST(SpikeLogPaths, DirectoryFallsBackToTheWorkingDirectoryWithoutLocalAppData)
{
    EXPECT_EQ(spike::logDirectory("C:\\Users\\x\\AppData\\Local"),
              "C:\\Users\\x\\AppData\\Local\\TrackingCorrector\\");
    // A log in the working directory still beats no log at all.
    EXPECT_EQ(spike::logDirectory(""), "");
}

TEST(SpikeLogPaths, PathCarriesThePrefixAndTheLoadingProcess)
{
    // The process name is in the file name because SteamVR loads the same DLL into
    // vrserver.exe and vrwatchdog.exe, and both would otherwise fight over one file.
    const spike::LogEnvironment environment{"C:\\local", "C:\\SteamVR\\vrserver.exe"};
    EXPECT_EQ(spike::logPath(environment, "driver-spike"),
              "C:\\local\\TrackingCorrector\\driver-spike-vrserver.log");
    EXPECT_EQ(spike::logPath({"", ""}, "client-spike"), "client-spike-unknown.log");
}

TEST(SpikeLogPaths, TimestampIsZeroPaddedToTheMillisecond)
{
    EXPECT_EQ(spike::formatTimestamp(1, 2, 3, 4), "01:02:03.004");
    EXPECT_EQ(spike::formatTimestamp(23, 59, 59, 999), "23:59:59.999");
}

TEST(SpikeLogPaths, LineFormatIsTimestampThenMessage)
{
    EXPECT_EQ(spike::formatLogLine("01:02:03.004", "hello"), "[01:02:03.004] hello\n");
}

// ------------------------------------------------------------------- Logger ----

class SpikeLoggerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        logger_.setTimestampSource([] { return std::string("00:00:00.000"); });
        ASSERT_TRUE(logger_.setStream(stream_, "C:\\log\\driver-spike-vrserver.log"));
    }

    std::string written() const { return stream_->str(); }

    std::shared_ptr<std::ostringstream> stream_ = std::make_shared<std::ostringstream>();
    spike::Logger logger_;
};

TEST_F(SpikeLoggerTest, FormatsAndTimestampsEveryLine)
{
    logger_.logf("device %u %s", 3u, "tracker");
    EXPECT_EQ(written(), "[00:00:00.000] device 3 tracker\n");
    EXPECT_TRUE(logger_.isOpen());
    EXPECT_EQ(logger_.filePath(), "C:\\log\\driver-spike-vrserver.log");
}

TEST_F(SpikeLoggerTest, SinkSeesTheBareMessage)
{
    // The sink is IVRDriverLog in the DLL, which timestamps the lines itself.
    std::vector<std::string> lines;
    logger_.setSink([&](const char* message) { lines.emplace_back(message); });
    logger_.write("hello");
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines[0], "hello");
    EXPECT_EQ(written(), "[00:00:00.000] hello\n");
}

TEST_F(SpikeLoggerTest, FirstStreamWins)
{
    // Every entry point offers a stream (HmdDriverFactory, server Init, watchdog Init)
    // and nothing orders them: the second offer must not swap the file out from under
    // a hook thread that is writing to it.
    auto second = std::make_shared<std::ostringstream>();
    EXPECT_TRUE(logger_.setStream(second, "C:\\log\\other.log"));
    logger_.write("hello");
    EXPECT_EQ(written(), "[00:00:00.000] hello\n");
    EXPECT_EQ(second->str(), "");
    EXPECT_EQ(logger_.filePath(), "C:\\log\\driver-spike-vrserver.log");
}

TEST_F(SpikeLoggerTest, CloseFlushesAndSilencesBothOutputs)
{
    bool sinkCalled = false;
    logger_.setSink([&](const char*) { sinkCalled = true; });
    logger_.close();

    EXPECT_FALSE(logger_.isOpen());
    EXPECT_EQ(logger_.filePath(), "");
    logger_.write("after close");
    EXPECT_EQ(written(), "");
    EXPECT_FALSE(sinkCalled);
}

TEST(SpikeLogger, UnusableStreamsAreRefusedRatherThanStored)
{
    spike::Logger logger;
    EXPECT_FALSE(logger.isOpen());
    EXPECT_EQ(logger.filePath(), "");

    // Closing a logger that was never opened: Cleanup runs even when Init failed.
    logger.close();
    EXPECT_FALSE(logger.isOpen());

    // What an ofstream on an unwritable path looks like to the logger.
    auto failed = std::make_shared<std::ostringstream>();
    failed->setstate(std::ios::badbit);
    EXPECT_FALSE(logger.setStream(failed, "C:\\unwritable\\spike.log"));
    EXPECT_FALSE(logger.setStream(nullptr, "C:\\nowhere.log"));
    EXPECT_FALSE(logger.isOpen());
    EXPECT_EQ(logger.filePath(), "");

    // ...and logging into that state must be a silent no-op, not a crash.
    logger.logf("dropped %d", 1);

    // A later, usable stream is still accepted: a refused offer must not latch.
    auto good = std::make_shared<std::ostringstream>();
    EXPECT_TRUE(logger.setStream(good, "C:\\log\\spike.log"));
    logger.write("kept");
    EXPECT_NE(good->str().find("kept"), std::string::npos);
}

TEST(SpikeLogger, DefaultTimestampSourceIsTheWallClock)
{
    // Only the shape is asserted — the value is the machine's clock by design.
    spike::Logger logger;
    auto stream = std::make_shared<std::ostringstream>();
    ASSERT_TRUE(logger.setStream(stream));
    logger.write("no timestamp source set");
    EXPECT_TRUE(std::regex_match(
        stream->str(), std::regex(R"(\[\d{2}:\d{2}:\d{2}\.\d{3}\] no timestamp source set\n)")))
        << stream->str();
}

TEST(SpikeLogger, ProcessWideLoggerIsASingleInstance)
{
    EXPECT_EQ(&spike::log(), &spike::log());
}
} // namespace
