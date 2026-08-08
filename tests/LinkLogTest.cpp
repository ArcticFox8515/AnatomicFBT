// Tests for the link layer's logging (doc/driver-plan.md).
//
// Logger does one thing: hand a formatted line to a sink. No file, no
// timestamp, no flush, no mutex — spdlog (installed by the adapter) owns all
// of that. What is tested here is the one decision left in Logger: format a
// printf-style line and dispatch it, and drop it silently when no sink is
// installed.

#include "link/Log.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace
{
TEST(LinkLogger, LogfFormatsAndDispatchesTheBareMessage)
{
    std::vector<std::string> lines;
    link::Logger logger;
    logger.setSink([&](const char* message) { lines.emplace_back(message); });

    logger.logf("device %u %s", 3u, "tracker");
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines[0], "device 3 tracker");
}

TEST(LinkLogger, WriteDispatchesTheBareMessage)
{
    std::vector<std::string> lines;
    link::Logger logger;
    logger.setSink([&](const char* message) { lines.emplace_back(message); });

    logger.write("hello");
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines[0], "hello");
}

TEST(LinkLogger, LinesAreDroppedWhenNoSinkIsInstalled)
{
    // Logging before the adapter wires spdlog in (or after a test clears the sink)
    // must be a silent no-op, never a crash.
    link::Logger logger;
    logger.logf("dropped %d", 1);
    logger.write("dropped too");
}

TEST(LinkLogger, SetSinkReplacesThePreviousSink)
{
    // Tests reassign the sink; production installs it once (header invariant).
    std::vector<std::string> first;
    link::Logger logger;
    logger.setSink([&](const char* message) { first.emplace_back(message); });
    logger.write("first");

    std::vector<std::string> second;
    logger.setSink([&](const char* message) { second.emplace_back(message); });
    logger.write("second");

    ASSERT_EQ(first.size(), 1u);
    EXPECT_EQ(first[0], "first");
    ASSERT_EQ(second.size(), 1u);
    EXPECT_EQ(second[0], "second");
}

TEST(LinkLogger, ProcessWideLoggerIsASingleInstance)
{
    EXPECT_EQ(&link::log(), &link::log());
}

// ---------------------------------------------------------- sink composition ----

TEST(CompositeSink, FansEveryLineOutToBothSinksInOrder)
{
    // What routeLogToDriverLog installs: the spdlog file sink plus IVRDriverLog, so the
    // driver's output lands in our file *and* in SteamVR's vrserver.txt.
    std::vector<std::string> order;
    const link::LogSink sink =
        link::compositeSink([&](const char* message) { order.emplace_back(std::string("a:") + message); },
                             [&](const char* message) { order.emplace_back(std::string("b:") + message); });

    sink("hello");

    ASSERT_EQ(order.size(), 2u);
    EXPECT_EQ(order[0], "a:hello");
    EXPECT_EQ(order[1], "b:hello");
}

TEST(CompositeSink, AnEmptySinkIsSkippedRatherThanCalled)
{
    // Calling an empty std::function throws std::bad_function_call — inside a detour, on
    // a vrserver thread.
    std::vector<std::string> lines;
    const link::LogSink withEmptyFirst =
        link::compositeSink({}, [&](const char* message) { lines.emplace_back(message); });
    const link::LogSink withEmptySecond =
        link::compositeSink([&](const char* message) { lines.emplace_back(message); }, {});

    withEmptyFirst("first");
    withEmptySecond("second");

    ASSERT_EQ(lines.size(), 2u);
    EXPECT_EQ(lines[0], "first");
    EXPECT_EQ(lines[1], "second");
}

TEST(CompositeSink, TwoEmptySinksDropTheLine)
{
    const link::LogSink sink = link::compositeSink({}, {});
    sink("dropped");
}

// ------------------------------------------------------------- install once ----

TEST(LoggingTo, InstallsTheSinkOnALoggerThatHasNone)
{
    std::vector<std::string> lines;
    link::Logger logger;
    EXPECT_FALSE(logger.hasSink());

    link::Logger& returned = link::loggingTo(logger, [&](const char* m) { lines.emplace_back(m); });
    returned.write("hello");

    EXPECT_EQ(&returned, &logger);
    EXPECT_TRUE(logger.hasSink());
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines[0], "hello");
}

TEST(LoggingTo, KeepsTheSinkAlreadyInstalled)
{
    // The driver's providers and HmdDriverFactory all funnel through this. By the time
    // the second one runs, routeLogToDriverLog has replaced the plain file sink with the
    // composite that also feeds IVRDriverLog — re-installing would drop SteamVR's copy.
    std::vector<std::string> installed;
    std::vector<std::string> late;
    link::Logger logger;
    logger.setSink([&](const char* message) { installed.emplace_back(message); });

    link::loggingTo(logger, [&](const char* message) { late.emplace_back(message); }).write("hello");

    ASSERT_EQ(installed.size(), 1u);
    EXPECT_EQ(installed[0], "hello");
    EXPECT_TRUE(late.empty());
}
} // namespace
