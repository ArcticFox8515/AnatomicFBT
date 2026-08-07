// Tests for the throwaway step-1 spike's log file path
// (doc/driver-plan.md, doc/driver-spike-handover.md §2.1a).
//
// This logic was a lambda inside SpikeDriver.cpp, duplicated in SpikeClient.cpp — the
// two files no test target compiles. Every branch in it (no extension, no directory
// separator, an unidentifiable executable, LOCALAPPDATA unset) therefore ran nowhere,
// while a live SteamVR run only ever takes the happy path.
//
// The environment and the executable path arrive as values through the ProcessApi fake,
// so nothing here reads the real environment or touches the filesystem (§2.1b).

#include "spike/SpikeLogFile.h"

#include <gtest/gtest.h>

#include <cstring>
#include <string>

namespace
{
class FakeProcessApi : public spike::ProcessApi
{
public:
    // GetEnvironmentVariableA: 0 when unset, and — without writing anything — the size
    // it would have needed when the value does not fit.
    unsigned long environmentVariable(const char* name, char* buffer,
                                      unsigned long size) override
    {
        requestedVariable = name;
        environmentSize = size;
        if (variableUnset)
            return 0;
        if (variableValue.size() + 1 > size)
            return static_cast<unsigned long>(variableValue.size() + 1);
        std::memcpy(buffer, variableValue.c_str(), variableValue.size());
        return static_cast<unsigned long>(variableValue.size());
    }

    // GetModuleFileNameA: fills the buffer and reports `size` when the path is longer
    // than it, so the caller gets a truncated path rather than nothing.
    unsigned long executablePath(char* buffer, unsigned long size) override
    {
        executableSize = size;
        const unsigned long written =
            static_cast<unsigned long>(executable.size() < size ? executable.size() : size);
        std::memcpy(buffer, executable.c_str(), written);
        return written;
    }

    std::string requestedVariable;
    unsigned long environmentSize = 0;
    unsigned long executableSize = 0;
    std::string variableValue = "C:\\Users\\Tester\\AppData\\Local";
    bool variableUnset = false;
    std::string executable = "C:\\Program Files\\Steam\\steamapps\\vrserver.exe";
};


// ------------------------------------------------------------- process name ----

TEST(SpikeProcessName, IsTheExecutableBaseNameWithoutExtension)
{
    EXPECT_EQ(spike::processNameFromPath("C:\\Steam\\bin\\win64\\vrserver.exe"), "vrserver");
}

TEST(SpikeProcessName, AcceptsForwardSlashes)
{
    EXPECT_EQ(spike::processNameFromPath("C:/Steam/bin/win64/vrwatchdog.exe"), "vrwatchdog");
}

TEST(SpikeProcessName, KeepsANameThatHasNoExtension)
{
    EXPECT_EQ(spike::processNameFromPath("C:\\Steam\\vrserver"), "vrserver");
}

TEST(SpikeProcessName, KeepsABareNameWithNoDirectory)
{
    EXPECT_EQ(spike::processNameFromPath("TrackingCorrectorTests.exe"),
              "TrackingCorrectorTests");
}

TEST(SpikeProcessName, StripsOnlyTheLastExtension)
{
    EXPECT_EQ(spike::processNameFromPath("C:\\Steam\\vrserver.debug.exe"), "vrserver.debug");
}

TEST(SpikeProcessName, IsUnknownWhenTheExecutableCannotBeIdentified)
{
    // GetModuleFileName failed. "driver-spike-.log" would be one shared file for every
    // such process, which defeats the point of one log per loading process.
    EXPECT_EQ(spike::processNameFromPath(""), spike::kUnknownProcessName);
}

TEST(SpikeProcessName, IsUnknownWhenThePathEndsInASeparator)
{
    EXPECT_EQ(spike::processNameFromPath("C:\\Steam\\"), spike::kUnknownProcessName);
}

TEST(SpikeProcessName, IsUnknownWhenTheNameIsNothingButAnExtension)
{
    EXPECT_EQ(spike::processNameFromPath("C:\\Steam\\.exe"), spike::kUnknownProcessName);
}

// ---------------------------------------------------------- log directory ----

TEST(SpikeLogDirectory, IsTheTrackingCorrectorFolderUnderLocalAppData)
{
    FakeProcessApi api;
    EXPECT_EQ(spike::logDirectory(api), "C:\\Users\\Tester\\AppData\\Local\\TrackingCorrector\\");
    EXPECT_EQ(api.requestedVariable, "LOCALAPPDATA");
    EXPECT_EQ(api.environmentSize, spike::kMaxLogPath);
}

TEST(SpikeLogDirectory, FallsBackToTheWorkingDirectoryWhenLocalAppDataIsUnset)
{
    FakeProcessApi api;
    api.variableUnset = true;
    EXPECT_EQ(spike::logDirectory(api), ".\\");
}

TEST(SpikeLogDirectory, FallsBackWhenLocalAppDataDoesNotFitTheBuffer)
{
    // Win32 leaves the buffer untouched in this case, so keeping what is in it would
    // build a path out of uninitialized bytes.
    FakeProcessApi api;
    api.variableValue = std::string(spike::kMaxLogPath + 10, 'x');
    EXPECT_EQ(spike::logDirectory(api), ".\\");
}

// -------------------------------------------------------------- full path ----

TEST(SpikeProcessLogPath, IsOneFilePerLoadingProcessUnderLocalAppData)
{
    FakeProcessApi api;
    EXPECT_EQ(spike::processLogPath(api, spike::kDriverLogPrefix),
              "C:\\Users\\Tester\\AppData\\Local\\TrackingCorrector\\driver-spike-vrserver.log");
    EXPECT_EQ(api.executableSize, spike::kMaxLogPath);
}

TEST(SpikeProcessLogPath, UsesTheGivenPrefix)
{
    FakeProcessApi api;
    api.executable = "D:\\Dev\\TrackingCorrector\\build\\spike_client.exe";
    EXPECT_EQ(spike::processLogPath(api, spike::kClientLogPrefix),
              "C:\\Users\\Tester\\AppData\\Local\\TrackingCorrector\\client-spike-spike_client.log");
}

TEST(SpikeProcessLogPath, FallsBackOnBothHalvesAtOnce)
{
    FakeProcessApi api;
    api.variableUnset = true;
    api.executable = "";
    EXPECT_EQ(spike::processLogPath(api, spike::kDriverLogPrefix), ".\\driver-spike-unknown.log");
}

TEST(SpikeProcessLogPath, KeepsOnlyWhatFitWhenTheExecutablePathIsTruncated)
{
    // GetModuleFileName fills the buffer and reports its size; a buffer-sized string
    // full of NULs would otherwise end up in the middle of the path.
    FakeProcessApi api;
    api.executable = std::string(spike::kMaxLogPath + 50, 'x');
    const std::string path = spike::processLogPath(api, spike::kDriverLogPrefix);
    EXPECT_EQ(path,
              "C:\\Users\\Tester\\AppData\\Local\\TrackingCorrector\\driver-spike-"
                  + std::string(spike::kMaxLogPath, 'x') + ".log");
}
} // namespace
