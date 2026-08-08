// Tests for the driver's log file path (doc/driver-plan.md).
//
// This logic was a lambda inside Driver.cpp — the file no test target compiles.
// Every branch in it (no extension, no directory separator, an unidentifiable
// executable, LOCALAPPDATA unset) therefore ran nowhere, while a live SteamVR run
// only ever takes the happy path.
//
// The environment and the executable path arrive as values through the ProcessApi fake,
// so nothing here reads the real environment or touches the filesystem.

#include "driver/LogFile.h"

#include <gtest/gtest.h>

#include <cstring>
#include <string>

namespace
{
class FakeProcessApi : public driver::ProcessApi
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

TEST(ProcessName, IsTheExecutableBaseNameWithoutExtension)
{
    EXPECT_EQ(driver::processNameFromPath("C:\\Steam\\bin\\win64\\vrserver.exe"), "vrserver");
}

TEST(ProcessName, AcceptsForwardSlashes)
{
    EXPECT_EQ(driver::processNameFromPath("C:/Steam/bin/win64/vrwatchdog.exe"), "vrwatchdog");
}

TEST(ProcessName, KeepsANameThatHasNoExtension)
{
    EXPECT_EQ(driver::processNameFromPath("C:\\Steam\\vrserver"), "vrserver");
}

TEST(ProcessName, KeepsABareNameWithNoDirectory)
{
    EXPECT_EQ(driver::processNameFromPath("TrackingCorrectorTests.exe"),
              "TrackingCorrectorTests");
}

TEST(ProcessName, StripsOnlyTheLastExtension)
{
    EXPECT_EQ(driver::processNameFromPath("C:\\Steam\\vrserver.debug.exe"), "vrserver.debug");
}

TEST(ProcessName, IsUnknownWhenTheExecutableCannotBeIdentified)
{
    // GetModuleFileName failed. "driver-.log" would be one shared file for every
    // such process, which defeats the point of one log per loading process.
    EXPECT_EQ(driver::processNameFromPath(""), driver::kUnknownProcessName);
}

TEST(ProcessName, IsUnknownWhenThePathEndsInASeparator)
{
    EXPECT_EQ(driver::processNameFromPath("C:\\Steam\\"), driver::kUnknownProcessName);
}

TEST(ProcessName, IsUnknownWhenTheNameIsNothingButAnExtension)
{
    EXPECT_EQ(driver::processNameFromPath("C:\\Steam\\.exe"), driver::kUnknownProcessName);
}

// ---------------------------------------------------------- log directory ----

TEST(LogDirectory, IsTheTrackingCorrectorFolderUnderLocalAppData)
{
    FakeProcessApi api;
    EXPECT_EQ(driver::logDirectory(api), "C:\\Users\\Tester\\AppData\\Local\\TrackingCorrector\\");
    EXPECT_EQ(api.requestedVariable, "LOCALAPPDATA");
    EXPECT_EQ(api.environmentSize, driver::kMaxLogPath);
}

TEST(LogDirectory, FallsBackToTheWorkingDirectoryWhenLocalAppDataIsUnset)
{
    FakeProcessApi api;
    api.variableUnset = true;
    EXPECT_EQ(driver::logDirectory(api), ".\\");
}

TEST(LogDirectory, FallsBackWhenLocalAppDataDoesNotFitTheBuffer)
{
    // Win32 leaves the buffer untouched in this case, so keeping what is in it would
    // build a path out of uninitialized bytes.
    FakeProcessApi api;
    api.variableValue = std::string(driver::kMaxLogPath + 10, 'x');
    EXPECT_EQ(driver::logDirectory(api), ".\\");
}

// -------------------------------------------------------------- full path ----

TEST(ProcessLogPath, IsOneFilePerLoadingProcessUnderLocalAppData)
{
    FakeProcessApi api;
    EXPECT_EQ(driver::processLogPath(api, driver::kDriverLogPrefix),
              "C:\\Users\\Tester\\AppData\\Local\\TrackingCorrector\\driver-vrserver.log");
    EXPECT_EQ(api.executableSize, driver::kMaxLogPath);
}

TEST(ProcessLogPath, FallsBackOnBothHalvesAtOnce)
{
    FakeProcessApi api;
    api.variableUnset = true;
    api.executable = "";
    EXPECT_EQ(driver::processLogPath(api, driver::kDriverLogPrefix), ".\\driver-unknown.log");
}

TEST(ProcessLogPath, KeepsOnlyWhatFitWhenTheExecutablePathIsTruncated)
{
    // GetModuleFileName fills the buffer and reports its size; a buffer-sized string
    // full of NULs would otherwise end up in the middle of the path.
    FakeProcessApi api;
    api.executable = std::string(driver::kMaxLogPath + 50, 'x');
    const std::string path = driver::processLogPath(api, driver::kDriverLogPrefix);
    EXPECT_EQ(path,
              "C:\\Users\\Tester\\AppData\\Local\\TrackingCorrector\\driver-"
                  + std::string(driver::kMaxLogPath, 'x') + ".log");
}
} // namespace
