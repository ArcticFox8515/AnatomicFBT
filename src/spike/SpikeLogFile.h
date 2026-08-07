#pragma once

// Throwaway step-1 spike (doc/driver-plan.md): where the spike's log file goes.
//
// This used to be a lambda inside SpikeDriver.cpp (and a copy of it inside
// SpikeClient.cpp), i.e. in the two files no unit test can execute — with a basename
// scan, an extension strip, an empty-name fallback and a "%LOCALAPPDATA% unset"
// fallback in it, none of which ran anywhere (doc/driver-spike-handover.md §2.1a).
// It is here instead, with the only two things that cannot be faked — the
// environment block and the running executable's path — injected as `ProcessApi`.
//
// Nothing here touches the filesystem: the path is computed, and spdlog's file sink
// (created by the adapter) creates the directory tree when it opens the file.

#include <string>

namespace spike
{
// MAX_PATH, so this header needs no windows.h.
constexpr unsigned long kMaxLogPath = 260;

// One file per loading process, so vrserver / vrwatchdog / the test executable do not
// interleave (doc/driver-spike-handover.md §8).
inline constexpr const char* kDriverLogPrefix = "driver-spike-";
inline constexpr const char* kClientLogPrefix = "client-spike-";

// The name the log file is reported as when the executable cannot be identified.
inline constexpr const char* kUnknownProcessName = "unknown";

// Deliberately as primitive as the Win32 calls behind it: caller's buffer, so the
// DLL's implementation is one call each and the buffer handling — which is where the
// mistakes are — happens here, under test.
class ProcessApi
{
public:
    virtual ~ProcessApi() = default;

    // GetEnvironmentVariableA: characters written excluding the terminator, 0 when the
    // variable is unset (and `size` or more when the value did not fit).
    virtual unsigned long environmentVariable(const char* name, char* buffer,
                                             unsigned long size) = 0;
    // GetModuleFileNameA(nullptr, ...): the running executable's full path.
    virtual unsigned long executablePath(char* buffer, unsigned long size) = 0;
};

// "C:\Program Files\Steam\...\vrserver.exe" -> "vrserver". Empty or nameless paths
// answer kUnknownProcessName, because "driver-spike-.log" would silently be one shared
// file for every process that failed the lookup.
std::string processNameFromPath(const std::string& executablePath);

// "%LOCALAPPDATA%\TrackingCorrector\", or ".\" when LOCALAPPDATA is unset — vrserver
// runs as the logged-in user, but a service-hosted process might not.
std::string logDirectory(ProcessApi& api);

// <log directory><prefix><process name>.log — the whole path the adapter hands to
// spdlog, and the entire body of the adapter's path helper.
std::string processLogPath(ProcessApi& api, const char* prefix);
} // namespace spike
