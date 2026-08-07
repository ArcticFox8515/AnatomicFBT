#pragma once

// Throwaway step-1 spike (doc/driver-plan.md): observation-only logging used by
// both the spike driver DLL and the spike client. Deleted with the rest of
// src/spike once the real driver exists.
//
// Structured so that every decision is unit-testable without touching the
// filesystem, the clock or the environment: the path building and the line
// formatting are pure functions, and `Logger` is handed a stream, a sink and a
// timestamp source from outside. The three functions at the bottom of this header
// are the adapter — branch-free forwarders over Win32 / ofstream — and are the only
// part of this file exempt from the "every line runs in a test" bar.

#include <functional>
#include <memory>
#include <mutex>
#include <ostream>
#include <string>

namespace spike
{
using LogSink = std::function<void(const char*)>;
using TimestampFn = std::function<std::string()>;

// "C:\...\vrserver.exe" -> "vrserver". A path without a separator or without an
// extension is handled; an empty path yields "unknown".
std::string processBaseName(const std::string& modulePath);

// The two pieces of the environment the log path depends on, as values, so the path
// building stays a pure function.
struct LogEnvironment
{
    std::string localAppData; // empty when the variable is not set
    std::string modulePath;   // empty when it could not be read
};

// "<localAppData>\TrackingCorrector\", or "" when LOCALAPPDATA is not set (the log
// then lands in the working directory rather than nowhere).
std::string logDirectory(const std::string& localAppData);

// <directory><prefix>-<processName>.log. The process name is part of the file name
// because SteamVR loads the same DLL into vrserver.exe and vrwatchdog.exe.
std::string logPath(const LogEnvironment& environment, const std::string& prefix);

// "hh:mm:ss.mmm".
std::string formatTimestamp(unsigned hours, unsigned minutes, unsigned seconds,
                            unsigned milliseconds);

// The line as it lands in the file.
std::string formatLogLine(const std::string& timestamp, const std::string& message);

class Logger
{
public:
    Logger();

    // First stream wins: every entry point offers one (the DLL is entered through
    // HmdDriverFactory, both providers' Init, and nothing guarantees an order), and
    // only the first usable one is kept. Returns whether the logger is open
    // afterwards — a stream that failed to open is refused rather than stored, so a
    // log that cannot be written is never the reason the driver fails.
    bool setStream(std::shared_ptr<std::ostream> stream, std::string path = {});

    // Additional sink receiving the same lines (the DLL routes them to IVRDriverLog
    // so they also land in vrserver.txt).
    void setSink(LogSink sink);

    void setTimestampSource(TimestampFn timestamp);

    bool isOpen() const;

    // By value, under the lock: read from hook threads while Init may still be
    // writing it.
    std::string filePath() const;

    // printf-style; flushed on every call so a crash keeps the tail.
    void logf(const char* format, ...);

    void write(const std::string& message);

    void close();

private:
    mutable std::mutex mutex_;
    std::shared_ptr<std::ostream> stream_;
    LogSink sink_;
    TimestampFn timestamp_;
    std::string path_;
};

// ---- adapter: Win32 / filesystem forwarders, no decisions of their own ----

// Process-wide logger: the detours and the providers are reached through C ABI entry
// points with nowhere to pass a logger through. Deliberately leaked (never
// destroyed) because a detour may still run after the DLL's static destructors.
Logger& log();

// GetEnvironmentVariableA("LOCALAPPDATA") + GetModuleFileNameA; a failed call leaves
// the corresponding string empty.
LogEnvironment currentLogEnvironment();

// GetLocalTime + formatTimestamp.
std::string localTimestamp();

// Offers the logger an append-mode ofstream at logPath(currentLogEnvironment(),
// prefix), creating the directory if it can. Whether that stream is adopted is
// Logger::setStream's decision; the return value is Logger's answer.
bool openProcessLog(Logger& logger, const std::string& prefix);
} // namespace spike
