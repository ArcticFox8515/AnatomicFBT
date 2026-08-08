#pragma once

// Logger for the link layer: format a line and hand it to a sink. No file, no
// timestamp, no flush, no mutex — spdlog (installed by the adapter, Driver.cpp)
// owns destinations, timestamps, buffering and flushing. LinkLib
// stays spdlog-free so the test binary and PipeLib do not link it; the spdlog logger is
// created in the adapter side and wired in via setSink before any hook is installed.
//
// Production invariant: the sink is installed once and never cleared. A hook thread
// reads `sink_` on every detour entry; the adapter writes it before installing hooks
// (Server::init wires logging, then hooks), so the read is sequenced after the
// write with no lock. setSink is only callable more than once from tests, which are
// single-threaded.

#include <functional>

namespace link
{
using LogSink = std::function<void(const char*)>;

// Fans every line out to both sinks, in order (the driver's file sink first, then
// IVRDriverLog). An empty sink is skipped rather than called — std::function would
// throw std::bad_function_call, inside a detour running on a vrserver thread.
LogSink compositeSink(LogSink first, LogSink second);

class Logger
{
public:
    Logger() = default;

    // Installed once by the adapter before any hook is installed (see header
    // note). Callable again from tests (single-threaded).
    void setSink(LogSink sink);

    // Whether a sink is installed, i.e. whether logged lines go anywhere.
    bool hasSink() const;

    // printf-style; the message is handed to the sink verbatim. If no sink is
    // installed, the line is dropped. A throw from the sink propagates — every
    // caller is inside runGuarded (Server / the detour forwarders).
    void logf(const char* format, ...);

    // The bare-message dispatch logf uses; exposed so tests can send a line
    // without a format string.
    void write(const char* message);

private:
    LogSink sink_;
};

// Process-wide logger: the detours and the providers are reached through C ABI
// entry points with nowhere to pass a logger through. Deliberately leaked
// (never destroyed) because a detour may still run after the DLL's static
// destructors.
Logger& log();

// Gives `logger` the sink if it has none yet and returns it, so the adapter's
// "open the log" accessor is a single expression. Every provider entry point and
// HmdDriverFactory funnel through it, and only the first arrival may win: by the time
// the second one runs, ServerEnvironment::routeLogToDriverLog has replaced the sink
// with the composite that also feeds IVRDriverLog, and re-installing the plain file
// sink would silently drop SteamVR's copy of the log.
Logger& loggingTo(Logger& logger, LogSink sink);
} // namespace link
