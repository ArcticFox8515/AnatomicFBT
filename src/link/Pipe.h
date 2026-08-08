#pragma once

// IPC transport seam for the driver link (doc/driver-plan.md phase A, step 5).
//
// A `Pipe` is a named-pipe instance. The real implementation
// (src/spike/SpikeDriverPipe.cpp, driver DLL only) is a pure wrapper over winapi:
// no logic, no constants, no state, no decisions. Every method is exactly one
// winapi call. All decisions live in `MessageChannel` (this layer, unit-tested).
//
// The overlapped buffers are owned by `MessageChannel` (the tested layer), sized
// via `overlappedSize()` (the implementation knows sizeof(OVERLAPPED)). The I/O
// buffer size is dictated by application needs and set by `MessageChannel`.
//
// The seam is overlapped-IO-native: each direction is a start/complete pair.
// `startX` issues the winapi call into the caller-owned overlapped buffer;
// `completeX` polls `GetOverlappedResult` on the same buffer. On a zero return
// (failure) the caller checks `lastError()` to classify the result. At most one
// op in flight per direction; enforcing that is the caller's job (tested).
//
// `PipeFactoryFn` constructs an *unconnected* instance (`CreateNamedPipe` in
// the ctor). `MessageChannel` drives the connection state machine, then
// read/write. A dead pipe is dropped and the factory makes the next instance.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

namespace link
{
class Pipe;

using PipeFactoryFn = std::function<std::shared_ptr<Pipe>()>;

enum class IoStatus
{
    Ok,
    Pending,
    Closed,
    Failed,
};

// Win32 error codes `classifyIo` maps. Defined as literals so LinkLib needs no
// windows.h; the values match the ERROR_* macros in winerror.h.
inline constexpr unsigned long errIoPending = 997;
inline constexpr unsigned long errIoIncomplete = 996;
inline constexpr unsigned long errBrokenPipe = 109;
inline constexpr unsigned long errPipeNotConnected = 229;
inline constexpr unsigned long errNoData = 232;
inline constexpr unsigned long errOperationAborted = 995;
inline constexpr unsigned long errPipeConnected = 535;

// The named-pipe path the driver creates and the app connects to.
inline constexpr char kDriverPipeName[] = R"(\\.\pipe\TrackingCorrector.driver)";

// I/O buffer size for the named pipe (in + out), dictated by application needs.
inline constexpr std::size_t pipeBufferSize = 65536;

// INVALID_HANDLE_VALUE on x64 — CreateNamedPipeA returns this on failure.
// Defined as extern (reinterpret_cast isn't constexpr) in Pipe.cpp.
extern const void* invalidHandle;

// Overlapped buffer indices: 0=connect, 1=read, 2=write. MessageChannel owns
// the buffers and passes the right one to each method.
inline constexpr int kConnectOverlapped = 0;
inline constexpr int kReadOverlapped = 1;
inline constexpr int kWriteOverlapped = 2;
inline constexpr int kOverlappedCount = 3;

IoStatus classifyIo(int ok, unsigned long lastError);

class Pipe
{
public:
    virtual ~Pipe() = default;

    // Returns sizeof(OVERLAPPED) — the size the caller must allocate for each
    // overlapped buffer. The implementation knows the real struct size.
    virtual std::size_t overlappedSize() const = 0;

    // CreateNamedPipeA. Returns the handle (void*). The caller owns it and
    // passes it to every subsequent method. Null or invalidHandle = failure.
    virtual void* createPipe(const char* name, std::size_t inBufferSize,
                             std::size_t outBufferSize) = 0;

    // CreateEventW. Stores the event into the overlapped buffer. Returns true
    // on success, false on failure (caller checks lastError).
    virtual bool createEvent(void* overlapped) = 0;

    // CloseHandle on the event stored in the overlapped buffer.
    virtual bool closeEvent(void* overlapped) = 0;

    // ConnectNamedPipe(handle, overlapped). Returns true on success/async-pending.
    virtual bool startConnect(void* handle, void* overlapped) = 0;

    // GetOverlappedResult(handle, overlapped, FALSE).
    virtual bool completeConnect(void* handle, void* overlapped) = 0;

    // WriteFile(handle, data, size, &written, overlapped).
    virtual bool startWrite(void* handle, const std::uint8_t* data, std::size_t size,
                           std::size_t& written, void* overlapped) = 0;

    // GetOverlappedResult(handle, overlapped, &written, FALSE).
    virtual bool completeWrite(void* handle, std::size_t& written, void* overlapped) = 0;

    // ReadFile(handle, buffer, size, &read, overlapped).
    virtual bool startRead(void* handle, std::uint8_t* buffer, std::size_t size,
                          std::size_t& read, void* overlapped) = 0;

    // GetOverlappedResult(handle, overlapped, &read, FALSE).
    virtual bool completeRead(void* handle, std::size_t& read, void* overlapped) = 0;

    // CloseHandle(handle).
    virtual bool close(void* handle) = 0;

    // GetLastError().
    virtual unsigned long lastError() const = 0;
};
} // namespace link