#pragma once

// IPC transport seam for the driver link (doc/driver-plan.md phase A, step 2).
//
// A `Pipe` is an already-connected, full-duplex byte stream between the driver
// (server) and the app (client). It maps 1:1 onto the Win32 named-pipe file
// handle operations — `WriteFile`, `ReadFile`, `CloseHandle` — so the real
// implementation (step 5) is three forwarders and the framing layer (built on
// top of this seam) is testable without Win32, threads, or a network.
//
// Endpoint construction is deliberately out of scope: `CreateNamedPipe` +
// `ConnectNamedPipe` (server) and `WaitNamedPipe` + `CreateFile` (client) are
// connection establishment, not stream operations, and they are asymmetrical
// between the two ends. A `Pipe` is what you get *after* the connection is up.
//
// The contract is poll-based: neither method ever blocks. A frame-driven
// caller (the driver's `RunFrame`, the app's main loop) retries on `Pending`
// until it gets `Ok`, `Closed`, or `Failed`. This keeps the seam free of
// overlapped-IO bookkeeping — step 5 can pick either a pending overlapped
// read with an owned buffer behind this interface, or `PIPE_NOWAIT` (both
// satisfy "never blocks"); the choice does not leak through here.
//
// Partial transfers are first-class: `write` may accept fewer bytes than
// offered (the framing layer buffers the tail and retries), and `read` may
// return fewer than requested (the framing layer reassembles across calls).

#include <cstddef>
#include <cstdint>
#include <string>

namespace link
{
// One per `Pipe` method call. `Pending` means "no progress now, not an error" —
// the caller retries on the next frame.
enum class IoStatus
{
    Ok,      // some bytes were transferred (check `processed`)
    Pending, // no bytes transferred and no error: empty pipe / full buffer
    Closed,  // peer closed the connection cleanly
    Failed,  // a system error; see lastError()
};

// An already-connected byte stream. The real implementation (step 5) wraps a
// Win32 `HANDLE` to a named-pipe instance; the test double (tests/FakePipe.h)
// scripts the same calls. No method throws.
class Pipe
{
public:
    virtual ~Pipe() = default;

    // WriteFile: writes up to `size` bytes, stores the accepted count in
    // `written` (0 on `Pending`). `Failed` leaves `written` at the partial
    // count accepted before the error.
    virtual IoStatus write(const std::uint8_t* data, std::size_t size,
                           std::size_t& written) = 0;

    // ReadFile: reads up to `size` bytes into `buffer`, stores the read count
    // in `read` (0 on `Pending` or `Closed`). `Closed` is the clean half-close
    // (no more data); `Failed` is a system error.
    virtual IoStatus read(std::uint8_t* buffer, std::size_t size, std::size_t& read) = 0;

    // CloseHandle: releases the underlying handle. Idempotent; safe to call
    // after `Closed`/`Failed`. Further `write`/`read` return `Closed`.
    virtual void close() = 0;

    // A short ASCII description of the last system error (the real impl
    // formats `GetLastError()`); the empty string when no error has occurred.
    virtual std::string lastError() const = 0;
};
} // namespace link