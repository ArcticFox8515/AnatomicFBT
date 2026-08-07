#pragma once

// Test double for `link::Pipe` (doc/driver-plan.md phase A, step 2).
//
// Scripts the same calls the real Win32 pipe handles — `write`, `read`,
// `close` — so the framing layer is exercised without Win32, threads, or a
// network. Lives in the test target only; the real implementation arrives at
// step 5 and is the only part of the link not covered by unit tests.

#include "link/Pipe.h"

#include <cstring>
#include <string>
#include <vector>

namespace link_test
{
class FakePipe : public link::Pipe
{
public:
    // ---- write side (the driver's send path) ----

    // Per-call cap on accepted bytes; 0 = accept all. A `writeCap` of N makes
    // the next `write` accept at most N bytes and return `Ok` with `written`
    // set to min(N, size); a subsequent call resumes the tail.
    std::size_t writeCap = 0;

    // After this many bytes have been accepted in total, `write` returns
    // `Pending` instead of accepting more. 0 = no budget (always accept, up to
    // writeCap). Set to a small N to force a partial send that survives a
    // `send` call and only drains on a later `flush`; reset to 0 to release.
    std::size_t writePendingAfter = 0;
    std::size_t writeAcceptedTotal = 0;

    // Force `write` to return this status instead of accepting bytes. `Failed`
    // also sets lastError to `forcedError`.
    link::IoStatus writeForce = link::IoStatus::Ok;
    std::string forcedError;

    // Every byte the pipe accepted, in order.
    std::vector<std::uint8_t> written;

    // ---- read side (the app's receive path) ----

    // Bytes queued for the app to read, consumed FIFO. Feed this with
    // `feedRead` (often in small chunks to exercise split reads).
    std::vector<std::uint8_t> readQueue;

    // Force `read` to return this status when `readQueue` is empty (or always,
    // when `readForceAlways` is set). `Closed`/`Failed` set the channel state.
    link::IoStatus readForce = link::IoStatus::Pending;
    bool readForceAlways = false;

    // ---- close / state ----

    bool isClosed = false;
    std::string lastErrorText;

    void feedRead(const std::uint8_t* data, std::size_t size)
    {
        readQueue.insert(readQueue.end(), data, data + size);
    }

    void feedRead(const std::vector<std::uint8_t>& bytes)
    {
        feedRead(bytes.data(), bytes.size());
    }

    link::IoStatus write(const std::uint8_t* data, std::size_t size,
                          std::size_t& writtenOut) override
    {
        if (writeForce == link::IoStatus::Failed)
        {
            lastErrorText = forcedError;
            return link::IoStatus::Failed;
        }
        if (writeForce == link::IoStatus::Closed)
            return link::IoStatus::Closed;
        if (writeForce == link::IoStatus::Pending)
        {
            writtenOut = 0;
            return link::IoStatus::Pending;
        }
        if (writePendingAfter && writeAcceptedTotal >= writePendingAfter)
        {
            writtenOut = 0;
            return link::IoStatus::Pending;
        }
        const std::size_t cap = writeCap == 0 ? size : (std::min)(writeCap, size);
        std::size_t accept = cap;
        if (writePendingAfter && writeAcceptedTotal + accept > writePendingAfter)
            accept = writePendingAfter - writeAcceptedTotal;
        if (accept == 0 && size > 0)
        {
            writtenOut = 0;
            return link::IoStatus::Pending;
        }
        written.insert(written.end(), data, data + accept);
        writeAcceptedTotal += accept;
        writtenOut = accept;
        return link::IoStatus::Ok;
    }

    link::IoStatus read(std::uint8_t* buffer, std::size_t size, std::size_t& readOut) override
    {
        if (readForceAlways && readForce != link::IoStatus::Ok)
        {
            readOut = 0;
            if (readForce == link::IoStatus::Failed)
                lastErrorText = forcedError;
            return readForce;
        }
        if (readQueue.empty())
        {
            readOut = 0;
            if (readForce == link::IoStatus::Failed)
                lastErrorText = forcedError;
            return readForce;
        }
        const std::size_t give = (std::min)(size, readQueue.size());
        std::memcpy(buffer, readQueue.data(), give);
        readQueue.erase(readQueue.begin(), readQueue.begin() + give);
        readOut = give;
        return link::IoStatus::Ok;
    }

    void close() override { isClosed = true; }

    std::string lastError() const override { return lastErrorText; }
};
} // namespace link_test