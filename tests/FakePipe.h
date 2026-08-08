#pragma once

#include "link/Pipe.h"

#include <algorithm>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace link_test
{
class FakePipe : public link::Pipe
{
public:
    // ---- connect ----
    bool startConnectResult = true;
    unsigned long startConnectErr = link::errIoPending;
    bool completeConnectResult = true;
    unsigned long completeConnectErr = link::errIoPending;
    bool connectInFlight = false;

    // ---- read side ----
    std::vector<std::uint8_t> readQueue;
    unsigned long readEmptyErr = link::errIoPending;

    // ---- write side ----
    unsigned long writeForceErr = 0;
    std::size_t writeChunk = 0;
    std::size_t writePendingAfter = 0;
    std::size_t writeAcceptedTotal = 0;
    std::vector<std::uint8_t> written;

    // ---- shared ----
    unsigned long forcedErr = 0;

    // createPipe result: non-null = success, null = failure.
    void* createPipeResult = reinterpret_cast<void*>(1);
    int createPipeCallCount = 0;
    std::string createPipeName;
    std::size_t createPipeInBufferSize = 0;
    std::size_t createPipeOutBufferSize = 0;

    // The handle createPipe returned — used to verify subsequent calls pass
    // the same handle.
    void* createdHandle = nullptr;

    // Handles passed to each method (for verifying the caller passes the
    // correct handle).
    std::vector<void*> handlesPassedToConnect;
    std::vector<void*> handlesPassedToRead;
    std::vector<void*> handlesPassedToWrite;
    std::vector<void*> handlesPassedToClose;

    // Creation result per call index (0, 1, 2 in call order).
    int createEventResults[3] = {1, 1, 1};
    int createEventCallCount = 0;

    // closeEvent return value. Default true (success).
    bool closeEventResult = true;

    // Whether close() was called and with what handle.
    bool closeCalled = false;
    void* closeHandle = nullptr;

    // Pointers received in createEvent calls (in order).
    std::vector<void*> createEventPtrs;
    // Pointers received in closeEvent calls (in order).
    std::vector<void*> closeEventPtrs;

    // The overlapped size this fake reports (default 32).
    std::size_t overlappedSizeResult = 32;

    void feedRead(const std::uint8_t* data, std::size_t size)
    {
        readQueue.insert(readQueue.end(), data, data + size);
    }

    void feedRead(const std::vector<std::uint8_t>& bytes)
    {
        feedRead(bytes.data(), bytes.size());
    }

    // ---- Pipe interface ----

    std::size_t overlappedSize() const override { return overlappedSizeResult; }

    void* createPipe(const char* name, std::size_t inBufferSize,
                     std::size_t outBufferSize) override
    {
        createPipeCallCount++;
        createPipeName = name ? name : "";
        createPipeInBufferSize = inBufferSize;
        createPipeOutBufferSize = outBufferSize;
        err_ = createPipeResult ? 0u : forcedErr;
        createdHandle = createPipeResult;
        return createPipeResult;
    }

    bool createEvent(void* ov) override
    {
        createEventPtrs.push_back(ov);
        const int result = createEventResults[createEventCallCount++];
        err_ = result ? 0u : forcedErr;
        return result != 0;
    }

    bool closeEvent(void* ov) override
    {
        closeEventPtrs.push_back(ov);
        err_ = closeEventResult ? 0u : forcedErr;
        return closeEventResult;
    }

    bool startConnect(void* handle, void*) override
    {
        handlesPassedToConnect.push_back(handle);
        if (!startConnectResult && startConnectErr == link::errIoPending)
            connectInFlight = true;
        err_ = startConnectResult ? 0u : startConnectErr;
        return startConnectResult;
    }

    bool completeConnect(void* handle, void*) override
    {
        handlesPassedToConnect.push_back(handle);
        if (!connectInFlight)
        {
            err_ = 5;
            return false;
        }
        connectInFlight = false;
        err_ = completeConnectResult ? 0u : completeConnectErr;
        return completeConnectResult;
    }

    bool startRead(void* handle, std::uint8_t* buffer, std::size_t size,
                   std::size_t& readOut, void*) override
    {
        handlesPassedToRead.push_back(handle);
        if (readInFlight_)
        {
            err_ = 5;
            return false;
        }

        if (readQueue.empty())
        {
            readOut = 0;
            err_ = readEmptyErr;
            if (readEmptyErr == link::errIoPending)
            {
                readInFlight_ = true;
                pendingReadBuffer_ = buffer;
                pendingReadSize_ = size;
            }
            return false;
        }

        const std::size_t give = (std::min)(size, readQueue.size());
        std::memcpy(buffer, readQueue.data(), give);
        readQueue.erase(readQueue.begin(), readQueue.begin() + give);
        readOut = give;
        err_ = 0;
        return true;
    }

    bool completeRead(void* handle, std::size_t& readOut, void*) override
    {
        handlesPassedToRead.push_back(handle);
        if (!readInFlight_)
        {
            err_ = 5;
            return false;
        }

        if (readQueue.empty())
        {
            readOut = 0;
            err_ = readEmptyErr;
            if (readEmptyErr != link::errIoPending)
                readInFlight_ = false;
            return false;
        }

        readInFlight_ = false;
        const std::size_t give = (std::min)(pendingReadSize_, readQueue.size());
        std::memcpy(pendingReadBuffer_, readQueue.data(), give);
        readQueue.erase(readQueue.begin(), readQueue.begin() + give);
        readOut = give;
        err_ = 0;
        return true;
    }

    bool startWrite(void* handle, const std::uint8_t* data, std::size_t size,
                    std::size_t& writtenOut, void*) override
    {
        handlesPassedToWrite.push_back(handle);
        if (writeForceErr != 0)
        {
            writtenOut = 0;
            err_ = writeForceErr;
            if (writeForceErr == link::errIoPending)
            {
                pendingWriteData_ = data;
                pendingWriteSize_ = size;
                writeInFlight_ = true;
            }
            return false;
        }

        std::size_t accept = writeChunk == 0 ? size : (std::min)(writeChunk, size);
        if (writePendingAfter && writeAcceptedTotal >= writePendingAfter)
        {
            writtenOut = 0;
            err_ = link::errIoPending;
            pendingWriteData_ = data;
            pendingWriteSize_ = size;
            writeInFlight_ = true;
            return false;
        }
        if (writePendingAfter && writeAcceptedTotal + accept > writePendingAfter)
            accept = writePendingAfter - writeAcceptedTotal;
        if (accept == 0 && size > 0)
        {
            writtenOut = 0;
            err_ = link::errIoPending;
            pendingWriteData_ = data;
            pendingWriteSize_ = size;
            writeInFlight_ = true;
            return false;
        }

        written.insert(written.end(), data, data + accept);
        writeAcceptedTotal += accept;
        writtenOut = accept;
        err_ = 0;
        return true;
    }

    bool completeWrite(void* handle, std::size_t& writtenOut, void*) override
    {
        handlesPassedToWrite.push_back(handle);
        if (!writeInFlight_)
        {
            err_ = 5;
            return false;
        }

        if (writeForceErr == link::errIoPending)
        {
            writtenOut = 0;
            err_ = link::errIoPending;
            return false;
        }

        writeInFlight_ = false;
        if (pendingWriteSize_ > 0)
        {
            written.insert(written.end(), pendingWriteData_,
                           pendingWriteData_ + pendingWriteSize_);
            writeAcceptedTotal += pendingWriteSize_;
        }
        writtenOut = pendingWriteSize_;
        pendingWriteData_ = nullptr;
        pendingWriteSize_ = 0;
        err_ = 0;
        return true;
    }

    bool close(void* handle) override
    {
        closeCalled = true;
        closeHandle = handle;
        handlesPassedToClose.push_back(handle);
        readInFlight_ = false;
        writeInFlight_ = false;
        connectInFlight = false;
        pendingWriteData_ = nullptr;
        pendingWriteSize_ = 0;
        return true;
    }

    unsigned long lastError() const override { return err_; }

private:
    bool readInFlight_ = false;
    std::uint8_t* pendingReadBuffer_ = nullptr;
    std::size_t pendingReadSize_ = 0;

    bool writeInFlight_ = false;
    const std::uint8_t* pendingWriteData_ = nullptr;
    std::size_t pendingWriteSize_ = 0;

    unsigned long err_ = 0;
};

inline link::PipeFactoryFn borrowPipeFactory(FakePipe& pipe)
{
    return [&pipe] {
        return std::shared_ptr<link::Pipe>(&pipe, [](link::Pipe*) {});
    };
}
} // namespace link_test
