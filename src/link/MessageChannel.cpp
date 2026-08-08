#include "MessageChannel.h"

#include <string>
#include <utility>

namespace link
{
namespace
{
inline constexpr std::size_t kMessageHeaderBytes = 8;
inline constexpr std::size_t kPayloadCap = sizeof(DevicePose);

static_assert(sizeof(Message) == kMessageHeaderBytes + sizeof(DevicePose),
              "Message must be header plus DevicePose");
} // namespace

MessageChannel::MessageChannel(Logger& logger, PipeFactoryFn factory)
    : log_(logger), factory_(std::move(factory))
{
}

bool MessageChannel::connected() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return connectState_ == ConnectState::Connected && pipe_ != nullptr;
}

std::string MessageChannel::lastError() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return lastError_;
}

void MessageChannel::flushLog()
{
    std::vector<std::string> lines;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lines.swap(pendingLog_);
    }
    for (const std::string& line : lines)
        log_.write(line.c_str());
}

std::size_t MessageChannel::readEnd() const
{
    if (readOffset_ < kMessageHeaderBytes)
        return kMessageHeaderBytes;
    return kMessageHeaderBytes + readMessage_.size;
}

std::size_t MessageChannel::writeEnd() const
{
    return kMessageHeaderBytes + writeMessage_.size;
}

void MessageChannel::dropPipe()
{
    if (pipe_)
    {
        for (int i = 0; i < kOverlappedCount; ++i)
            if (eventCreated_[i])
                pipe_->closeEvent(overlapped_[i]);
        if (pipeHandle_)
            pipe_->close(pipeHandle_);
    }
    pipe_.reset();
    pipeHandle_ = nullptr;
    connectState_ = ConnectState::Unconnected;
    readInFlight_ = false;
    readOffset_ = 0;
    writeInFlight_ = false;
    writeOffset_ = sizeof(Message);
    for (int i = 0; i < kOverlappedCount; ++i)
    {
        overlapped_[i].clear();
        eventCreated_[i] = false;
    }
}

void MessageChannel::pumpConnect()
{
    if (connectState_ == ConnectState::Connected)
        return;

    if (connectState_ == ConnectState::Unconnected)
    {
        const bool ok = pipe_->startConnect(pipeHandle_, overlapped_[kConnectOverlapped]);
        const IoStatus s = classifyIo(ok, pipe_->lastError());
        switch (s)
        {
        case IoStatus::Ok:
            queueLog("client connected");
            connectState_ = ConnectState::Connected;
            break;
        case IoStatus::Pending:
            connectState_ = ConnectState::Connecting;
            break;
        case IoStatus::Closed:
        case IoStatus::Failed:
            lastError_ = std::to_string(pipe_->lastError());
            queueLog("pipe dropped: {} ({})", "connect", pipe_->lastError());
            dropPipe();
            break;
        }
        return;
    }

    const bool ok = pipe_->completeConnect(pipeHandle_, overlapped_[kConnectOverlapped]);
    const IoStatus s = classifyIo(ok, pipe_->lastError());
    switch (s)
    {
    case IoStatus::Ok:
        queueLog("client connected");
        connectState_ = ConnectState::Connected;
        break;
    case IoStatus::Pending:
        break;
    case IoStatus::Closed:
    case IoStatus::Failed:
        lastError_ = std::to_string(pipe_->lastError());
        queueLog("pipe dropped: {} ({})", "connect", pipe_->lastError());
        dropPipe();
        break;
    }
}

void MessageChannel::pumpWrite()
{
    if (writeInFlight_)
    {
        std::size_t written = 0;
        const bool ok = pipe_->completeWrite(pipeHandle_, written, overlapped_[kWriteOverlapped]);
        const IoStatus s = classifyIo(ok, pipe_->lastError());
        switch (s)
        {
        case IoStatus::Ok:
            writeInFlight_ = false;
            writeOffset_ += written;
            break;
        case IoStatus::Pending:
            return;
        case IoStatus::Closed:
            queueLog("pipe dropped: {} ({})", "write", pipe_->lastError());
            dropPipe();
            return;
        case IoStatus::Failed:
            lastError_ = std::to_string(pipe_->lastError());
            queueLog("pipe dropped: {} ({})", "write", pipe_->lastError());
            dropPipe();
            return;
        }
    }

    const std::size_t end = writeEnd();
    if (writeOffset_ >= end)
        return;

    std::size_t written = 0;
    const bool ok =
        pipe_->startWrite(pipeHandle_, reinterpret_cast<std::uint8_t*>(&writeMessage_) + writeOffset_,
                          end - writeOffset_, written, overlapped_[kWriteOverlapped]);
    const IoStatus s = classifyIo(ok, pipe_->lastError());
    switch (s)
    {
    case IoStatus::Ok:
        writeOffset_ += written;
        break;
    case IoStatus::Pending:
        writeInFlight_ = true;
        return;
    case IoStatus::Closed:
        queueLog("pipe dropped: {} ({})", "write", pipe_->lastError());
        dropPipe();
        return;
    case IoStatus::Failed:
        lastError_ = std::to_string(pipe_->lastError());
        queueLog("pipe dropped: {} ({})", "write", pipe_->lastError());
        dropPipe();
        return;
    }
}

void MessageChannel::drainPipe(std::vector<Message>& out)
{
    for (;;)
    {
        if (readInFlight_)
        {
            std::size_t read = 0;
            const bool ok = pipe_->completeRead(pipeHandle_, read, overlapped_[kReadOverlapped]);
            const IoStatus s = classifyIo(ok, pipe_->lastError());
            switch (s)
            {
            case IoStatus::Ok:
                readInFlight_ = false;
                if (read == 0)
                    return;
                readOffset_ += read;
                break;
            case IoStatus::Pending:
                return;
            case IoStatus::Closed:
                queueLog("pipe dropped: {} ({})", "read", pipe_->lastError());
                dropPipe();
                return;
            case IoStatus::Failed:
                lastError_ = std::to_string(pipe_->lastError());
                queueLog("pipe dropped: {} ({})", "read", pipe_->lastError());
                dropPipe();
                return;
            }
        }
        else
        {
            std::size_t want = readEnd() - readOffset_;
            std::size_t read = 0;
            const bool ok = pipe_->startRead(pipeHandle_,
                                             reinterpret_cast<std::uint8_t*>(&readMessage_) + readOffset_,
                                             want, read, overlapped_[kReadOverlapped]);
            const IoStatus s = classifyIo(ok, pipe_->lastError());
            switch (s)
            {
            case IoStatus::Ok:
                if (read == 0)
                    return;
                readOffset_ += read;
                break;
            case IoStatus::Pending:
                readInFlight_ = true;
                return;
            case IoStatus::Closed:
                queueLog("pipe dropped: {} ({})", "read", pipe_->lastError());
                dropPipe();
                return;
            case IoStatus::Failed:
                lastError_ = std::to_string(pipe_->lastError());
                queueLog("pipe dropped: {} ({})", "read", pipe_->lastError());
                dropPipe();
                return;
            }
        }

        if (readOffset_ == kMessageHeaderBytes && readMessage_.size > kPayloadCap)
        {
            lastError_ = "payload length exceeds maximum";
            queueLog("size above cap: {}", readMessage_.size);
            dropPipe();
            return;
        }

        if (readOffset_ >= readEnd())
        {
            if (readMessage_.type == MessageType::DevicePose)
                out.push_back(readMessage_);
            else
                queueLog("unexpected type {} skipped", static_cast<unsigned>(readMessage_.type));
            readOffset_ = 0;
        }
    }
}

void MessageChannel::unsafeSend(const Message& message)
{
    if (!pipe_ || connectState_ != ConnectState::Connected)
        return;
    if (message.size > kPayloadCap)
    {
        queueLog("size above cap: {}", message.size);
        return;
    }

    pumpWrite();
    if (writeOffset_ < writeEnd())
        return;

    writeMessage_ = message;
    writeOffset_ = 0;

    std::size_t written = 0;
    const bool ok = pipe_->startWrite(pipeHandle_, reinterpret_cast<std::uint8_t*>(&writeMessage_),
                                      writeEnd(), written, overlapped_[kWriteOverlapped]);
    const IoStatus s = classifyIo(ok, pipe_->lastError());
    switch (s)
    {
    case IoStatus::Ok:
        writeOffset_ += written;
        break;
    case IoStatus::Pending:
        writeInFlight_ = true;
        break;
    case IoStatus::Closed:
        queueLog("pipe dropped: {} ({})", "write", pipe_->lastError());
        dropPipe();
        break;
    case IoStatus::Failed:
        lastError_ = std::to_string(pipe_->lastError());
        queueLog("pipe dropped: {} ({})", "write", pipe_->lastError());
        dropPipe();
        break;
    }
}

void MessageChannel::send(const Message& message)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        unsafeSend(message);
    }
    flushLog();
}

std::size_t MessageChannel::unsafeReceive(std::vector<Message>& out)
{
    if (!pipe_)
    {
        pipe_ = factory_();
        if (!pipe_)
            return 0;

        pipeHandle_ = pipe_->createPipe(pipeBufferSize, pipeBufferSize);
        if (!pipeHandle_ || pipeHandle_ == invalidHandle)
        {
            // Silent: the client's CreateFileA fails on every retry while the
            // driver is absent, and that would spam the log once per frame.
            // lastError_ still carries the win32 code for diagnostics.
            lastError_ = std::to_string(pipe_->lastError());
            pipe_.reset();
            pipeHandle_ = nullptr;
            return 0;
        }

        const std::size_t ovSize = pipe_->overlappedSize();
        for (int i = 0; i < kOverlappedCount; ++i)
            overlapped_[i].resize(ovSize);

        for (int i = 0; i < kOverlappedCount; ++i)
        {
            if (!pipe_->createEvent(overlapped_[i]))
            {
                lastError_ = std::to_string(pipe_->lastError());
                queueLog("createEvent failed ({})", pipe_->lastError());
                for (int j = i - 1; j >= 0; --j)
                {
                    pipe_->closeEvent(overlapped_[j]);
                    eventCreated_[j] = false;
                }
                pipe_->close(pipeHandle_);
                pipe_.reset();
                pipeHandle_ = nullptr;
                for (int k = 0; k < kOverlappedCount; ++k)
                    overlapped_[k].clear();
                return 0;
            }
            eventCreated_[i] = true;
        }

        connectState_ = ConnectState::Unconnected;
        readInFlight_ = false;
        readOffset_ = 0;
        writeInFlight_ = false;
        writeOffset_ = sizeof(Message);
        lastError_.clear();
    }

    pumpConnect();
    if (connectState_ != ConnectState::Connected)
        return 0;

    const std::size_t before = out.size();
    pumpWrite();
    if (pipe_)
        drainPipe(out);
    return out.size() - before;
}

std::size_t MessageChannel::receive(std::vector<Message>& out)
{
    std::size_t added;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        added = unsafeReceive(out);
    }
    flushLog();
    return added;
}
} // namespace link
