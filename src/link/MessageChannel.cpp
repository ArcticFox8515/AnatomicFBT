#include "MessageChannel.h"

#include <cstring>
#include <string>
#include <utility>

namespace link
{
namespace
{
struct FrameHeader
{
    std::uint32_t payloadLength = 0;
    MessageType type = MessageType::DevicePose;
};
static_assert(sizeof(FrameHeader) == 8, "FrameHeader must be 8 bytes (u32 + u16 + 2 pad)");
} // namespace

MessageChannel::MessageChannel(PipeFactoryFn factory) : factory_(std::move(factory))
{
    writing_.reserve(kMaxPayloadBytes);
}

std::string MessageChannel::lastError() const
{
    return lastError_;
}

void MessageChannel::appendOutbound(const std::uint8_t* data, std::size_t size)
{
    outbound_.insert(outbound_.end(), data, data + size);
}

void MessageChannel::dropPipe()
{
    if (pipe_)
    {
        for (int i = 0; i < kOverlappedCount; ++i)
            if (eventCreated_[i])
                pipe_->closeEvent(overlapped_[i].data());
        if (pipeHandle_)
            pipe_->close(pipeHandle_);
    }
    pipe_.reset();
    pipeHandle_ = nullptr;
    connectState_ = ConnectState::Unconnected;
    readInFlight_ = false;
    writeInFlight_ = false;
    outbound_.clear();
    writing_.clear();
    writingOffset_ = 0;
    inbound_.clear();
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
        const bool ok = pipe_->startConnect(pipeHandle_, overlapped_[kConnectOverlapped].data());
        const IoStatus s = classifyIo(ok, pipe_->lastError());
        switch (s)
        {
        case IoStatus::Ok:
            connectState_ = ConnectState::Connected;
            break;
        case IoStatus::Pending:
            connectState_ = ConnectState::Connecting;
            break;
        case IoStatus::Closed:
        case IoStatus::Failed:
            lastError_ = std::to_string(pipe_->lastError());
            dropPipe();
            break;
        }
        return;
    }

    const bool ok = pipe_->completeConnect(pipeHandle_, overlapped_[kConnectOverlapped].data());
    const IoStatus s = classifyIo(ok, pipe_->lastError());
    switch (s)
    {
    case IoStatus::Ok:
        connectState_ = ConnectState::Connected;
        break;
    case IoStatus::Pending:
        break;
    case IoStatus::Closed:
    case IoStatus::Failed:
        lastError_ = std::to_string(pipe_->lastError());
        dropPipe();
        break;
    }
}

void MessageChannel::pumpWrite()
{
    for (;;)
    {
        if (writeInFlight_)
        {
            std::size_t written = 0;
            const bool ok = pipe_->completeWrite(pipeHandle_, written, overlapped_[kWriteOverlapped].data());
            const IoStatus s = classifyIo(ok, pipe_->lastError());
            switch (s)
            {
            case IoStatus::Ok:
                writeInFlight_ = false;
                writingOffset_ += written;
                if (writingOffset_ < writing_.size())
                    continue;
                writing_.clear();
                writingOffset_ = 0;
                break;
            case IoStatus::Pending:
                return;
            case IoStatus::Closed:
                dropPipe();
                return;
            case IoStatus::Failed:
                lastError_ = std::to_string(pipe_->lastError());
                dropPipe();
                return;
            }
        }

        if (writingOffset_ >= writing_.size())
        {
            if (outbound_.empty())
                return;
            std::swap(outbound_, writing_);
            outbound_.clear();
            writingOffset_ = 0;
        }

        std::size_t written = 0;
        const bool ok =
            pipe_->startWrite(pipeHandle_, writing_.data() + writingOffset_,
                              writing_.size() - writingOffset_, written,
                              overlapped_[kWriteOverlapped].data());
        const IoStatus s = classifyIo(ok, pipe_->lastError());
        switch (s)
        {
        case IoStatus::Ok:
            writingOffset_ += written;
            if (writingOffset_ < writing_.size())
                continue;
            writing_.clear();
            writingOffset_ = 0;
            break;
        case IoStatus::Pending:
            writeInFlight_ = true;
            return;
        case IoStatus::Closed:
            dropPipe();
            return;
        case IoStatus::Failed:
            lastError_ = std::to_string(pipe_->lastError());
            dropPipe();
            return;
        }
    }
}

std::size_t MessageChannel::drainPipe()
{
    std::size_t totalRead = 0;
    for (;;)
    {
        if (readInFlight_)
        {
            std::size_t read = 0;
            const bool ok = pipe_->completeRead(pipeHandle_, read, overlapped_[kReadOverlapped].data());
            const IoStatus s = classifyIo(ok, pipe_->lastError());
            switch (s)
            {
            case IoStatus::Ok:
                readInFlight_ = false;
                if (read == 0)
                    return totalRead;
                inbound_.insert(inbound_.end(), readBuffer_.data(),
                                readBuffer_.data() + read);
                totalRead += read;
                break;
            case IoStatus::Pending:
                return totalRead;
            case IoStatus::Closed:
                dropPipe();
                return totalRead;
            case IoStatus::Failed:
                lastError_ = std::to_string(pipe_->lastError());
                dropPipe();
                return totalRead;
            }
        }

        std::size_t read = 0;
        const bool ok =
            pipe_->startRead(pipeHandle_, readBuffer_.data(), readBuffer_.size(), read,
                             overlapped_[kReadOverlapped].data());
        const IoStatus s = classifyIo(ok, pipe_->lastError());
        switch (s)
        {
        case IoStatus::Ok:
            if (read == 0)
                return totalRead;
            inbound_.insert(inbound_.end(), readBuffer_.data(),
                            readBuffer_.data() + read);
            totalRead += read;
            break;
        case IoStatus::Pending:
            readInFlight_ = true;
            return totalRead;
        case IoStatus::Closed:
            dropPipe();
            return totalRead;
        case IoStatus::Failed:
            lastError_ = std::to_string(pipe_->lastError());
            dropPipe();
            return totalRead;
        }
    }
}

bool MessageChannel::parseFrames(std::vector<Message>& out)
{
    std::size_t consumed = 0;
    while (inbound_.size() - consumed >= sizeof(FrameHeader))
    {
        FrameHeader header;
        std::memcpy(&header, inbound_.data() + consumed, sizeof(FrameHeader));
        const std::uint32_t payloadLength = header.payloadLength;
        const MessageType type = header.type;

        if (payloadLength > kMaxPayloadBytes)
            return false;

        if (inbound_.size() - consumed < sizeof(FrameHeader) + payloadLength)
            break;

        const bool known = type == MessageType::DevicePose;
        if (known)
        {
            const std::uint8_t* payload = inbound_.data() + consumed + sizeof(FrameHeader);
            out.push_back(Message{type, std::vector<std::uint8_t>(payload, payload + payloadLength)});
        }
        consumed += sizeof(FrameHeader) + payloadLength;
    }

    if (consumed)
        inbound_.erase(inbound_.begin(), inbound_.begin() + consumed);
    return true;
}

bool MessageChannel::send(MessageType type, const std::uint8_t* payload, std::size_t size)
{
    if (!pipe_ || connectState_ != ConnectState::Connected)
        return false;
    if (size > kMaxPayloadBytes)
        return false;

    const std::size_t frameSize = sizeof(FrameHeader) + size;
    const std::size_t pending =
        (outbound_.size()) + (writing_.size() - writingOffset_);
    if (pending + frameSize > kMaxPayloadBytes)
        return false;

    FrameHeader header;
    header.payloadLength = static_cast<std::uint32_t>(size);
    header.type = type;
    const auto* headerBytes = reinterpret_cast<const std::uint8_t*>(&header);
    appendOutbound(headerBytes, sizeof(FrameHeader));
    if (size)
        appendOutbound(payload, size);

    pumpWrite();
    return true;
}

std::size_t MessageChannel::receive(std::vector<Message>& out)
{
    if (!pipe_)
    {
        pipe_ = factory_();
        if (!pipe_)
            return 0;

        pipeHandle_ = pipe_->createPipe(kDriverPipeName, pipeBufferSize, pipeBufferSize);
        if (!pipeHandle_ || pipeHandle_ == invalidHandle)
        {
            lastError_ = std::to_string(pipe_->lastError());
            pipe_.reset();
            pipeHandle_ = nullptr;
            return 0;
        }

        // Size the overlapped buffers — the implementation dictates the size.
        const std::size_t ovSize = pipe_->overlappedSize();
        for (int i = 0; i < kOverlappedCount; ++i)
            overlapped_[i].resize(ovSize);

        // Create events for each overlapped buffer. On failure, de-allocate
        // only the events that were created, then drop the pipe.
        for (int i = 0; i < kOverlappedCount; ++i)
        {
            if (!pipe_->createEvent(overlapped_[i].data()))
            {
                lastError_ = std::to_string(pipe_->lastError());
                for (int j = i - 1; j >= 0; --j)
                {
                    pipe_->closeEvent(overlapped_[j].data());
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
        writeInFlight_ = false;
        outbound_.clear();
        writing_.clear();
        writingOffset_ = 0;
        inbound_.clear();
        lastError_.clear();
    }

    pumpConnect();
    if (connectState_ != ConnectState::Connected)
        return 0;

    const std::size_t before = out.size();
    pumpWrite();
    drainPipe();
    if (pipe_ && !parseFrames(out))
    {
        lastError_ = "payload length exceeds maximum";
        dropPipe();
    }
    return out.size() - before;
}
} // namespace link