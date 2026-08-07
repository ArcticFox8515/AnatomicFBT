#include "MessageChannel.h"

#include <cstring>

namespace link
{
namespace
{
// Frame header: u32 payloadLength, u16 type — memcpy'd whole, same style as the
// payload PODs in Protocol.h. Natural alignment (no pragma pack); both ends
// are MSVC x64 so the layout matches.
struct FrameHeader
{
    std::uint32_t payloadLength = 0;
    MessageType type = MessageType::DevicePose;
};
} // namespace

MessageChannel::MessageChannel(PipeFactoryFn factory) : factory_(std::move(factory)) {}

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
    pipe_.reset();
    outbound_.clear();
    outboundOffset_ = 0;
    inbound_.clear();
}

bool MessageChannel::send(MessageType type, const std::uint8_t* payload, std::size_t size)
{
    if (!pipe_)
        return false;
    if (size > kMaxPayloadBytes)
        return false;

    // Refuse if the frame would overflow the outbound cap. What to drop then
    // is the publisher's policy (step 3), not this layer's.
    const std::size_t frameSize = sizeof(FrameHeader) + size;
    const std::size_t pending = outbound_.size() - outboundOffset_;
    if (pending + frameSize > kMaxPayloadBytes)
        return false;

    FrameHeader header;
    header.payloadLength = static_cast<std::uint32_t>(size);
    header.type = type;
    const auto* headerBytes = reinterpret_cast<const std::uint8_t*>(&header);
    appendOutbound(headerBytes, sizeof(FrameHeader));
    if (size)
        appendOutbound(payload, size);

    // Snapshot the shared_ptr so a concurrent `receive` drop cannot free the
    // Pipe out from under this write.
    auto pipe = pipe_;
    while (outboundOffset_ < outbound_.size())
    {
        std::size_t written = 0;
        const std::size_t remaining = outbound_.size() - outboundOffset_;
        const IoStatus status = pipe->write(outbound_.data() + outboundOffset_, remaining,
                                            written);
        switch (status)
        {
        case IoStatus::Ok:
            outboundOffset_ += written;
            break;
        case IoStatus::Pending:
            return true;
        case IoStatus::Closed:
            lastError_.clear();
            dropPipe();
            return true;
        case IoStatus::Failed:
            lastError_ = pipe->lastError();
            dropPipe();
            return true;
        }
    }
    // Fully flushed: drop the sent prefix so outbound_ does not grow across a session.
    outbound_.clear();
    outboundOffset_ = 0;
    return true;
}

std::size_t MessageChannel::drainPipe()
{
    std::size_t totalRead = 0;
    std::uint8_t buffer[4096];
    for (;;)
    {
        std::size_t read = 0;
        const IoStatus status = pipe_->read(buffer, sizeof(buffer), read);
        switch (status)
        {
        case IoStatus::Ok:
            if (read == 0)
                return totalRead;
            inbound_.insert(inbound_.end(), buffer, buffer + read);
            totalRead += read;
            break;
        case IoStatus::Pending:
            return totalRead;
        case IoStatus::Closed:
            lastError_.clear();
            dropPipe();
            return totalRead;
        case IoStatus::Failed:
            lastError_ = pipe_->lastError();
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
            return false;  // stream unrecoverable — caller drops the pipe

        if (inbound_.size() - consumed < sizeof(FrameHeader) + payloadLength)
            break;  // not a whole frame yet

        // Unknown types are skipped (consumed, not emitted): that is how an
        // older app survives a newer driver. The known set is the current
        // MessageType enum; a future type is invisible to this build.
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

std::size_t MessageChannel::receive(std::vector<Message>& out)
{
    // Construct a pipe if there is none. The factory returns nullptr when no
    // client is connected yet — retry next frame.
    if (!pipe_)
    {
        pipe_ = factory_();
        if (!pipe_)
            return 0;
        // A new connection: clear any stale error from a previous drop and
        // discard buffered bytes (no snapshot on connect).
        lastError_.clear();
        outbound_.clear();
        outboundOffset_ = 0;
        inbound_.clear();
    }

    if (!pipe_)
        return 0;

    const std::size_t before = out.size();
    drainPipe();
    if (pipe_ && !parseFrames(out))
    {
        lastError_ = "payload length exceeds maximum";
        dropPipe();
    }
    return out.size() - before;
}
} // namespace link
