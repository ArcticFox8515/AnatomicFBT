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
    MessageType type = MessageType::DeviceMetadata;
};
} // namespace

MessageChannel::MessageChannel(Pipe& pipe) : pipe_(pipe) {}

MessageChannel::State MessageChannel::state() const
{
    return state_;
}

std::string MessageChannel::lastError() const
{
    return lastError_;
}

void MessageChannel::appendOutbound(const std::uint8_t* data, std::size_t size)
{
    outbound_.insert(outbound_.end(), data, data + size);
}

bool MessageChannel::send(MessageType type, const std::uint8_t* payload, std::size_t size)
{
    if (state_ != State::Open)
        return false;
    if (size > kMaxPayloadBytes)
        return false;

    // Refuse if the frame would overflow the outbound cap. What to drop then
    // is the publisher's policy (step 4), not this layer's.
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

    // Write from the front: old backlog + the new frame together. The pipe is
    // non-blocking, so a `Pending` here just leaves the tail buffered; the next
    // `send` retries from `outboundOffset_`. No separate drain trigger — if the
    // pipe is stuck, retrying synchronously would return `Pending` again, and
    // if there is nothing new to send there is nothing to retry into.
    while (outboundOffset_ < outbound_.size())
    {
        std::size_t written = 0;
        const std::size_t remaining = outbound_.size() - outboundOffset_;
        const IoStatus status = pipe_.write(outbound_.data() + outboundOffset_, remaining,
                                            written);
        switch (status)
        {
        case IoStatus::Ok:
            outboundOffset_ += written;
            break;
        case IoStatus::Pending:
            return true;
        case IoStatus::Closed:
            state_ = State::Closed;
            return true;
        case IoStatus::Failed:
            state_ = State::Failed;
            lastError_ = pipe_.lastError();
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
        const IoStatus status = pipe_.read(buffer, sizeof(buffer), read);
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
            state_ = State::Closed;
            return totalRead;
        case IoStatus::Failed:
            state_ = State::Failed;
            lastError_ = pipe_.lastError();
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
        {
            state_ = State::Failed;
            lastError_ = "payload length exceeds maximum";
            break;
        }

        if (inbound_.size() - consumed < sizeof(FrameHeader) + payloadLength)
            break;  // not a whole frame yet

        // Unknown types are skipped (consumed, not emitted): that is how an
        // older app survives a newer driver. The known set is the current
        // MessageType enum; a future type is invisible to this build.
        const bool known = type == MessageType::DeviceMetadata || type == MessageType::DevicePose;
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
    if (state_ != State::Open)
        return 0;

    const std::size_t before = out.size();
    drainPipe();
    if (state_ == State::Open)
        parseFrames(out);
    return out.size() - before;
}
} // namespace link