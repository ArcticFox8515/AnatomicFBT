#pragma once

// Length-prefixed framing and reassembly over a `Pipe` (doc/driver-plan.md
// phase A, step 2). This layer owns the buffer the byte-stream seam (`Pipe`)
// cannot: it frames outbound payloads and reassembles inbound frames across
// whatever partial reads the pipe hands it.
//
// Frame layout (little-endian, memcpy'd POD, same style as `.tcrec`):
//   u32 payloadLength, u16 type, payload[payloadLength]
// A 6-byte header; `payloadLength` is the only thing the framer needs to know
// to split the stream, so an unknown `type` is skipped by payloadLength, not
// rejected — that is how an older app survives a newer driver. A
// `payloadLength > kMaxPayloadBytes` is a protocol error: the stream is
// unrecoverable, so the channel fails permanently rather than reading past
// garbage.
//
// The channel is single-threaded and frame-driven: `send`/`flush`/`receive`
// are called from the poll loop (the driver's `RunFrame`, the app's main
// loop). Nothing blocks; `IoStatus::Pending` from the pipe is not an error,
// it just means "try again next frame". `send` never blocks on a full pipe —
// it buffers outbound bytes up to `kMaxPayloadBytes` and refuses (returns
// false) once full; what to drop when that happens is the publisher's policy
// (step 4), not this layer's.

#include "Pipe.h"
#include "Protocol.h"

#include <cstdint>
#include <vector>

namespace link
{
// One reassembled frame from the wire.
struct Message
{
    MessageType type = MessageType::DeviceMetadata;
    std::vector<std::uint8_t> payload;
};

class MessageChannel
{
public:
    explicit MessageChannel(Pipe& pipe);

    // State of the underlying transport + this layer's own fatal conditions.
    enum class State
    {
        Open,    // accepting sends and receives
        Closed,  // peer closed the connection cleanly
        Failed,  // pipe error or oversize length; lastError() holds the reason
    };

    State state() const;
    std::string lastError() const;

    // Frames `payload` with `type` and pushes it into the outbound buffer,
    // then writes from the front as much as the pipe accepts (old backlog +
    // the new frame together). Returns false when the outbound buffer is full
    // (caller policy decides what to do) or the channel is not Open. A payload
    // larger than `kMaxPayloadBytes` is refused without being framed.
    bool send(MessageType type, const std::uint8_t* payload, std::size_t size);

    // Drains the pipe and returns every fully reassembled frame in `out`
    // (in arrival order). Returns the number of messages appended. Stops at
    // `Closed`/`Failed`; a partial frame is kept for the next call.
    std::size_t receive(std::vector<Message>& out);

private:
    void appendOutbound(const std::uint8_t* data, std::size_t size);
    std::size_t drainPipe();
    bool parseFrames(std::vector<Message>& out);

    Pipe& pipe_;
    State state_ = State::Open;
    std::string lastError_;

    std::vector<std::uint8_t> outbound_;       // pending send bytes (front = next to write)
    std::vector<std::uint8_t>::size_type outboundOffset_ = 0;  // first unflushed byte

    std::vector<std::uint8_t> inbound_;  // bytes read but not yet a full frame
};
} // namespace link