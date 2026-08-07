#pragma once

// Length-prefixed framing and reassembly over a `Pipe` (doc/driver-plan.md
// phase A, step 3). This layer owns the buffer the byte-stream seam (`Pipe`)
// cannot: it frames outbound payloads and reassembles inbound frames across
// whatever partial reads the pipe hands it.
//
// Frame layout (little-endian, memcpy'd POD, same style as `.tcrec`):
//   u32 payloadLength, u16 type, payload[payloadLength]
// A 6-byte header; `payloadLength` is the only thing the framer needs to know
// to split the stream, so an unknown `type` is skipped by payloadLength, not
// rejected — that is how an older app survives a newer driver. A
// `payloadLength > kMaxPayloadBytes` is a protocol error: the stream is
// unrecoverable, so the channel drops the pipe and waits for the next client.
//
// The channel is frame-driven: `send`/`receive` are called from the poll loop
// (the driver's `RunFrame`, the app's main loop). Nothing blocks;
// `IoStatus::Pending` from the pipe is not an error, it just means "try again
// next frame". `send` never blocks on a full pipe — it buffers outbound bytes
// up to `kMaxPayloadBytes` and refuses (returns false) once full; what to drop
// when that happens is the publisher's policy, not this layer's.
//
// The channel owns the pipe it constructs via the factory, calls the factory
// from `receive()` while it has none, and drops a dead pipe (Closed/Failed)
// and constructs the next one itself. A dead client connection is not fatal —
// the channel drops the pipe and waits for the next connect. Per step 3's "no
// snapshot on connect", nothing is buffered before a pipe exists. The pipe
// pointer is held in a `shared_ptr` so a `send` on the hook thread can hold a
// copy that outlives a concurrent `receive` drop.

#include "Pipe.h"
#include "Protocol.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace link
{
// One reassembled frame from the wire.
struct Message
{
    MessageType type = MessageType::DevicePose;
    std::vector<std::uint8_t> payload;
};

class MessageChannel
{
public:
    // The channel constructs pipes via `factory` and drops them on death.
    explicit MessageChannel(PipeFactoryFn factory);

    // The error from the last dropped pipe (empty on clean close). Preserved
    // until the next pipe is constructed.
    std::string lastError() const;

    // Frames `payload` with `type` and pushes it into the outbound buffer,
    // then writes from the front as much as the pipe accepts (old backlog +
    // the new frame together). Returns false when there is no pipe or the
    // outbound buffer is full (caller policy decides what to do). A payload
    // larger than `kMaxPayloadBytes` is refused without being framed.
    bool send(MessageType type, const std::uint8_t* payload, std::size_t size);

    // Constructs a pipe via the factory if there is none, drains the pipe, and
    // returns every fully reassembled frame in `out` (in arrival order). Drops
    // a dead pipe (Closed/Failed or oversize length) and resets for the next
    // client. Returns the number of messages appended. A partial frame is kept
    // for the next call.
    std::size_t receive(std::vector<Message>& out);

private:
    void appendOutbound(const std::uint8_t* data, std::size_t size);
    std::size_t drainPipe();
    bool parseFrames(std::vector<Message>& out);  // false = oversize length
    void dropPipe();

    PipeFactoryFn factory_;
    std::shared_ptr<Pipe> pipe_;

    std::string lastError_;

    std::vector<std::uint8_t> outbound_;       // pending send bytes (front = next to write)
    std::vector<std::uint8_t>::size_type outboundOffset_ = 0;  // first unflushed byte

    std::vector<std::uint8_t> inbound_;  // bytes read but not yet a full frame
};
} // namespace link
