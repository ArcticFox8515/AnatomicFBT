#pragma once

#include "Pipe.h"
#include "Protocol.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace link
{
struct Message
{
    MessageType type = MessageType::DevicePose;
    std::vector<std::uint8_t> payload;
};

class MessageChannel
{
public:
    explicit MessageChannel(PipeFactoryFn factory);

    std::string lastError() const;

    bool send(MessageType type, const std::uint8_t* payload, std::size_t size);
    std::size_t receive(std::vector<Message>& out);

    // Test access: the size of the overlapped buffer at `index`.
    std::size_t overlappedBufferSize(int index) const { return overlapped_[index].size(); }

private:
    enum class ConnectState { Unconnected, Connecting, Connected };

    void pumpConnect();
    void pumpWrite();
    std::size_t drainPipe();
    bool parseFrames(std::vector<Message>& out);
    void dropPipe();
    void appendOutbound(const std::uint8_t* data, std::size_t size);

    PipeFactoryFn factory_;
    std::shared_ptr<Pipe> pipe_;
    void* pipeHandle_ = nullptr;
    ConnectState connectState_ = ConnectState::Unconnected;

    std::string lastError_;

    // Overlapped buffers — owned by the channel, sized via pipe->overlappedSize().
    // The pipe is a stateless wrapper; it casts these to OVERLAPPED* in each call.
    std::vector<unsigned char> overlapped_[kOverlappedCount];
    bool eventCreated_[kOverlappedCount] = {};

    // Read side: fixed member buffer (I/O buffer size dictated by the channel).
    std::array<std::uint8_t, 4096> readBuffer_{};
    bool readInFlight_ = false;

    // Write side: append area + stable buffer.
    std::vector<std::uint8_t> outbound_;
    std::vector<std::uint8_t> writing_;
    std::size_t writingOffset_ = 0;
    bool writeInFlight_ = false;

    std::vector<std::uint8_t> inbound_;
};
} // namespace link