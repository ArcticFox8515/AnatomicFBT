#pragma once

#include "Log.h"
#include "Pipe.h"
#include "Protocol.h"

#include <format>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace link
{
class MessageChannel
{
public:
    explicit MessageChannel(Logger& logger, PipeFactoryFn factory);

    std::string lastError() const;

    void send(const Message& message);
    std::size_t receive(std::vector<Message>& out);

private:
    enum class ConnectState { Unconnected, Connecting, Connected };

    void pumpConnect();
    void pumpWrite();
    void drainPipe(std::vector<Message>& out);
    void dropPipe();

    void unsafeSend(const Message& message);
    std::size_t unsafeReceive(std::vector<Message>& out);

    std::size_t readEnd() const;
    std::size_t writeEnd() const;

    template <class... Args>
    void queueLog(std::format_string<Args...> fmt, Args&&... args)
    {
        pendingLog_.push_back(std::format(fmt, std::forward<Args>(args)...));
    }

    void flushLog();

    Logger& log_;
    PipeFactoryFn factory_;
    std::shared_ptr<Pipe> pipe_;
    void* pipeHandle_ = nullptr;
    ConnectState connectState_ = ConnectState::Unconnected;

    std::string lastError_;
    std::vector<std::string> pendingLog_;

    std::vector<unsigned char> overlapped_[kOverlappedCount];
    bool eventCreated_[kOverlappedCount] = {};

    Message readMessage_;
    std::size_t readOffset_ = 0;
    bool readInFlight_ = false;

    Message writeMessage_;
    std::size_t writeOffset_ = sizeof(Message);
    bool writeInFlight_ = false;

    mutable std::mutex mutex_;
};
} // namespace link
