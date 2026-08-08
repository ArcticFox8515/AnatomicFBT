#pragma once

// Win32 named-pipe implementations of `link::Pipe` (doc/messagechannel-rework-plan.md
// §5). One server pipe and one client pipe, each taking its pipe name as a constructor
// argument. The server uses `CreateNamedPipeA` + `ConnectNamedPipe`; the client uses
// `CreateFileA` and answers the connect steps immediately. All overlapped-IO methods
// are shared and cast `overlapped.data()` to `OVERLAPPED*`.

#include "link/Pipe.h"

#include <string>

namespace link
{
class Win32Pipe : public Pipe
{
public:
    std::size_t overlappedSize() const override;
    bool createEvent(std::vector<unsigned char>& overlapped) override;
    bool closeEvent(std::vector<unsigned char>& overlapped) override;
    bool startWrite(void* handle, const std::uint8_t* data, std::size_t size,
                    std::size_t& written, std::vector<unsigned char>& overlapped) override;
    bool completeWrite(void* handle, std::size_t& written,
                       std::vector<unsigned char>& overlapped) override;
    bool startRead(void* handle, std::uint8_t* buffer, std::size_t size,
                   std::size_t& read, std::vector<unsigned char>& overlapped) override;
    bool completeRead(void* handle, std::size_t& read,
                      std::vector<unsigned char>& overlapped) override;
    bool close(void* handle) override;
    unsigned long lastError() const override;

protected:
    explicit Win32Pipe(std::string name) : name_(std::move(name)) {}

    std::string name_;
};

class Win32ServerPipe final : public Win32Pipe
{
public:
    explicit Win32ServerPipe(std::string name) : Win32Pipe(std::move(name)) {}

    void* createPipe(std::size_t inBufferSize, std::size_t outBufferSize) override;
    bool startConnect(void* handle, std::vector<unsigned char>& overlapped) override;
    bool completeConnect(void* handle, std::vector<unsigned char>& overlapped) override;
};

class Win32ClientPipe final : public Win32Pipe
{
public:
    explicit Win32ClientPipe(std::string name) : Win32Pipe(std::move(name)) {}

    void* createPipe(std::size_t inBufferSize, std::size_t outBufferSize) override;
    bool startConnect(void* handle, std::vector<unsigned char>& overlapped) override;
    bool completeConnect(void* handle, std::vector<unsigned char>& overlapped) override;
};
} // namespace link
