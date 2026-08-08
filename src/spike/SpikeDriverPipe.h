#pragma once

#include "link/Pipe.h"

namespace link
{
class Win32Pipe final : public Pipe
{
public:
    std::size_t overlappedSize() const override;
    void* createPipe(const char* name, std::size_t inBufferSize,
                     std::size_t outBufferSize) override;
    bool createEvent(void* overlapped) override;
    bool closeEvent(void* overlapped) override;
    bool startConnect(void* handle, void* overlapped) override;
    bool completeConnect(void* handle, void* overlapped) override;
    bool startWrite(void* handle, const std::uint8_t* data, std::size_t size,
                    std::size_t& written, void* overlapped) override;
    bool completeWrite(void* handle, std::size_t& written, void* overlapped) override;
    bool startRead(void* handle, std::uint8_t* buffer, std::size_t size,
                   std::size_t& read, void* overlapped) override;
    bool completeRead(void* handle, std::size_t& read, void* overlapped) override;
    bool close(void* handle) override;
    unsigned long lastError() const override;
};
} // namespace link