#include "SpikeDriverPipe.h"

#include <windows.h>

namespace link
{
std::size_t Win32Pipe::overlappedSize() const { return sizeof(OVERLAPPED); }

void* Win32Pipe::createPipe(const char* name, std::size_t inBufferSize,
                             std::size_t outBufferSize)
{
    return CreateNamedPipeA(name,
                            PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED,
                            PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT,
                            1, static_cast<DWORD>(outBufferSize),
                            static_cast<DWORD>(inBufferSize), 0, nullptr);
}

bool Win32Pipe::createEvent(void* ov)
{
    return (static_cast<OVERLAPPED*>(ov)->hEvent = CreateEventW(nullptr, TRUE, TRUE, nullptr)) != nullptr;
}

bool Win32Pipe::closeEvent(void* ov)
{
    return CloseHandle(static_cast<OVERLAPPED*>(ov)->hEvent);
}

bool Win32Pipe::startConnect(void* handle, void* ov)
{
    return ConnectNamedPipe(handle, static_cast<OVERLAPPED*>(ov));
}

bool Win32Pipe::completeConnect(void* handle, void* ov)
{
    DWORD n = 0;
    return GetOverlappedResult(handle, static_cast<OVERLAPPED*>(ov), &n, FALSE);
}

bool Win32Pipe::startWrite(void* handle, const std::uint8_t* data, std::size_t size,
                            std::size_t& written, void* ov)
{
    DWORD n = 0;
    const bool r = WriteFile(handle, data, static_cast<DWORD>(size), &n,
                             static_cast<OVERLAPPED*>(ov));
    written = n;
    return r;
}

bool Win32Pipe::completeWrite(void* handle, std::size_t& written, void* ov)
{
    DWORD n = 0;
    const bool r = GetOverlappedResult(handle, static_cast<OVERLAPPED*>(ov), &n, FALSE);
    written = n;
    return r;
}

bool Win32Pipe::startRead(void* handle, std::uint8_t* buffer, std::size_t size,
                           std::size_t& read, void* ov)
{
    DWORD n = 0;
    const bool r = ReadFile(handle, buffer, static_cast<DWORD>(size), &n,
                            static_cast<OVERLAPPED*>(ov));
    read = n;
    return r;
}

bool Win32Pipe::completeRead(void* handle, std::size_t& read, void* ov)
{
    DWORD n = 0;
    const bool r = GetOverlappedResult(handle, static_cast<OVERLAPPED*>(ov), &n, FALSE);
    read = n;
    return r;
}

bool Win32Pipe::close(void* handle)
{
    return CloseHandle(handle);
}

unsigned long Win32Pipe::lastError() const
{
    return GetLastError();
}
} // namespace link