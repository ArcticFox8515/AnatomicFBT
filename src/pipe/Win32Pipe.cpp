#include "Win32Pipe.h"

#include <windows.h>

namespace link
{
std::size_t Win32Pipe::overlappedSize() const { return sizeof(OVERLAPPED); }

bool Win32Pipe::createEvent(std::vector<unsigned char>& overlapped)
{
    return (reinterpret_cast<OVERLAPPED*>(overlapped.data())->hEvent =
                CreateEventW(nullptr, TRUE, TRUE, nullptr)) != nullptr;
}

bool Win32Pipe::closeEvent(std::vector<unsigned char>& overlapped)
{
    return CloseHandle(reinterpret_cast<OVERLAPPED*>(overlapped.data())->hEvent);
}

bool Win32Pipe::startWrite(void* handle, const std::uint8_t* data, std::size_t size,
                            std::size_t& written, std::vector<unsigned char>& overlapped)
{
    DWORD n = 0;
    const bool r = WriteFile(handle, data, static_cast<DWORD>(size), &n,
                             reinterpret_cast<OVERLAPPED*>(overlapped.data()));
    written = n;
    return r;
}

bool Win32Pipe::completeWrite(void* handle, std::size_t& written,
                               std::vector<unsigned char>& overlapped)
{
    DWORD n = 0;
    const bool r = GetOverlappedResult(handle, reinterpret_cast<OVERLAPPED*>(overlapped.data()),
                                       &n, FALSE);
    written = n;
    return r;
}

bool Win32Pipe::startRead(void* handle, std::uint8_t* buffer, std::size_t size,
                           std::size_t& read, std::vector<unsigned char>& overlapped)
{
    DWORD n = 0;
    const bool r = ReadFile(handle, buffer, static_cast<DWORD>(size), &n,
                            reinterpret_cast<OVERLAPPED*>(overlapped.data()));
    read = n;
    return r;
}

bool Win32Pipe::completeRead(void* handle, std::size_t& read,
                              std::vector<unsigned char>& overlapped)
{
    DWORD n = 0;
    const bool r = GetOverlappedResult(handle, reinterpret_cast<OVERLAPPED*>(overlapped.data()),
                                       &n, FALSE);
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

void* Win32ServerPipe::createPipe(std::size_t inBufferSize, std::size_t outBufferSize)
{
    return CreateNamedPipeA(name_.c_str(),
                            PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED,
                            PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT,
                            1, static_cast<DWORD>(outBufferSize),
                            static_cast<DWORD>(inBufferSize), 0, nullptr);
}

bool Win32ServerPipe::startConnect(void* handle, std::vector<unsigned char>& overlapped)
{
    return ConnectNamedPipe(handle, reinterpret_cast<OVERLAPPED*>(overlapped.data()));
}

bool Win32ServerPipe::completeConnect(void* handle, std::vector<unsigned char>& overlapped)
{
    DWORD n = 0;
    return GetOverlappedResult(handle, reinterpret_cast<OVERLAPPED*>(overlapped.data()), &n, FALSE);
}

void* Win32ClientPipe::createPipe(std::size_t, std::size_t)
{
    return CreateFileA(name_.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
                       FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, nullptr);
}

bool Win32ClientPipe::startConnect(void*, std::vector<unsigned char>&)
{
    return true;
}

bool Win32ClientPipe::completeConnect(void*, std::vector<unsigned char>&)
{
    return true;
}
} // namespace link
