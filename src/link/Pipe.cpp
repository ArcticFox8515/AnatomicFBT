#include "Pipe.h"

#include <cstdint>

namespace link
{
const void* invalidHandle = reinterpret_cast<const void*>(static_cast<std::intptr_t>(-1));

IoStatus classifyIo(int ok, unsigned long lastError)
{
    if (ok)
        return IoStatus::Ok;
    if (lastError == errIoPending || lastError == errIoIncomplete)
        return IoStatus::Pending;
    if (lastError == errPipeConnected)
        return IoStatus::Ok;
    if (lastError == errBrokenPipe || lastError == errPipeNotConnected ||
        lastError == errNoData || lastError == errOperationAborted)
        return IoStatus::Closed;
    return IoStatus::Failed;
}
} // namespace link
