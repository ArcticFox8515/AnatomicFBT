#include "Log.h"

#include <cstdarg>
#include <cstdio>
#include <utility>

namespace link
{
LogSink compositeSink(LogSink first, LogSink second)
{
    return [first = std::move(first), second = std::move(second)](const char* message) {
        if (first)
            first(message);
        if (second)
            second(message);
    };
}

void Logger::setSink(LogSink sink)
{
    sink_ = std::move(sink);
}

bool Logger::hasSink() const
{
    return static_cast<bool>(sink_);
}

void Logger::logf(const char* format, ...)
{
    char message[4096] = {};
    va_list args;
    va_start(args, format);
    std::vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    write(message);
}

void Logger::write(const char* message)
{
    if (sink_)
        sink_(message);
}

Logger& log()
{
    static Logger* instance = new Logger();
    return *instance;
}

Logger& loggingTo(Logger& logger, LogSink sink)
{
    if (!logger.hasSink())
        logger.setSink(std::move(sink));
    return logger;
}
} // namespace link
