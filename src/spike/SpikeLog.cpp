#include "SpikeLog.h"

#include <windows.h>

#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <utility>

namespace spike
{
std::string processBaseName(const std::string& modulePath)
{
    if (modulePath.empty())
        return "unknown";
    const size_t slash = modulePath.find_last_of("\\/");
    const std::string name =
        slash == std::string::npos ? modulePath : modulePath.substr(slash + 1);
    const size_t dot = name.find_last_of('.');
    return dot == std::string::npos ? name : name.substr(0, dot);
}

std::string logDirectory(const std::string& localAppData)
{
    if (localAppData.empty())
        return {};
    return localAppData + "\\TrackingCorrector\\";
}

std::string logPath(const LogEnvironment& environment, const std::string& prefix)
{
    return logDirectory(environment.localAppData) + prefix + "-"
           + processBaseName(environment.modulePath) + ".log";
}

std::string formatTimestamp(unsigned hours, unsigned minutes, unsigned seconds,
                            unsigned milliseconds)
{
    char buffer[32] = {};
    std::snprintf(buffer, sizeof(buffer), "%02u:%02u:%02u.%03u", hours, minutes, seconds,
                  milliseconds);
    return buffer;
}

std::string formatLogLine(const std::string& timestamp, const std::string& message)
{
    return "[" + timestamp + "] " + message + "\n";
}

Logger::Logger() : timestamp_(&localTimestamp) {}

bool Logger::setStream(std::shared_ptr<std::ostream> stream, std::string path)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_)
        return true;
    if (!stream || !stream->good())
        return false;
    stream_ = std::move(stream);
    path_ = std::move(path);
    return true;
}

void Logger::setSink(LogSink sink)
{
    std::lock_guard<std::mutex> lock(mutex_);
    sink_ = std::move(sink);
}

void Logger::setTimestampSource(TimestampFn timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    timestamp_ = std::move(timestamp);
}

bool Logger::isOpen() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return stream_ != nullptr;
}

std::string Logger::filePath() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return path_;
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

void Logger::write(const std::string& message)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_)
    {
        *stream_ << formatLogLine(timestamp_(), message);
        stream_->flush();
    }
    if (sink_)
        sink_(message.c_str());
}

void Logger::close()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_)
        stream_->flush();
    stream_.reset();
    sink_ = nullptr;
    path_.clear();
}

// ---- adapter: forwarders only, no decisions (see the header) ----

Logger& log()
{
    static Logger* instance = new Logger();
    return *instance;
}

LogEnvironment currentLogEnvironment()
{
    // Zero-initialized buffers: a failed call leaves an empty string, which is a case
    // logDirectory / processBaseName already handle, so there is nothing to branch on.
    char localAppData[MAX_PATH] = {};
    GetEnvironmentVariableA("LOCALAPPDATA", localAppData, MAX_PATH);
    char modulePath[MAX_PATH] = {};
    GetModuleFileNameA(nullptr, modulePath, MAX_PATH);
    return {localAppData, modulePath};
}

std::string localTimestamp()
{
    SYSTEMTIME t{};
    GetLocalTime(&t);
    return formatTimestamp(t.wHour, t.wMinute, t.wSecond, t.wMilliseconds);
}

bool openProcessLog(Logger& logger, const std::string& prefix)
{
    const std::string path = logPath(currentLogEnvironment(), prefix);
    std::error_code ignored;
    std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ignored);
    return logger.setStream(std::make_shared<std::ofstream>(path, std::ios::app), path);
}
} // namespace spike
