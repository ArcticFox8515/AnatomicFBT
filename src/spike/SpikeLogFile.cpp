#include "SpikeLogFile.h"

namespace spike
{
namespace
{
// The Win32 out-parameter dance, once: hand the call a buffer, keep exactly what it
// wrote. A buffer-sized std::string full of NULs would otherwise append 200 zero bytes
// to every path built from it.
template <typename Read>
std::string readIntoBuffer(Read read)
{
    std::string value(kMaxLogPath, '\0');
    const unsigned long written = read(&value[0], kMaxLogPath);
    value.resize(written < kMaxLogPath ? written : kMaxLogPath);
    return value;
}

std::string environmentValue(ProcessApi& api, const char* name)
{
    std::string value(kMaxLogPath, '\0');
    const unsigned long written = api.environmentVariable(name, &value[0], kMaxLogPath);
    // GetEnvironmentVariableA answers 0 when the variable is unset and the *required*
    // size (>= our buffer) when the value did not fit, leaving the buffer untouched in
    // both cases — so a value that does not fit is treated exactly like an unset one
    // rather than read back as uninitialized bytes.
    if (written == 0 || written >= kMaxLogPath)
        return {};
    value.resize(written);
    return value;
}

std::string executableName(ProcessApi& api)
{
    return processNameFromPath(
        readIntoBuffer([&](char* buffer, unsigned long size) { return api.executablePath(buffer, size); }));
}
} // namespace

std::string processNameFromPath(const std::string& executablePath)
{
    std::string name = executablePath;
    if (const size_t slash = name.find_last_of("\\/"); slash != std::string::npos)
        name = name.substr(slash + 1);
    if (const size_t dot = name.find_last_of('.'); dot != std::string::npos)
        name = name.substr(0, dot);
    // "driver-spike-.log" would be one shared file for every process whose executable
    // could not be identified — and the spike's whole point is knowing which process
    // wrote which line.
    if (name.empty())
        return kUnknownProcessName;
    return name;
}

std::string logDirectory(ProcessApi& api)
{
    const std::string localAppData = environmentValue(api, "LOCALAPPDATA");
    // vrserver runs as the logged-in user, so %LOCALAPPDATA% is normally there; a
    // process without it still gets a log, next to its working directory.
    if (localAppData.empty())
        return ".\\";
    return localAppData + "\\TrackingCorrector\\";
}

std::string processLogPath(ProcessApi& api, const char* prefix)
{
    return logDirectory(api) + prefix + executableName(api) + ".log";
}
} // namespace spike
