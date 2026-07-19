#pragma once

#include <source_location>
#include <stdexcept>
#include <string>

// Exception type for all errors thrown by our own code. what() carries the
// throw site (file:line) via std::source_location, so catch sites just log
// what(). Third-party exceptions (nlohmann, std, ...) describe themselves in
// what(); calls into them are wrapped at the call site (rethrown as Error)
// so the log says which call threw.
class Error : public std::runtime_error
{
public:
    explicit Error(const std::string& message,
                   const std::source_location& location = std::source_location::current())
        : std::runtime_error(message + " (at " + location.file_name() + ":"
                             + std::to_string(location.line()) + ")")
    {
    }
};
