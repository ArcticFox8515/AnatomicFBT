#pragma once

// The decision behind the GetGenericInterface detour, as a pure function so it is
// directly unit-testable (tests/DriverTest.cpp) instead of only reachable through a
// loaded DLL.
//
// The version we build against is a parameter rather than a hardcoded literal: the
// interesting case is precisely "vrserver offers a version other than ours", and a
// test must be able to state both sides.

#include <string>

namespace driver
{
enum class InterfaceAction
{
    HookServerDriverHost,
    // Same interface family, a version we cannot hook. Must be reported loudly: an
    // unhooked version is otherwise a silent no-op — devices behind it are invisible.
    UnsupportedVersion,
    NotNeeded,
};

// "IVRServerDriverHost_006" -> "IVRServerDriverHost_"; a string without '_' is its own family.
inline std::string interfaceFamily(const std::string& version)
{
    const size_t underscore = version.rfind('_');
    return underscore == std::string::npos ? version : version.substr(0, underscore + 1);
}

inline InterfaceAction classifyInterface(const std::string& version,
                                         const std::string& serverDriverHostVersion)
{
    if (version == serverDriverHostVersion)
        return InterfaceAction::HookServerDriverHost;

    if (interfaceFamily(version) == interfaceFamily(serverDriverHostVersion))
        return InterfaceAction::UnsupportedVersion;

    return InterfaceAction::NotNeeded;
}
} // namespace driver
