#pragma once

// Throwaway step-1 spike (doc/driver-plan.md): the decision behind the
// GetGenericInterface detour, as a pure function so it is directly unit-testable
// (tests/SpikeDriverTest.cpp) instead of only reachable through a loaded DLL.
//
// The versions we build against are parameters rather than hardcoded literals: the
// interesting case is precisely "vrserver offers a version other than ours", and a
// test must be able to state both sides.

#include <string>

namespace spike
{
enum class InterfaceAction
{
    HookServerDriverHost,
    HookDriverInput,
    // Same interface family, a version we cannot hook. Must be reported loudly: an
    // unhooked version is otherwise a silent no-op — devices behind it are invisible.
    UnsupportedVersion,
    NotNeeded,
};

// "IVRDriverInput_003" -> "IVRDriverInput_"; a string without '_' is its own family.
inline std::string interfaceFamily(const std::string& version)
{
    const size_t underscore = version.rfind('_');
    return underscore == std::string::npos ? version : version.substr(0, underscore + 1);
}

inline InterfaceAction classifyInterface(const std::string& version,
                                        const std::string& serverDriverHostVersion,
                                        const std::string& driverInputVersion)
{
    if (version == serverDriverHostVersion)
        return InterfaceAction::HookServerDriverHost;
    if (version == driverInputVersion)
        return InterfaceAction::HookDriverInput;

    const std::string family = interfaceFamily(version);
    if (family == interfaceFamily(serverDriverHostVersion)
        || family == interfaceFamily(driverInputVersion))
        return InterfaceAction::UnsupportedVersion;

    return InterfaceAction::NotNeeded;
}
} // namespace spike
