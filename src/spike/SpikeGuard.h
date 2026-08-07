#pragma once

// Throwaway step-1 spike (doc/driver-plan.md).
//
// Hard rule: no exception may leave a hook or a provider entry point — an exception
// escaping into vrserver.exe kills SteamVR. Every such boundary funnels through this
// one helper instead of spelling out a try/catch, so the swallow is written (and
// tested) once and the detours stay pass-through.

namespace spike
{
template <class Action>
void runGuarded(Action&& action) noexcept
{
    try
    {
        action();
    }
    catch (...)
    {
        // Deliberately silent: the state that made the observer throw is exactly the
        // state in which logging is least trustworthy, and vrserver must not notice.
    }
}
} // namespace spike
