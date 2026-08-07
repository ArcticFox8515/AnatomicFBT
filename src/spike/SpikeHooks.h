#pragma once

// Throwaway step-1 spike (doc/driver-plan.md): vtable detour helper.
//
// MinHook itself sits behind the `HookApi` seam: the DLL supplies the real
// implementation (SpikeDriver.cpp), the tests a fake, so the install/remove state
// machine — including every failure branch — runs in a unit test without patching
// code inside the test process.

#include "SpikeLog.h"

namespace spike
{
// MH_OK. Kept as a plain int so this header needs no MinHook.
constexpr int kHookOk = 0;

class HookApi
{
public:
    virtual ~HookApi() = default;

    // MH_Initialize / MH_Uninitialize. Split from the classification below so the
    // adapter has no branch: "is this status still a success" is a decision, and
    // decisions live in initializeHookLibrary().
    virtual int initialize() = 0;
    virtual void shutdown() = 0;
    // MH_ERROR_ALREADY_INITIALIZED, which happens whenever a second driver in the
    // same vrserver got there first and is not an error for us.
    virtual bool isAlreadyInitialized(int status) = 0;

    virtual int create(void* target, void* detour, void** original) = 0;
    virtual int enable(void* target) = 0;
    virtual int remove(void* target) = 0;
    virtual const char* statusName(int status) = 0;
};

// nullptr on success, else the status text for the log. This is ServerEnvironment::
// initHookLibrary()'s entire body, here instead of in the DLL so its failure path —
// which a live SteamVR run essentially never takes — runs in a unit test.
const char* initializeHookLibrary(HookApi& api);

// Patches the function a vtable slot points to (MinHook trampolines the body, the
// vtable itself is untouched), so every instance of the interface's concrete class
// inside vrserver is intercepted — including the interfaces other drivers hold.
class VTableHookBase
{
public:
    VTableHookBase(HookApi& api, Logger& logger) : api_(api), log_(logger) {}

    bool install(const char* name, void* object, int vtableIndex, void* detour);
    void remove();

    bool installed() const { return target_ != nullptr; }

protected:
    void* originalFunction() const { return original_; }

private:
    HookApi& api_;
    Logger& log_;
    void* target_ = nullptr;
    void* original_ = nullptr;
    const char* name_ = "";
    // Reentrancy guard: a second install() on this hook object that lands while the
    // first is still inside MH_CreateHook (the live double-install race, §5.2) must
    // not walk into a second create. Serialized via the hook object, not the version
    // string: vrserver hands the same interface to every driver, and Init hooks
    // eagerly on top of that — two callers, one hook object.
    bool installing_ = false;
};

// Typed façade: the detours need the original as a callable function pointer.
template <typename FuncPtr>
class VTableHook : public VTableHookBase
{
public:
    using VTableHookBase::VTableHookBase;

    FuncPtr original() const { return reinterpret_cast<FuncPtr>(originalFunction()); }
};
} // namespace spike
