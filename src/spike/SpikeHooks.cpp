#include "SpikeHooks.h"

namespace spike
{
const char* initializeHookLibrary(HookApi& api)
{
    const int status = api.initialize();
    if (status == kHookOk || status == api.alreadyInitializedStatus())
        return nullptr;
    return api.statusName(status);
}

bool VTableHookBase::install(const char* name, void* object, int vtableIndex, void* detour)
{
    if (installed())
        return true;
    // A second caller re-entering install() from inside the first's MH_CreateHook
    // (the race §5.2 reproduces single-threaded via the fake's `onCreate` seam) would
    // otherwise issue a second create on the same function — chaining a hook onto a
    // hook, or losing and clearing the winner's trampoline. Dedupe by hook object: the
    // first caller publishes, the second sees the claim and lets the first win.
    if (installing_)
        return true;
    if (!object)
    {
        log_.logf("hook %s: NOT installed, null object", name);
        return false;
    }

    struct InstallingFlag
    {
        bool& flag;
        explicit InstallingFlag(bool& f) : flag(f) { flag = true; }
        ~InstallingFlag() { flag = false; }
    } guard(installing_);

    void** vtable = *reinterpret_cast<void***>(object);
    void* target = vtable[vtableIndex];

    int status = api_.create(target, detour, &original_);
    if (status != kHookOk)
    {
        log_.logf("hook %s: MH_CreateHook failed (%s)", name, api_.statusName(status));
        // Do NOT clear original_ here: MH_CreateHook publishes the trampoline through
        // the out-param before returning, so a concurrent winner of this race may have
        // already written our `original_` before our failure rolled back. Clearing it
        // would null the trampoline the live hook needs to call through, at 1.6 kHz
        // inside vrserver. A create that failed without ever publishing leaves
        // original_ untouched at its prior value (nullptr for a fresh hook).
        return false;
    }
    status = api_.enable(target);
    if (status != kHookOk)
    {
        log_.logf("hook %s: MH_EnableHook failed (%s)", name, api_.statusName(status));
        api_.remove(target);
        original_ = nullptr;
        return false;
    }

    target_ = target;
    name_ = name;
    log_.logf("hook %s: installed (vtable index %d, target %p)", name, vtableIndex, target);
    return true;
}

void VTableHookBase::remove()
{
    if (!installed())
        return;
    api_.remove(target_);
    log_.logf("hook %s: removed", name_);
    target_ = nullptr;
    original_ = nullptr;
}
} // namespace spike
