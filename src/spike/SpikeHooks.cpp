#include "SpikeHooks.h"

namespace spike
{
const char* initializeHookLibrary(HookApi& api)
{
    const int status = api.initialize();
    if (status == kHookOk || api.isAlreadyInitialized(status))
        return nullptr;
    return api.statusName(status);
}

bool VTableHookBase::install(const char* name, void* object, int vtableIndex, void* detour)
{
    if (installed())
        return true;
    if (!object)
    {
        log_.logf("hook %s: NOT installed, null object", name);
        return false;
    }

    void** vtable = *reinterpret_cast<void***>(object);
    void* target = vtable[vtableIndex];

    int status = api_.create(target, detour, &original_);
    if (status != kHookOk)
    {
        log_.logf("hook %s: MH_CreateHook failed (%s)", name, api_.statusName(status));
        original_ = nullptr;
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
