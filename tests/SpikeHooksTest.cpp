// Tests for the throwaway step-1 spike's vtable detour helper (doc/driver-plan.md).
//
// MinHook is replaced by a fake here, so this file drives the state machine —
// including the failure branches a live SteamVR session would never show us — without
// patching a single byte of code in the test process. That the *real* MinHook lands
// on the vtable slots the plan hardcodes is proved separately, in SpikeDriverTest.

#include "spike/SpikeHooks.h"
#include "spike/SpikeLog.h"

#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace
{
// An object shaped like a COM-style interface: first member is the vtable pointer.
struct FakeInterface
{
    explicit FakeInterface(void** vtable) : vtable_(vtable) {}
    void** vtable_;
};

void slotZero() {}
void slotOne() {}
void slotTwo() {}

void detour() {}

// MinHook's MH_ERROR_ALREADY_INITIALIZED. The value is the real DLL's business; the
// fake only has to report *a* status the way the adapter reports MinHook's.
constexpr int kFakeAlreadyInitialized = 1;

// MinHook's MH_ERROR_ALREADY_CREATED: what the *second* MH_CreateHook on one function
// returns. Only the loser of a concurrent install ever sees it.
constexpr int kFakeAlreadyCreated = 2;

class FakeHookApi : public spike::HookApi
{
public:
    struct Call
    {
        std::string what;
        void* target;
    };

    int initialize() override
    {
        ++initializeCalls;
        return initializeStatus;
    }

    void shutdown() override { ++shutdownCalls; }

    int alreadyInitializedStatus() override { return kFakeAlreadyInitialized; }

    int create(void* target, void* detourFunction, void** original) override
    {
        calls.push_back({"create", target});
        detours.push_back(detourFunction);
        const int status = nextCreateStatus();
        // MH_CreateHook writes the trampoline out-parameter before it returns, so the
        // winner of a race has already published its `original` by the time the loser
        // runs its rollback. Ordering the fake the same way is what makes the loser's
        // damage visible.
        if (status == spike::kHookOk)
            *original = trampoline;
        // The re-entry seam: a second install() lands here, inside the first one's
        // create, which is exactly where a second thread sits — after the `installed()`
        // check and before `target_` is published. One shot, or it recurses forever.
        if (onCreate)
        {
            const std::function<void()> reentry = onCreate;
            onCreate = nullptr;
            reentry();
        }
        return status;
    }

    int enable(void* target) override
    {
        calls.push_back({"enable", target});
        return enableStatus;
    }

    int remove(void* target) override
    {
        calls.push_back({"remove", target});
        return spike::kHookOk;
    }

    const char* statusName(int status) override
    {
        lastStatusNameQuery = status;
        return "MH_ERROR_FAKE";
    }

    std::vector<Call> calls;
    std::vector<void*> detours;
    int createStatus = spike::kHookOk;
    // Per-call statuses, for the cases where the same function is created twice and the
    // two calls must answer differently. Falls back to createStatus when exhausted.
    std::vector<int> createStatusQueue;
    std::function<void()> onCreate;
    int enableStatus = spike::kHookOk;
    int initializeStatus = spike::kHookOk;
    int initializeCalls = 0;
    int shutdownCalls = 0;
    int lastStatusNameQuery = spike::kHookOk;
    void* trampoline = reinterpret_cast<void*>(&slotTwo);

private:
    int nextCreateStatus()
    {
        if (createStatusQueue.empty())
            return createStatus;
        const int status = createStatusQueue.front();
        createStatusQueue.erase(createStatusQueue.begin());
        return status;
    }
};

class SpikeHooksTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        logger_.setSink([this](const char* message) { lines_.emplace_back(message); });
    }

    bool logged(const std::string& needle) const
    {
        for (const std::string& line : lines_)
            if (line.find(needle) != std::string::npos)
                return true;
        return false;
    }

    void* slot(int index) const { return vtable_[index]; }

    void* vtable_[3] = {reinterpret_cast<void*>(&slotZero), reinterpret_cast<void*>(&slotOne),
                        reinterpret_cast<void*>(&slotTwo)};
    FakeInterface object_{vtable_};

    std::vector<std::string> lines_;
    spike::Logger logger_;
    FakeHookApi api_;
};

using VoidHook = spike::VTableHook<void (*)()>;

TEST_F(SpikeHooksTest, InstallPatchesTheFunctionTheRequestedSlotPointsAt)
{
    // The whole point of hooking the *function*, not the vtable: it is shared by every
    // instance of vrserver's concrete class, which is how other drivers' devices become
    // visible to us.
    VoidHook hook(api_, logger_);
    ASSERT_TRUE(hook.install("Fake::SlotOne", &object_, 1, &detour));

    ASSERT_EQ(api_.calls.size(), 2u);
    EXPECT_EQ(api_.calls[0].what, "create");
    EXPECT_EQ(api_.calls[0].target, slot(1));
    EXPECT_EQ(api_.calls[1].what, "enable");
    EXPECT_EQ(api_.calls[1].target, slot(1));
    ASSERT_EQ(api_.detours.size(), 1u);
    EXPECT_EQ(api_.detours[0], reinterpret_cast<void*>(&detour));

    EXPECT_TRUE(hook.installed());
    EXPECT_EQ(reinterpret_cast<void*>(hook.original()), api_.trampoline);
    EXPECT_TRUE(logged("hook Fake::SlotOne: installed (vtable index 1"));
}

TEST_F(SpikeHooksTest, InstallingTwiceIsANoOp)
{
    // vrserver hands the same interface to every driver, and Init hooks eagerly on top
    // of that: a second install must not create a second hook on the same function.
    VoidHook hook(api_, logger_);
    ASSERT_TRUE(hook.install("Fake::SlotZero", &object_, 0, &detour));
    const size_t callsAfterFirst = api_.calls.size();

    EXPECT_TRUE(hook.install("Fake::SlotZero", &object_, 0, &detour));
    EXPECT_EQ(api_.calls.size(), callsAfterFirst);
}

// ---- two callers, one hook object ------------------------------------------------
//
// The sequential double install above is the easy half. The live run showed the other
// half in driver-spike-vrserver.log: the eager install from SpikeServer::init ("hook
// IVRServerDriverHost::TrackedDeviceAdded: installed", 18:25:43.854) and a second
// install driven by another caller's GetGenericInterface ("interface
// \"IVRServerDriverHost_006\": hooking", 18:25:43.958) — two callers on one hook
// object, 104 ms apart, with nothing serializing them.
//
// install() publishes target_ only at its very end, so the second caller does not see
// installed() and walks straight into MH_CreateHook on a function that is already
// hooked. Reproduced without threads: the fake re-enters install() from inside the
// first call's create, which is the exact point a second thread occupies.

TEST_F(SpikeHooksTest, AnInstallReenteredDuringCreateKeepsAnOriginalToCallThrough)
{
    // First caller wins MH_CreateHook and gets the trampoline; the second gets
    // MH_ERROR_ALREADY_CREATED and runs the rollback branch, which clears original_.
    VoidHook hook(api_, logger_);
    api_.createStatusQueue = {spike::kHookOk, kFakeAlreadyCreated};
    api_.onCreate = [&] { hook.install("Fake::SlotZero", &object_, 0, &detour); };

    ASSERT_TRUE(hook.install("Fake::SlotZero", &object_, 0, &detour));

    // The function inside vrserver is patched and enabled either way. What must never
    // survive is a live hook with no original: every detour body starts with
    // `hooks.x.original()(...)`, so this state is a call through null on the next pose
    // update — with vrserver's threads already running through it.
    ASSERT_TRUE(hook.installed());
    EXPECT_NE(hook.original(), nullptr);
}

TEST_F(SpikeHooksTest, AnInstallReenteredDuringCreateHooksTheFunctionOnlyOnce)
{
    // Same interleaving, the case where MinHook would accept both creates: two
    // create/enable pairs on one function chain a hook onto a hook, so every call is
    // observed twice and remove() in Cleanup unwinds only the outer one.
    VoidHook hook(api_, logger_);
    api_.onCreate = [&] { hook.install("Fake::SlotOne", &object_, 1, &detour); };

    ASSERT_TRUE(hook.install("Fake::SlotOne", &object_, 1, &detour));

    ASSERT_EQ(api_.calls.size(), 2u);
    EXPECT_EQ(api_.calls[0].what, "create");
    EXPECT_EQ(api_.calls[1].what, "enable");
}

TEST_F(SpikeHooksTest, NullObjectIsReportedAndNotHooked)
{
    // vrserver returning no interface must never be dereferenced.
    VoidHook hook(api_, logger_);
    EXPECT_FALSE(hook.install("Fake::Missing", nullptr, 0, &detour));
    EXPECT_FALSE(hook.installed());
    EXPECT_EQ(hook.original(), nullptr);
    EXPECT_TRUE(api_.calls.empty());
    EXPECT_TRUE(logged("hook Fake::Missing: NOT installed, null object"));
}

TEST_F(SpikeHooksTest, CreateFailureLeavesNoHookAndNoOriginal)
{
    api_.createStatus = -1;
    VoidHook hook(api_, logger_);
    EXPECT_FALSE(hook.install("Fake::SlotZero", &object_, 0, &detour));

    EXPECT_FALSE(hook.installed());
    EXPECT_EQ(hook.original(), nullptr);
    ASSERT_EQ(api_.calls.size(), 1u); // create only, no enable
    EXPECT_EQ(api_.calls[0].what, "create");
    EXPECT_EQ(api_.lastStatusNameQuery, -1);
    EXPECT_TRUE(logged("hook Fake::SlotZero: MH_CreateHook failed (MH_ERROR_FAKE)"));
}

TEST_F(SpikeHooksTest, EnableFailureRollsTheCreatedHookBack)
{
    // The dangerous case: the hook exists but is not enabled. Leaving it behind would
    // strand a trampoline inside vrserver, and keeping `original` would make the detour
    // callable through a hook that is not installed.
    api_.enableStatus = -1;
    VoidHook hook(api_, logger_);
    EXPECT_FALSE(hook.install("Fake::SlotTwo", &object_, 2, &detour));

    EXPECT_FALSE(hook.installed());
    EXPECT_EQ(hook.original(), nullptr);
    ASSERT_EQ(api_.calls.size(), 3u);
    EXPECT_EQ(api_.calls[1].what, "enable");
    EXPECT_EQ(api_.calls[2].what, "remove");
    EXPECT_EQ(api_.calls[2].target, slot(2));
    EXPECT_TRUE(logged("hook Fake::SlotTwo: MH_EnableHook failed (MH_ERROR_FAKE)"));
}

TEST_F(SpikeHooksTest, RemoveUnhooksOnceAndIsSafeWhenNotInstalled)
{
    VoidHook hook(api_, logger_);
    ASSERT_TRUE(hook.install("Fake::SlotOne", &object_, 1, &detour));
    api_.calls.clear();

    hook.remove();
    ASSERT_EQ(api_.calls.size(), 1u);
    EXPECT_EQ(api_.calls[0].what, "remove");
    EXPECT_EQ(api_.calls[0].target, slot(1));
    EXPECT_FALSE(hook.installed());
    EXPECT_TRUE(logged("hook Fake::SlotOne: removed"));

    // Cleanup runs once per provider, but nothing stops it being reached twice.
    hook.remove();
    EXPECT_EQ(api_.calls.size(), 1u);
}

TEST_F(SpikeHooksTest, ReinstallAfterRemoveWorks)
{
    // vrwatchdog / vrserver can load and unload us repeatedly within one Steam session.
    VoidHook hook(api_, logger_);
    ASSERT_TRUE(hook.install("Fake::SlotZero", &object_, 0, &detour));
    hook.remove();
    EXPECT_TRUE(hook.install("Fake::SlotZero", &object_, 0, &detour));
    EXPECT_TRUE(hook.installed());
    EXPECT_EQ(reinterpret_cast<void*>(hook.original()), api_.trampoline);
}

// ------------------------------------------------- hook library initialization ----

TEST_F(SpikeHooksTest, HookLibraryInitializationSucceeds)
{
    EXPECT_EQ(spike::initializeHookLibrary(api_), nullptr);
    EXPECT_EQ(api_.initializeCalls, 1);
}

TEST_F(SpikeHooksTest, HookLibraryAlreadyInitializedIsNotAnError)
{
    // Another driver in the same vrserver called MH_Initialize before us. Treating that
    // as a failure would abort our Init for no reason.
    api_.initializeStatus = kFakeAlreadyInitialized;
    EXPECT_EQ(spike::initializeHookLibrary(api_), nullptr);
}

TEST_F(SpikeHooksTest, HookLibraryFailureReportsTheStatusText)
{
    // The branch a live SteamVR run essentially never takes.
    api_.initializeStatus = -1;
    EXPECT_STREQ(spike::initializeHookLibrary(api_), "MH_ERROR_FAKE");
    EXPECT_EQ(api_.lastStatusNameQuery, -1);
}
} // namespace
