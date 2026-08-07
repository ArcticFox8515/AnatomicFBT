// Tests for the throwaway step-1 spike's vtable detour helper (doc/driver-plan.md).
//
// MinHook is replaced by a fake here, so this file drives the state machine —
// including the failure branches a live SteamVR session would never show us — without
// patching a single byte of code in the test process. That the *real* MinHook lands
// on the vtable slots the plan hardcodes is proved separately, in SpikeDriverTest.

#include "spike/SpikeHooks.h"
#include "spike/SpikeLog.h"

#include <gtest/gtest.h>

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
// fake only has to answer the same question the adapter answers with one comparison.
constexpr int kFakeAlreadyInitialized = 1;

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

    bool isAlreadyInitialized(int status) override { return status == kFakeAlreadyInitialized; }

    int create(void* target, void* detourFunction, void** original) override
    {
        calls.push_back({"create", target});
        detours.push_back(detourFunction);
        if (createStatus == spike::kHookOk)
            *original = trampoline;
        return createStatus;
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
    int enableStatus = spike::kHookOk;
    int initializeStatus = spike::kHookOk;
    int initializeCalls = 0;
    int shutdownCalls = 0;
    int lastStatusNameQuery = spike::kHookOk;
    void* trampoline = reinterpret_cast<void*>(&slotTwo);
};

class SpikeHooksTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        logger_.setTimestampSource([] { return std::string("00:00:00.000"); });
        logger_.setStream(stream_);
    }

    bool logged(const std::string& needle) const
    {
        return stream_->str().find(needle) != std::string::npos;
    }

    void* slot(int index) const { return vtable_[index]; }

    void* vtable_[3] = {reinterpret_cast<void*>(&slotZero), reinterpret_cast<void*>(&slotOne),
                        reinterpret_cast<void*>(&slotTwo)};
    FakeInterface object_{vtable_};

    std::shared_ptr<std::ostringstream> stream_ = std::make_shared<std::ostringstream>();
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
