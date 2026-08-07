// Tests for the throwaway step-1 spike's hook table and detour bodies
// (doc/driver-plan.md, doc/driver-spike-handover.md §2.1).
//
// Everything here used to live in SpikeDriver.cpp, where it was unreachable: that file
// is compiled only into the driver DLL, so the only thing that ever ran it was
// SpikeDriverTest through LoadLibrary — an integration test, and therefore no evidence
// about these decisions at all. What is proved here, with MinHook replaced by a fake and
// no DLL in sight:
//
//   * the vtable index table the plan fixed, and which detour goes into which slot,
//   * that removal happens in reverse install order,
//   * that each detour forwards to the original with its arguments unchanged and returns
//     vrserver's answer unchanged,
//   * the observe-before / observe-after ordering per hook, which is a real decision:
//     TrackedDeviceAdded must be recorded before vrserver can call back into the device
//     driver, while GetGenericInterface can only be observed after it returns a pointer,
//   * that a failed component creation is not recorded (there is no handle, and *handle
//     would be uninitialized memory).

#include "spike/SpikeDriverHooks.h"
#include "spike/SpikeLog.h"

#include <gtest/gtest.h>

#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace
{
// An object shaped like a COM-style interface: first member is the vtable pointer, which
// is all VTableHookBase::install reads.
struct FakeInterface
{
    explicit FakeInterface(void** vtable) : vtable_(vtable) {}
    void** vtable_;
};

void slotZero() {}
void slotOne() {}
void slotTwo() {}

class FakeHookApi : public spike::HookApi
{
public:
    struct Call
    {
        std::string what;
        void* target;
        void* detour;
    };

    int initialize() override { return spike::kHookOk; }
    void shutdown() override {}
    bool isAlreadyInitialized(int) override { return false; }

    int create(void* target, void* detour, void** original) override
    {
        calls.push_back({"create", target, detour});
        *original = trampoline;
        return spike::kHookOk;
    }

    int enable(void* target) override
    {
        calls.push_back({"enable", target, nullptr});
        return spike::kHookOk;
    }

    int remove(void* target) override
    {
        calls.push_back({"remove", target, nullptr});
        return spike::kHookOk;
    }

    const char* statusName(int) override { return "MH_ERROR_FAKE"; }

    std::vector<Call> createdTargets() const
    {
        std::vector<Call> result;
        for (const Call& call : calls)
            if (call.what == "create")
                result.push_back(call);
        return result;
    }

    std::vector<void*> removedTargets() const
    {
        std::vector<void*> result;
        for (const Call& call : calls)
            if (call.what == "remove")
                result.push_back(call.target);
        return result;
    }

    std::vector<Call> calls;
    // What install() hands back as the "original" function. Set per hook so a detour
    // body can be pointed at a stub with the matching signature.
    void* trampoline = nullptr;
};

// ---- the stubs standing in for vrserver's real implementations -------------------
//
// The detour bodies call these through the hook's original() pointer. Everything they
// receive is recorded so "forwarded unchanged" can be asserted, and each snapshots the
// log so the observe-before / observe-after order is visible.

struct StubRecord
{
    int calls = 0;
    std::string logAtCallTime;

    // GetGenericInterface
    vr::IVRDriverContext* context = nullptr;
    std::string version;
    vr::EVRInitError* error = nullptr;

    // TrackedDeviceAdded
    vr::IVRServerDriverHost* host = nullptr;
    std::string serial;
    vr::ETrackedDeviceClass deviceClass = vr::TrackedDeviceClass_Invalid;
    vr::ITrackedDeviceServerDriver* driver = nullptr;

    // TrackedDevicePoseUpdated
    uint32_t poseIndex = 0;
    double posePositionX = 0.0;
    uint32_t poseStructSize = 0;
    const vr::DriverPose_t* poseAddress = nullptr;

    // components
    vr::IVRDriverInput* input = nullptr;
    vr::PropertyContainerHandle_t container = vr::k_ulInvalidPropertyContainer;
    std::string name;
    vr::VRInputComponentHandle_t* handleOut = nullptr;
    vr::VRInputComponentHandle_t updatedHandle = vr::k_ulInvalidInputComponentHandle;
    bool updatedValue = false;
    double timeOffset = 0.0;
    vr::EVRScalarType scalarType = vr::VRScalarType_Absolute;
    vr::EVRScalarUnits scalarUnits = vr::VRScalarUnits_NormalizedOneSided;
};

StubRecord g_stub;
std::ostringstream* g_logStream = nullptr;

// What the stubs return, so "returns vrserver's answer unchanged" is checkable.
void* g_returnedInterface = nullptr;
bool g_returnedAdded = false;
vr::EVRInputError g_returnedInputError = vr::VRInputError_None;
vr::VRInputComponentHandle_t g_createdHandle = 0;

void snapshotLog()
{
    g_stub.logAtCallTime = g_logStream ? g_logStream->str() : std::string();
}

void* stubGetGenericInterface(vr::IVRDriverContext* self, const char* version,
                              vr::EVRInitError* error)
{
    ++g_stub.calls;
    snapshotLog();
    g_stub.context = self;
    g_stub.version = version ? version : "(null)";
    g_stub.error = error;
    return g_returnedInterface;
}

bool stubTrackedDeviceAdded(vr::IVRServerDriverHost* self, const char* serial,
                            vr::ETrackedDeviceClass deviceClass,
                            vr::ITrackedDeviceServerDriver* driver)
{
    ++g_stub.calls;
    snapshotLog();
    g_stub.host = self;
    g_stub.serial = serial ? serial : "(null)";
    g_stub.deviceClass = deviceClass;
    g_stub.driver = driver;
    return g_returnedAdded;
}

void stubTrackedDevicePoseUpdated(vr::IVRServerDriverHost* self, uint32_t index,
                                  const vr::DriverPose_t& pose, uint32_t poseStructSize)
{
    ++g_stub.calls;
    snapshotLog();
    g_stub.host = self;
    g_stub.poseIndex = index;
    g_stub.posePositionX = pose.vecPosition[0];
    g_stub.poseStructSize = poseStructSize;
    g_stub.poseAddress = &pose;
}

vr::EVRInputError stubCreateBooleanComponent(vr::IVRDriverInput* self,
                                             vr::PropertyContainerHandle_t container,
                                             const char* name,
                                             vr::VRInputComponentHandle_t* handle)
{
    ++g_stub.calls;
    snapshotLog();
    g_stub.input = self;
    g_stub.container = container;
    g_stub.name = name ? name : "(null)";
    g_stub.handleOut = handle;
    if (handle)
        *handle = g_createdHandle;
    return g_returnedInputError;
}

vr::EVRInputError stubUpdateBooleanComponent(vr::IVRDriverInput* self,
                                             vr::VRInputComponentHandle_t handle, bool value,
                                             double timeOffset)
{
    ++g_stub.calls;
    snapshotLog();
    g_stub.input = self;
    g_stub.updatedHandle = handle;
    g_stub.updatedValue = value;
    g_stub.timeOffset = timeOffset;
    return g_returnedInputError;
}

vr::EVRInputError stubCreateScalarComponent(vr::IVRDriverInput* self,
                                            vr::PropertyContainerHandle_t container,
                                            const char* name,
                                            vr::VRInputComponentHandle_t* handle,
                                            vr::EVRScalarType type, vr::EVRScalarUnits units)
{
    ++g_stub.calls;
    snapshotLog();
    g_stub.input = self;
    g_stub.container = container;
    g_stub.name = name ? name : "(null)";
    g_stub.handleOut = handle;
    g_stub.scalarType = type;
    g_stub.scalarUnits = units;
    if (handle)
        *handle = g_createdHandle;
    return g_returnedInputError;
}

// The addresses the DLL would pass; only their identity matters here.
void detourA() {}
void detourB() {}
void detourC() {}
void detourD() {}
void detourE() {}
void detourF() {}

class SpikeDriverHooksTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        g_stub = StubRecord{};
        g_logStream = stream_.get();
        g_returnedInterface = nullptr;
        g_returnedAdded = false;
        g_returnedInputError = vr::VRInputError_None;
        g_createdHandle = 0;

        logger_.setTimestampSource([] { return std::string("00:00:00.000"); });
        logger_.setStream(stream_);
    }

    void TearDown() override { g_logStream = nullptr; }

    std::string logText() const { return stream_->str(); }

    bool logged(const std::string& needle) const
    {
        return logText().find(needle) != std::string::npos;
    }

    void* slot(int index) const { return vtable_[index]; }

    // Installs one hook with `stub` as the function vrserver would have run.
    template <typename Hook>
    void installWith(Hook& hook, void* stub, int vtableIndex)
    {
        api_.trampoline = stub;
        ASSERT_TRUE(hook.install("Fake::Hook", &object_, vtableIndex, reinterpret_cast<void*>(&detourA)));
        api_.calls.clear();
    }

    void* vtable_[3] = {reinterpret_cast<void*>(&slotZero), reinterpret_cast<void*>(&slotOne),
                        reinterpret_cast<void*>(&slotTwo)};
    FakeInterface object_{vtable_};

    std::shared_ptr<std::ostringstream> stream_ = std::make_shared<std::ostringstream>();
    spike::Logger logger_;
    FakeHookApi api_;

    spike::DriverDetours detours_{reinterpret_cast<void*>(&detourA),
                                  reinterpret_cast<void*>(&detourB),
                                  reinterpret_cast<void*>(&detourC),
                                  reinterpret_cast<void*>(&detourD),
                                  reinterpret_cast<void*>(&detourE),
                                  reinterpret_cast<void*>(&detourF)};
    spike::DriverHookSet hooks_{api_, logger_, detours_};

    spike::SpikeObserver observer_{logger_, hooks_, [] { return 0.0; }};
};

// ---------------------------------------------------------- the index table ----

TEST_F(SpikeDriverHooksTest, VtableIndicesAreThePlansTable)
{
    // Declaration order in openvr_driver.h, i.e. the ABI. If a future openvr version
    // reorders these interfaces, this is the assertion that has to change — and the
    // reason the numbers are named constants instead of literals inside the DLL.
    EXPECT_EQ(spike::kDriverContextGetGenericInterfaceIndex, 0);
    EXPECT_EQ(spike::kServerDriverHostTrackedDeviceAddedIndex, 0);
    EXPECT_EQ(spike::kServerDriverHostTrackedDevicePoseUpdatedIndex, 1);
    EXPECT_EQ(spike::kDriverInputCreateBooleanComponentIndex, 0);
    EXPECT_EQ(spike::kDriverInputUpdateBooleanComponentIndex, 1);
    EXPECT_EQ(spike::kDriverInputCreateScalarComponentIndex, 2);
}

TEST_F(SpikeDriverHooksTest, DriverContextHookGoesIntoSlotZeroWithTheContextDetour)
{
    hooks_.hookDriverContext(reinterpret_cast<vr::IVRDriverContext*>(&object_));

    const std::vector<FakeHookApi::Call> created = api_.createdTargets();
    ASSERT_EQ(created.size(), 1u);
    EXPECT_EQ(created[0].target, slot(0));
    EXPECT_EQ(created[0].detour, detours_.getGenericInterface);
    EXPECT_TRUE(logged(std::string("hook ") + spike::kGetGenericInterfaceHookName + ": installed"));
}

TEST_F(SpikeDriverHooksTest, ServerDriverHostHookGoesIntoTheTwoPlannedSlots)
{
    hooks_.hookServerDriverHost(&object_);

    const std::vector<FakeHookApi::Call> created = api_.createdTargets();
    ASSERT_EQ(created.size(), 2u);
    EXPECT_EQ(created[0].target, slot(0));
    EXPECT_EQ(created[0].detour, detours_.trackedDeviceAdded);
    EXPECT_EQ(created[1].target, slot(1));
    EXPECT_EQ(created[1].detour, detours_.poseUpdated);
    EXPECT_TRUE(logged(std::string("hook ") + spike::kTrackedDeviceAddedHookName + ": installed"));
    EXPECT_TRUE(logged(std::string("hook ") + spike::kPoseUpdatedHookName + ": installed"));
}

TEST_F(SpikeDriverHooksTest, DriverInputHookGoesIntoTheThreePlannedSlots)
{
    hooks_.hookDriverInput(&object_);

    const std::vector<FakeHookApi::Call> created = api_.createdTargets();
    ASSERT_EQ(created.size(), 3u);
    EXPECT_EQ(created[0].target, slot(0));
    EXPECT_EQ(created[0].detour, detours_.createBoolean);
    EXPECT_EQ(created[1].target, slot(1));
    EXPECT_EQ(created[1].detour, detours_.updateBoolean);
    EXPECT_EQ(created[2].target, slot(2));
    EXPECT_EQ(created[2].detour, detours_.createScalar);
}

TEST_F(SpikeDriverHooksTest, RemoveAllUnhooksInReverseInstallOrder)
{
    // The context hook is what discovers the interfaces, so it must be the last one
    // removed: a GetGenericInterface call in flight may still install an interface hook.
    hooks_.hookDriverContext(reinterpret_cast<vr::IVRDriverContext*>(&object_));
    hooks_.hookServerDriverHost(&object_);
    hooks_.hookDriverInput(&object_);
    api_.calls.clear();

    hooks_.removeAll();

    // Slot identity is all the fake object can distinguish, so the order is checked
    // through the log labels, which name the hook.
    const std::string text = logText();
    const size_t scalar = text.find(std::string(spike::kCreateScalarHookName) + ": removed");
    const size_t update = text.find(std::string(spike::kUpdateBooleanHookName) + ": removed");
    const size_t create = text.find(std::string(spike::kCreateBooleanHookName) + ": removed");
    const size_t pose = text.find(std::string(spike::kPoseUpdatedHookName) + ": removed");
    const size_t added = text.find(std::string(spike::kTrackedDeviceAddedHookName) + ": removed");
    const size_t context =
        text.find(std::string(spike::kGetGenericInterfaceHookName) + ": removed");

    ASSERT_NE(context, std::string::npos);
    EXPECT_LT(scalar, update);
    EXPECT_LT(update, create);
    EXPECT_LT(create, pose);
    EXPECT_LT(pose, added);
    EXPECT_LT(added, context);
    EXPECT_EQ(api_.removedTargets().size(), 6u);
}

TEST_F(SpikeDriverHooksTest, InstallingAnInterfaceTwiceDoesNotHookItTwice)
{
    // vrserver hands the same IVRDriverInput to every driver, and the spike installs
    // both eagerly (from Init) and lazily (from the context detour).
    hooks_.hookDriverInput(&object_);
    api_.calls.clear();

    hooks_.hookDriverInput(&object_);
    EXPECT_TRUE(api_.calls.empty());
}

// ------------------------------------------------------- the detour bodies ----

TEST_F(SpikeDriverHooksTest, GetGenericInterfaceForwardsUnchangedAndObservesAfterwards)
{
    installWith(hooks_.getGenericInterface, reinterpret_cast<void*>(&stubGetGenericInterface), 0);
    int interfaceObject = 0;
    g_returnedInterface = &interfaceObject;

    vr::IVRDriverContext* const context = reinterpret_cast<vr::IVRDriverContext*>(&object_);
    vr::EVRInitError error = vr::VRInitError_None;
    void* const result =
        spike::observeGetGenericInterface(hooks_, observer_, context, "IVRSettings_007", &error);

    EXPECT_EQ(result, &interfaceObject);
    EXPECT_EQ(g_stub.calls, 1);
    EXPECT_EQ(g_stub.context, context);
    EXPECT_EQ(g_stub.version, "IVRSettings_007");
    EXPECT_EQ(g_stub.error, &error);

    // Observed only after the call: the pointer being observed is the return value.
    EXPECT_EQ(g_stub.logAtCallTime.find("interface \"IVRSettings_007\""), std::string::npos);
    EXPECT_TRUE(logged("interface \"IVRSettings_007\""));
}

TEST_F(SpikeDriverHooksTest, TrackedDeviceAddedIsObservedBeforeItIsForwarded)
{
    // vrserver can call back into the device driver (activation, component creation)
    // before TrackedDeviceAdded returns, and those callbacks must find the device
    // already recorded.
    installWith(hooks_.trackedDeviceAdded, reinterpret_cast<void*>(&stubTrackedDeviceAdded), 0);
    g_returnedAdded = true;

    vr::IVRServerDriverHost* const host = reinterpret_cast<vr::IVRServerDriverHost*>(&object_);
    vr::ITrackedDeviceServerDriver* const driver =
        reinterpret_cast<vr::ITrackedDeviceServerDriver*>(&api_);

    EXPECT_TRUE(spike::observeTrackedDeviceAdded(hooks_, observer_, host, "LHR-TEST",
                                                 vr::TrackedDeviceClass_GenericTracker, driver));

    EXPECT_EQ(g_stub.calls, 1);
    EXPECT_EQ(g_stub.host, host);
    EXPECT_EQ(g_stub.serial, "LHR-TEST");
    EXPECT_EQ(g_stub.deviceClass, vr::TrackedDeviceClass_GenericTracker);
    EXPECT_EQ(g_stub.driver, driver);
    EXPECT_NE(g_stub.logAtCallTime.find("TrackedDeviceAdded: serial=\"LHR-TEST\""),
              std::string::npos);
}

TEST_F(SpikeDriverHooksTest, FalseFromVrserverIsReturnedUnchanged)
{
    installWith(hooks_.trackedDeviceAdded, reinterpret_cast<void*>(&stubTrackedDeviceAdded), 0);
    g_returnedAdded = false;

    EXPECT_FALSE(spike::observeTrackedDeviceAdded(
        hooks_, observer_, reinterpret_cast<vr::IVRServerDriverHost*>(&object_), "LHR-TEST",
        vr::TrackedDeviceClass_Controller, nullptr));
}

TEST_F(SpikeDriverHooksTest, PoseUpdateIsForwardedByReferenceAndUnmodified)
{
    // The spike must not perturb the driver it watches: same object, same values, same
    // struct size.
    installWith(hooks_.poseUpdated, reinterpret_cast<void*>(&stubTrackedDevicePoseUpdated), 1);

    vr::DriverPose_t pose{};
    pose.vecPosition[0] = 1.25;
    pose.poseIsValid = true;
    pose.deviceIsConnected = true;
    pose.qRotation = {1.0, 0.0, 0.0, 0.0};
    pose.qWorldFromDriverRotation = {1.0, 0.0, 0.0, 0.0};
    pose.qDriverFromHeadRotation = {1.0, 0.0, 0.0, 0.0};

    spike::observeTrackedDevicePoseUpdated(hooks_, observer_,
                                           reinterpret_cast<vr::IVRServerDriverHost*>(&object_), 7,
                                           pose, sizeof(vr::DriverPose_t));

    EXPECT_EQ(g_stub.calls, 1);
    EXPECT_EQ(g_stub.poseIndex, 7u);
    EXPECT_EQ(g_stub.poseStructSize, sizeof(vr::DriverPose_t));
    EXPECT_EQ(g_stub.poseAddress, &pose);
    EXPECT_DOUBLE_EQ(g_stub.posePositionX, 1.25);
    EXPECT_DOUBLE_EQ(pose.vecPosition[0], 1.25);
    EXPECT_NE(g_stub.logAtCallTime.find("first pose update from device 7"), std::string::npos);
}

TEST_F(SpikeDriverHooksTest, BooleanComponentCreationIsForwardedAndRecordedAfterwards)
{
    installWith(hooks_.createBoolean, reinterpret_cast<void*>(&stubCreateBooleanComponent), 0);
    g_createdHandle = 4242;

    vr::VRInputComponentHandle_t handle = 0;
    vr::IVRDriverInput* const input = reinterpret_cast<vr::IVRDriverInput*>(&object_);
    EXPECT_EQ(spike::observeCreateBooleanComponent(hooks_, observer_, input, 500,
                                                   "/input/trigger/click", &handle),
              vr::VRInputError_None);

    EXPECT_EQ(g_stub.calls, 1);
    EXPECT_EQ(g_stub.input, input);
    EXPECT_EQ(g_stub.container, 500u);
    EXPECT_EQ(g_stub.name, "/input/trigger/click");
    EXPECT_EQ(g_stub.handleOut, &handle);
    EXPECT_EQ(handle, 4242u);

    // Observed after the call — the handle only exists once vrserver has written it.
    EXPECT_EQ(g_stub.logAtCallTime.find("CreateBooleanComponent"), std::string::npos);
    EXPECT_TRUE(logged("CreateBooleanComponent: container=500 handle=4242 "
                       "name=\"/input/trigger/click\" <-- trigger click"));
}

TEST_F(SpikeDriverHooksTest, FailedBooleanComponentCreationIsForwardedButNotRecorded)
{
    // There is no handle to attribute later updates to, and *handle would be
    // uninitialized memory.
    installWith(hooks_.createBoolean, reinterpret_cast<void*>(&stubCreateBooleanComponent), 0);
    g_returnedInputError = vr::VRInputError_InvalidHandle;

    vr::VRInputComponentHandle_t handle = 0;
    EXPECT_EQ(spike::observeCreateBooleanComponent(
                  hooks_, observer_, reinterpret_cast<vr::IVRDriverInput*>(&object_), 500,
                  "/input/trigger/click", &handle),
              vr::VRInputError_InvalidHandle);

    EXPECT_EQ(g_stub.calls, 1);
    EXPECT_FALSE(logged("CreateBooleanComponent"));
}

TEST_F(SpikeDriverHooksTest, BooleanComponentCreationWithoutAHandlePointerIsNotRecorded)
{
    installWith(hooks_.createBoolean, reinterpret_cast<void*>(&stubCreateBooleanComponent), 0);

    EXPECT_EQ(spike::observeCreateBooleanComponent(
                  hooks_, observer_, reinterpret_cast<vr::IVRDriverInput*>(&object_), 500,
                  "/input/trigger/click", nullptr),
              vr::VRInputError_None);

    EXPECT_EQ(g_stub.calls, 1);
    EXPECT_FALSE(logged("CreateBooleanComponent"));
}

TEST_F(SpikeDriverHooksTest, BooleanUpdateIsObservedBeforeForwardingAndReachesTheComponent)
{
    // The two hooks together are what produces a trigger edge: creation registers the
    // handle, the update resolves against it.
    installWith(hooks_.createBoolean, reinterpret_cast<void*>(&stubCreateBooleanComponent), 0);
    g_createdHandle = 77;
    vr::VRInputComponentHandle_t handle = 0;
    ASSERT_EQ(spike::observeCreateBooleanComponent(
                  hooks_, observer_, reinterpret_cast<vr::IVRDriverInput*>(&object_), 500,
                  "/input/trigger/click", &handle),
              vr::VRInputError_None);

    installWith(hooks_.updateBoolean, reinterpret_cast<void*>(&stubUpdateBooleanComponent), 1);
    vr::IVRDriverInput* const input = reinterpret_cast<vr::IVRDriverInput*>(&object_);
    EXPECT_EQ(spike::observeUpdateBooleanComponent(hooks_, observer_, input, 77, true, 0.5),
              vr::VRInputError_None);

    EXPECT_EQ(g_stub.input, input);
    EXPECT_EQ(g_stub.updatedHandle, 77u);
    EXPECT_TRUE(g_stub.updatedValue);
    EXPECT_DOUBLE_EQ(g_stub.timeOffset, 0.5);
    EXPECT_NE(g_stub.logAtCallTime.find("trigger DOWN"), std::string::npos);
}

TEST_F(SpikeDriverHooksTest, ScalarComponentCreationForwardsTypeAndUnitsUnchanged)
{
    installWith(hooks_.createScalar, reinterpret_cast<void*>(&stubCreateScalarComponent), 2);
    g_createdHandle = 9;

    vr::VRInputComponentHandle_t handle = 0;
    EXPECT_EQ(spike::observeCreateScalarComponent(
                  hooks_, observer_, reinterpret_cast<vr::IVRDriverInput*>(&object_), 600,
                  "/input/trigger/value", &handle, vr::VRScalarType_Absolute,
                  vr::VRScalarUnits_NormalizedOneSided),
              vr::VRInputError_None);

    EXPECT_EQ(g_stub.calls, 1);
    EXPECT_EQ(g_stub.container, 600u);
    EXPECT_EQ(g_stub.name, "/input/trigger/value");
    EXPECT_EQ(g_stub.scalarType, vr::VRScalarType_Absolute);
    EXPECT_EQ(g_stub.scalarUnits, vr::VRScalarUnits_NormalizedOneSided);
    EXPECT_TRUE(logged("CreateScalarComponent: container=600 handle=9 "
                       "name=\"/input/trigger/value\""));
}

TEST_F(SpikeDriverHooksTest, FailedScalarComponentCreationIsNotRecorded)
{
    installWith(hooks_.createScalar, reinterpret_cast<void*>(&stubCreateScalarComponent), 2);
    g_returnedInputError = vr::VRInputError_InvalidHandle;

    vr::VRInputComponentHandle_t handle = 0;
    EXPECT_EQ(spike::observeCreateScalarComponent(
                  hooks_, observer_, reinterpret_cast<vr::IVRDriverInput*>(&object_), 600,
                  "/input/trigger/value", &handle, vr::VRScalarType_Absolute,
                  vr::VRScalarUnits_NormalizedOneSided),
              vr::VRInputError_InvalidHandle);

    EXPECT_FALSE(logged("CreateScalarComponent"));
}
} // namespace
