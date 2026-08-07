// Tests for the throwaway step-1 spike (doc/driver-plan.md).
//
// This file is the DLL-boundary *integration* proof, and nothing else: the logic it
// drives is unit-tested against fakes in SpikeObserverTest / SpikeServerTest /
// SpikeHooksTest / SpikeLogTest / SpikeClientReportTest, where every branch is
// reachable. Here the real DLL is loaded and driven through a fake vrserver, because
// three things can only be shown with the real binary in the loop.
//
// What this file can and cannot prove:
//   CAN  — the hook mechanism end to end without SteamVR: the driver DLL loads, its
//          factory serves both providers, MinHook patches the vtable slots the plan
//          hardcodes (GetGenericInterface 0, TrackedDeviceAdded 0,
//          TrackedDevicePoseUpdated 1, Create/UpdateBooleanComponent 0/1,
//          CreateScalarComponent 2), the detour signatures are ABI-correct, every
//          hooked call is forwarded unchanged, device metadata is read back through
//          IVRProperties, boolean components resolve to device ids, trigger edges are
//          detected, and the pose composition is wired from the right DriverPose_t
//          fields. Plus the composition math against independent analytic values.
//   CANNOT — anything only SteamVR knows: which composition formula matches the
//          client-side raw pose, real device classes / cadence / component paths, and
//          whether other drivers' interfaces route through the same vrserver code.
//          That is the live run the spike exists for.

#include "spike/SpikeInterfaces.h"
#include "spike/SpikePoseMath.h"

#include <windows.h>

#include <openvr_driver.h>

#include <gtest/gtest.h>

#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace
{
constexpr vr::PropertyContainerHandle_t kContainerBase = 5000;
constexpr uint32_t kTrackerIndex = 3;

// ------------------------------------------------------------- fake vrserver ----

class FakeDriverLog : public vr::IVRDriverLog
{
public:
    void Log(const char* message) override { lines.emplace_back(message ? message : ""); }

    bool contains(const std::string& needle) const
    {
        for (const std::string& line : lines)
            if (line.find(needle) != std::string::npos)
                return true;
        return false;
    }

    std::vector<std::string> lines;
};

class FakeSettings : public vr::IVRSettings
{
public:
    const char* GetSettingsErrorNameFromEnum(vr::EVRSettingsError) override { return "fake"; }
    void SetBool(const char*, const char*, bool, vr::EVRSettingsError*) override {}
    void SetInt32(const char*, const char*, int32_t, vr::EVRSettingsError*) override {}
    void SetFloat(const char*, const char*, float, vr::EVRSettingsError*) override {}
    void SetString(const char*, const char*, const char*, vr::EVRSettingsError*) override {}
    bool GetBool(const char*, const char*, vr::EVRSettingsError*) override { return false; }
    int32_t GetInt32(const char*, const char*, vr::EVRSettingsError*) override { return 0; }
    float GetFloat(const char*, const char*, vr::EVRSettingsError*) override { return 0.0f; }
    void GetString(const char*, const char*, char* value, uint32_t length,
                   vr::EVRSettingsError*) override
    {
        if (value && length)
            *value = '\0';
    }
    void RemoveSection(const char*, vr::EVRSettingsError*) override {}
    void RemoveKeyInSection(const char*, const char*, vr::EVRSettingsError*) override {}
};

class FakeDriverManager : public vr::IVRDriverManager
{
public:
    uint32_t GetDriverCount() const override { return 1; }
    uint32_t GetDriverName(vr::DriverId_t, char* value, uint32_t length) override
    {
        if (value && length)
            *value = '\0';
        return 1;
    }
    vr::DriverHandle_t GetDriverHandle(const char*) override { return 1; }
    bool IsEnabled(vr::DriverId_t) const override { return true; }
};

class FakeResources : public vr::IVRResources
{
public:
    uint32_t LoadSharedResource(const char*, char*, uint32_t) override { return 0; }
    uint32_t GetResourceFullPath(const char*, const char*, char* value, uint32_t length) override
    {
        if (value && length)
            *value = '\0';
        return 0;
    }
};

struct FakeDevice
{
    std::string serial;
    std::string model;
    std::string trackingSystem;
    int32_t deviceClass = vr::TrackedDeviceClass_Invalid;
    int32_t roleHint = vr::TrackedControllerRole_Invalid;
};

// Containers exist for every index (as in vrserver), but only indices with a device
// answer property reads — that is the "container valid, no device" path the driver
// has to skip over while enumerating.
class FakeProperties : public vr::IVRProperties
{
public:
    void add(uint32_t index, FakeDevice device)
    {
        if (devices.size() <= index)
            devices.resize(index + 1);
        devices[index] = std::move(device);
    }

    vr::ETrackedPropertyError ReadPropertyBatch(vr::PropertyContainerHandle_t container,
                                               vr::PropertyRead_t* batch,
                                               uint32_t count) override
    {
        for (uint32_t i = 0; i < count; ++i)
        {
            vr::PropertyRead_t& read = batch[i];
            read.unTag = 0;
            read.unRequiredBufferSize = 0;

            const FakeDevice* device = find(container);
            if (!device)
            {
                read.eError = vr::TrackedProp_InvalidDevice;
                continue;
            }

            switch (read.prop)
            {
            case vr::Prop_SerialNumber_String: writeString(read, device->serial); break;
            case vr::Prop_ModelNumber_String: writeString(read, device->model); break;
            case vr::Prop_TrackingSystemName_String:
                writeString(read, device->trackingSystem);
                break;
            case vr::Prop_DeviceClass_Int32: writeInt32(read, device->deviceClass); break;
            case vr::Prop_ControllerRoleHint_Int32: writeInt32(read, device->roleHint); break;
            default: read.eError = vr::TrackedProp_UnknownProperty; break;
            }
        }
        return vr::TrackedProp_Success;
    }

    vr::ETrackedPropertyError WritePropertyBatch(vr::PropertyContainerHandle_t,
                                                vr::PropertyWrite_t*, uint32_t) override
    {
        return vr::TrackedProp_Success;
    }

    const char* GetPropErrorNameFromEnum(vr::ETrackedPropertyError) override { return "fake"; }

    vr::PropertyContainerHandle_t TrackedDeviceToPropertyContainer(
        vr::TrackedDeviceIndex_t index) override
    {
        ++containerLookups;
        return kContainerBase + index;
    }

    uint32_t containerLookups = 0;
    std::vector<FakeDevice> devices;

private:
    const FakeDevice* find(vr::PropertyContainerHandle_t container) const
    {
        if (container < kContainerBase)
            return nullptr;
        const size_t index = static_cast<size_t>(container - kContainerBase);
        if (index >= devices.size() || devices[index].serial.empty())
            return nullptr;
        return &devices[index];
    }

    static void writeString(vr::PropertyRead_t& read, const std::string& value)
    {
        read.unTag = vr::k_unStringPropertyTag;
        read.unRequiredBufferSize = static_cast<uint32_t>(value.size() + 1);
        if (!read.pvBuffer || read.unBufferSize < read.unRequiredBufferSize)
        {
            read.eError = vr::TrackedProp_BufferTooSmall;
            return;
        }
        std::memcpy(read.pvBuffer, value.c_str(), value.size() + 1);
        read.eError = vr::TrackedProp_Success;
    }

    static void writeInt32(vr::PropertyRead_t& read, int32_t value)
    {
        read.unTag = vr::k_unInt32PropertyTag;
        read.unRequiredBufferSize = sizeof(int32_t);
        if (!read.pvBuffer || read.unBufferSize < sizeof(int32_t))
        {
            read.eError = vr::TrackedProp_BufferTooSmall;
            return;
        }
        std::memcpy(read.pvBuffer, &value, sizeof(value));
        read.eError = vr::TrackedProp_Success;
    }
};

class FakeServerDriverHost : public vr::IVRServerDriverHost
{
public:
    struct ForwardedPose
    {
        uint32_t index;
        double positionX;
        uint32_t structSize;
    };

    bool TrackedDeviceAdded(const char* serial, vr::ETrackedDeviceClass deviceClass,
                            vr::ITrackedDeviceServerDriver*) override
    {
        addedSerials.emplace_back(serial ? serial : "");
        addedClasses.push_back(deviceClass);
        return true;
    }

    void TrackedDevicePoseUpdated(uint32_t index, const vr::DriverPose_t& pose,
                                  uint32_t structSize) override
    {
        forwardedPoses.push_back({index, pose.vecPosition[0], structSize});
    }

    void VsyncEvent(double) override {}
    void VendorSpecificEvent(uint32_t, vr::EVREventType, const vr::VREvent_Data_t&,
                             double) override
    {
    }
    bool IsExiting() override { return false; }
    bool PollNextEvent(vr::VREvent_t*, uint32_t) override { return false; }
    void GetRawTrackedDevicePoses(float, vr::TrackedDevicePose_t*, uint32_t) override {}
    void RequestRestart(const char*, const char*, const char*, const char*) override {}
    uint32_t GetFrameTimings(vr::Compositor_FrameTiming*, uint32_t) override { return 0; }
    void SetDisplayEyeToHead(uint32_t, const vr::HmdMatrix34_t&, const vr::HmdMatrix34_t&) override
    {
    }
    void SetDisplayProjectionRaw(uint32_t, const vr::HmdRect2_t&, const vr::HmdRect2_t&) override {}
    void SetRecommendedRenderTargetSize(uint32_t, uint32_t, uint32_t) override {}

    std::vector<std::string> addedSerials;
    std::vector<vr::ETrackedDeviceClass> addedClasses;
    std::vector<ForwardedPose> forwardedPoses;
};

class FakeDriverInput : public vr::IVRDriverInput
{
public:
    struct BooleanUpdate
    {
        vr::VRInputComponentHandle_t handle;
        bool value;
    };

    vr::EVRInputError CreateBooleanComponent(vr::PropertyContainerHandle_t container,
                                             const char* name,
                                             vr::VRInputComponentHandle_t* handle) override
    {
        createdBooleanNames.emplace_back(name ? name : "");
        createdBooleanContainers.push_back(container);
        *handle = nextHandle++;
        return vr::VRInputError_None;
    }

    vr::EVRInputError UpdateBooleanComponent(vr::VRInputComponentHandle_t handle, bool value,
                                            double) override
    {
        forwardedBooleanUpdates.push_back({handle, value});
        return vr::VRInputError_None;
    }

    vr::EVRInputError CreateScalarComponent(vr::PropertyContainerHandle_t, const char* name,
                                           vr::VRInputComponentHandle_t* handle,
                                           vr::EVRScalarType, vr::EVRScalarUnits) override
    {
        createdScalarNames.emplace_back(name ? name : "");
        *handle = nextHandle++;
        return vr::VRInputError_None;
    }

    vr::EVRInputError UpdateScalarComponent(vr::VRInputComponentHandle_t, float, double) override
    {
        return vr::VRInputError_None;
    }

    vr::EVRInputError CreateHapticComponent(vr::PropertyContainerHandle_t, const char*,
                                            vr::VRInputComponentHandle_t* handle) override
    {
        *handle = nextHandle++;
        return vr::VRInputError_None;
    }

    vr::EVRInputError CreateSkeletonComponent(vr::PropertyContainerHandle_t, const char*,
                                              const char*, const char*,
                                              vr::EVRSkeletalTrackingLevel,
                                              const vr::VRBoneTransform_t*, uint32_t,
                                              vr::VRInputComponentHandle_t* handle) override
    {
        *handle = nextHandle++;
        return vr::VRInputError_None;
    }

    vr::EVRInputError UpdateSkeletonComponent(vr::VRInputComponentHandle_t,
                                              vr::EVRSkeletalMotionRange,
                                              const vr::VRBoneTransform_t*, uint32_t) override
    {
        return vr::VRInputError_None;
    }

    vr::VRInputComponentHandle_t nextHandle = 100;
    std::vector<std::string> createdBooleanNames;
    std::vector<vr::PropertyContainerHandle_t> createdBooleanContainers;
    std::vector<std::string> createdScalarNames;
    std::vector<BooleanUpdate> forwardedBooleanUpdates;
};

class FakeDriverContext : public vr::IVRDriverContext
{
public:
    void* GetGenericInterface(const char* version, vr::EVRInitError* error) override
    {
        requested.emplace_back(version ? version : "");
        if (error)
            *error = vr::VRInitError_None;

        const std::string name = version ? version : "";
        // Lets a test make vrserver "offer" an arbitrary version (a dummy pointer, only
        // ever used for versions the driver must refuse to hook) or return NULL for one.
        const auto extra = extraInterfaces.find(name);
        if (extra != extraInterfaces.end())
        {
            if (!extra->second && error)
                *error = vr::VRInitError_Init_InterfaceNotFound;
            return extra->second;
        }

        if (name == vr::IVRServerDriverHost_Version)
            return static_cast<vr::IVRServerDriverHost*>(&host);
        if (name == vr::IVRSettings_Version)
            return static_cast<vr::IVRSettings*>(&settings);
        if (name == vr::IVRProperties_Version)
            return static_cast<vr::IVRProperties*>(&properties);
        if (name == vr::IVRDriverLog_Version)
            return static_cast<vr::IVRDriverLog*>(&log);
        if (name == vr::IVRDriverManager_Version)
            return static_cast<vr::IVRDriverManager*>(&manager);
        if (name == vr::IVRResources_Version)
            return static_cast<vr::IVRResources*>(&resources);
        if (name == vr::IVRDriverInput_Version)
            return static_cast<vr::IVRDriverInput*>(&input);

        if (error)
            *error = vr::VRInitError_Init_InterfaceNotFound;
        return nullptr;
    }

    vr::DriverHandle_t GetDriverHandle() override { return 1; }

    bool wasRequested(const std::string& version) const
    {
        for (const std::string& name : requested)
            if (name == version)
                return true;
        return false;
    }

    FakeServerDriverHost host;
    FakeSettings settings;
    FakeProperties properties;
    FakeDriverLog log;
    FakeDriverManager manager;
    FakeResources resources;
    FakeDriverInput input;
    std::vector<std::string> requested;
    std::map<std::string, void*> extraInterfaces;
};

vr::DriverPose_t makeDriverPose()
{
    vr::DriverPose_t pose{};
    pose.poseIsValid = true;
    pose.deviceIsConnected = true;
    pose.result = vr::TrackingResult_Running_OK;
    pose.poseTimeOffset = -0.011;

    // Values chosen to be exactly representable, so the driver's Release-built math and
    // the test's Debug-built math must agree bit for bit.
    pose.qWorldFromDriverRotation = {0.0, 0.0, 1.0, 0.0}; // 180 deg about Y
    pose.vecWorldFromDriverTranslation[0] = 10.0;
    pose.vecPosition[0] = 1.0;
    pose.vecPosition[1] = 2.0;
    pose.vecPosition[2] = 3.0;
    pose.qRotation = {0.0, 0.0, 1.0, 0.0}; // 180 deg about Y
    pose.qDriverFromHeadRotation = {1.0, 0.0, 0.0, 0.0};
    pose.vecDriverFromHeadTranslation[1] = 0.5;
    return pose;
}

spike::RigidPose worldFromDriverOf(const vr::DriverPose_t& pose)
{
    return {{pose.vecWorldFromDriverTranslation[0], pose.vecWorldFromDriverTranslation[1],
             pose.vecWorldFromDriverTranslation[2]},
            {pose.qWorldFromDriverRotation.w, pose.qWorldFromDriverRotation.x,
             pose.qWorldFromDriverRotation.y, pose.qWorldFromDriverRotation.z}};
}

spike::RigidPose localOf(const vr::DriverPose_t& pose)
{
    return {{pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2]},
            {pose.qRotation.w, pose.qRotation.x, pose.qRotation.y, pose.qRotation.z}};
}

spike::RigidPose driverFromHeadOf(const vr::DriverPose_t& pose)
{
    return {{pose.vecDriverFromHeadTranslation[0], pose.vecDriverFromHeadTranslation[1],
             pose.vecDriverFromHeadTranslation[2]},
            {pose.qDriverFromHeadRotation.w, pose.qDriverFromHeadRotation.x,
             pose.qDriverFromHeadRotation.y, pose.qDriverFromHeadRotation.z}};
}

using HmdDriverFactoryFn = void*(*)(const char*, int*);

// MinHook patches the *function* a vtable slot points to, so a test call only reaches
// the detour if it is a real virtual dispatch. A Release build devirtualizes and
// inlines calls made directly on a concrete fake, which skips the patched body and
// makes a working hook look broken (this cost a Release-only test failure once). A
// volatile pointer hides the object's identity from the optimizer, forcing the same
// indirect call vrserver's own callers make.
template <class T>
T* throughVtable(T* pointer)
{
    T* volatile hidden = pointer;
    return hidden;
}

// The DLL keeps global state (the device table, the "already logged" interface set)
// and its providers are function statics, so the driver lifecycle is exercised once,
// in order, in a single test.
class SpikeDriverLifecycle : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        module_ = LoadLibraryA(SPIKE_DRIVER_DLL);
        ASSERT_NE(module_, nullptr) << "LoadLibrary(" << SPIKE_DRIVER_DLL
                                    << ") failed, GetLastError=" << GetLastError();
        factory_ = reinterpret_cast<HmdDriverFactoryFn>(
            GetProcAddress(module_, "HmdDriverFactory"));
        ASSERT_NE(factory_, nullptr) << "HmdDriverFactory not exported";
    }

    static void TearDownTestSuite()
    {
        if (module_)
            FreeLibrary(module_);
        module_ = nullptr;
        factory_ = nullptr;
    }

    static HMODULE module_;
    static HmdDriverFactoryFn factory_;
};

HMODULE SpikeDriverLifecycle::module_ = nullptr;
HmdDriverFactoryFn SpikeDriverLifecycle::factory_ = nullptr;

TEST_F(SpikeDriverLifecycle, FactoryServesBothProvidersAndRejectsUnknownInterfaces)
{
    int code = 0;
    EXPECT_NE(factory_(vr::IServerTrackedDeviceProvider_Version, &code), nullptr);
    EXPECT_NE(factory_(vr::IVRWatchdogProvider_Version, &code), nullptr);

    code = 0;
    EXPECT_EQ(factory_("IDoesNotExist_001", &code), nullptr);
    EXPECT_EQ(code, vr::VRInitError_Init_InterfaceNotFound);

    // Not supposed to happen, but the C ABI cannot stop it: the DLL must answer rather
    // than dereference it while SteamVR is still loading and has no log to read.
    code = 0;
    EXPECT_EQ(factory_(nullptr, &code), nullptr);
    EXPECT_EQ(code, vr::VRInitError_Init_InterfaceNotFound);

    // A caller that does not want the error code at all.
    EXPECT_EQ(factory_("IDoesNotExist_001", nullptr), nullptr);
}

TEST_F(SpikeDriverLifecycle, HooksObserveAndForwardTheWholeDriverSurface)
{
    auto* server = static_cast<vr::IServerTrackedDeviceProvider*>(
        factory_(vr::IServerTrackedDeviceProvider_Version, nullptr));
    ASSERT_NE(server, nullptr);

    // The ABI validation table SteamVR rejects the driver without.
    ASSERT_NE(server->GetInterfaceVersions(), nullptr);
    EXPECT_STREQ(server->GetInterfaceVersions()[0], vr::IVRSettings_Version);

    FakeDriverContext context;
    context.properties.add(kTrackerIndex, {"LHR-TESTTRACKER", "Vive Tracker", "lighthouse",
                                           vr::TrackedDeviceClass_GenericTracker,
                                           vr::TrackedControllerRole_Invalid});

    ASSERT_EQ(server->Init(&context), vr::VRInitError_None);

    // Context init fetched the interfaces InitServer needs...
    EXPECT_TRUE(context.wasRequested(vr::IVRServerDriverHost_Version));
    EXPECT_TRUE(context.wasRequested(vr::IVRProperties_Version));
    EXPECT_TRUE(context.wasRequested(vr::IVRDriverLog_Version));
    // ...and IVRDriverInput was fetched afterwards, i.e. through the detour, which is
    // the mechanism that also catches interfaces other drivers ask for.
    EXPECT_TRUE(context.wasRequested(vr::IVRDriverInput_Version));

    // Every hook actually landed on its vtable slot.
    EXPECT_TRUE(context.log.contains("hook IVRDriverContext::GetGenericInterface: installed"));
    EXPECT_TRUE(context.log.contains("hook IVRServerDriverHost::TrackedDeviceAdded: installed"));
    EXPECT_TRUE(
        context.log.contains("hook IVRServerDriverHost::TrackedDevicePoseUpdated: installed"));
    EXPECT_TRUE(context.log.contains("hook IVRDriverInput::CreateBooleanComponent: installed"));
    EXPECT_TRUE(context.log.contains("hook IVRDriverInput::UpdateBooleanComponent: installed"));
    EXPECT_TRUE(context.log.contains("hook IVRDriverInput::CreateScalarComponent: installed"));
    EXPECT_TRUE(context.log.contains("interface \"IVRDriverInput_003\": hooking"));

    // Every call below is made through a pointer the optimizer cannot resolve, so it is
    // a real virtual dispatch into the patched function — the same path vrserver's own
    // callers take. Calling the fakes directly would let a Release build inline them and
    // skip the detour entirely.
    vr::IVRServerDriverHost* const host = throughVtable<vr::IVRServerDriverHost>(&context.host);
    vr::IVRDriverInput* const input = throughVtable<vr::IVRDriverInput>(&context.input);
    vr::IVRDriverContext* const driverContext =
        throughVtable<vr::IVRDriverContext>(&context);

    // ---- TrackedDeviceAdded: observed and forwarded ----
    EXPECT_TRUE(host->TrackedDeviceAdded("LHR-TESTTRACKER",
                                         vr::TrackedDeviceClass_GenericTracker, nullptr));
    ASSERT_EQ(context.host.addedSerials.size(), 1u);
    EXPECT_EQ(context.host.addedSerials[0], "LHR-TESTTRACKER");
    EXPECT_TRUE(context.log.contains(
        "TrackedDeviceAdded: serial=\"LHR-TESTTRACKER\" class=tracker"));

    // ---- pose hook: observed and forwarded byte-identically ----
    const vr::DriverPose_t pose = makeDriverPose();
    host->TrackedDevicePoseUpdated(kTrackerIndex, pose, sizeof(pose));
    ASSERT_EQ(context.host.forwardedPoses.size(), 1u);
    EXPECT_EQ(context.host.forwardedPoses[0].index, kTrackerIndex);
    EXPECT_EQ(context.host.forwardedPoses[0].positionX, pose.vecPosition[0]);
    EXPECT_EQ(context.host.forwardedPoses[0].structSize, sizeof(pose));
    EXPECT_TRUE(context.log.contains("first pose update from device 3"));

    // ---- input components ----
    vr::VRInputComponentHandle_t trigger = vr::k_ulInvalidInputComponentHandle;
    ASSERT_EQ(input->CreateBooleanComponent(kContainerBase + kTrackerIndex,
                                            "/input/trigger/click", &trigger),
              vr::VRInputError_None);
    EXPECT_NE(trigger, vr::k_ulInvalidInputComponentHandle);
    EXPECT_TRUE(context.log.contains("name=\"/input/trigger/click\" <-- trigger click"));

    vr::VRInputComponentHandle_t grip = vr::k_ulInvalidInputComponentHandle;
    input->CreateBooleanComponent(kContainerBase + kTrackerIndex, "/input/grip/click", &grip);
    EXPECT_FALSE(context.log.contains("name=\"/input/grip/click\" <-- trigger click"));

    vr::VRInputComponentHandle_t triggerValue = vr::k_ulInvalidInputComponentHandle;
    input->CreateScalarComponent(kContainerBase + kTrackerIndex, "/input/trigger/value",
                                 &triggerValue, vr::VRScalarType_Absolute,
                                 vr::VRScalarUnits_NormalizedOneSided);
    EXPECT_TRUE(context.log.contains("CreateScalarComponent"));
    EXPECT_TRUE(context.log.contains("/input/trigger/value"));

    // ---- the GetGenericInterface detour's branches, driven through the live hook ----
    // Every call below goes through the installed detour, so this exercises the real
    // onInterfaceRequested dispatch, not a copy of it.
    int dummyInterface = 0;
    context.extraInterfaces["IVRServerDriverHost_005"] = &dummyInterface;
    context.extraInterfaces["IVRDriverInput_002"] = &dummyInterface;
    context.extraInterfaces["IVRCameraComponent_004"] = nullptr;

    vr::EVRInitError interfaceError = vr::VRInitError_None;

    // A version of an interface we DO care about, but not the one we build against:
    // must be loud, and must NOT be hooked (the dummy pointer has no vtable — hooking
    // it would crash, so surviving this call is itself part of the assertion).
    driverContext->GetGenericInterface("IVRServerDriverHost_005", &interfaceError);
    EXPECT_TRUE(context.log.contains("interface \"IVRServerDriverHost_005\": NOT HOOKED"));
    driverContext->GetGenericInterface("IVRDriverInput_002", &interfaceError);
    EXPECT_TRUE(context.log.contains("interface \"IVRDriverInput_002\": NOT HOOKED"));

    // An interface we do not need at all.
    driverContext->GetGenericInterface(vr::IVRResources_Version, &interfaceError);
    EXPECT_TRUE(context.log.contains("interface \"IVRResources_001\": seen, not hooked"));

    // vrserver refusing an interface must be visible too.
    driverContext->GetGenericInterface("IVRCameraComponent_004", &interfaceError);
    EXPECT_TRUE(
        context.log.contains("interface \"IVRCameraComponent_004\": requested, vrserver "
                             "returned NULL"));

    // Deduplication: the same version string is reported once, however many drivers ask.
    const size_t linesBeforeRepeat = context.log.lines.size();
    driverContext->GetGenericInterface("IVRServerDriverHost_005", &interfaceError);
    driverContext->GetGenericInterface(vr::IVRResources_Version, &interfaceError);
    driverContext->GetGenericInterface("IVRCameraComponent_004", &interfaceError);
    EXPECT_EQ(context.log.lines.size(), linesBeforeRepeat);

    // ---- RunFrame: metadata read back driver-side, components resolved, pose sampled ----
    const uint32_t lookupsBefore = context.properties.containerLookups;
    server->RunFrame();
    EXPECT_GT(context.properties.containerLookups, lookupsBefore);
    EXPECT_TRUE(context.log.contains("first RunFrame call"));
    EXPECT_TRUE(context.log.contains(
        "device 3: class=tracker(3) role=invalid serial=\"LHR-TESTTRACKER\" "
        "model=\"Vive Tracker\" trackingSystem=\"lighthouse\""));
    EXPECT_TRUE(context.log.contains(
        "component \"/input/trigger/click\" resolved to device 3 (\"LHR-TESTTRACKER\")"));

    // The composition is wired from the DriverPose_t fields the plan names, in the
    // stated order: A = WorldFromDriver o local, B = A o DriverFromHead.
    const spike::RigidPose a = spike::compose(worldFromDriverOf(pose), localOf(pose));
    const spike::RigidPose b = spike::compose(a, driverFromHeadOf(pose));
    EXPECT_TRUE(context.log.contains("A = wFd o local  " + spike::formatPose(a)));
    EXPECT_TRUE(context.log.contains("B = A o dFh      " + spike::formatPose(b)));
    EXPECT_NE(spike::formatPose(a), spike::formatPose(b)) << "DriverFromHead must matter here";

    // ---- trigger edges ----
    input->UpdateBooleanComponent(trigger, true, 0.0);
    ASSERT_EQ(context.input.forwardedBooleanUpdates.size(), 1u);
    EXPECT_TRUE(context.input.forwardedBooleanUpdates[0].value);
    EXPECT_TRUE(context.log.contains(
        "trigger DOWN: device 3 (LHR-TESTTRACKER) component \"/input/trigger/click\""));

    const size_t linesAfterPress = context.log.lines.size();
    input->UpdateBooleanComponent(trigger, true, 0.0); // repeat, not an edge
    EXPECT_EQ(context.log.lines.size(), linesAfterPress);

    input->UpdateBooleanComponent(trigger, false, 0.0);
    EXPECT_TRUE(context.log.contains("trigger up  : device 3"));

    // A handle we never saw created must not crash and must not be attributed.
    input->UpdateBooleanComponent(987654321, true, 0.0);
    EXPECT_EQ(context.input.forwardedBooleanUpdates.size(), 4u);

    // ---- standby: forwarded to the observer, and never blocking SteamVR ----
    EXPECT_FALSE(server->ShouldBlockStandbyMode());
    server->EnterStandby();
    server->LeaveStandby();
    EXPECT_TRUE(context.log.contains("EnterStandby"));
    EXPECT_TRUE(context.log.contains("LeaveStandby"));

    // ---- Cleanup unhooks everything ----
    server->Cleanup();
    EXPECT_TRUE(context.log.contains("hook IVRDriverContext::GetGenericInterface: removed"));
    EXPECT_TRUE(
        context.log.contains("hook IVRServerDriverHost::TrackedDevicePoseUpdated: removed"));
    EXPECT_TRUE(context.log.contains("hook IVRDriverInput::UpdateBooleanComponent: removed"));
    EXPECT_TRUE(context.log.contains("summary: device 3 tracker \"LHR-TESTTRACKER\": 1 pose"));

    // Calls still reach the real implementation once the detours are gone.
    const size_t linesAfterCleanup = context.log.lines.size();
    host->TrackedDevicePoseUpdated(kTrackerIndex, pose, sizeof(pose));
    input->UpdateBooleanComponent(trigger, true, 0.0);
    EXPECT_EQ(context.host.forwardedPoses.size(), 2u);
    EXPECT_EQ(context.input.forwardedBooleanUpdates.size(), 5u);
    EXPECT_EQ(context.log.lines.size(), linesAfterCleanup) << "driver still observing after Cleanup";
}

// ------------------------------------------------ interface classification ----
// The detour's decision table, tested directly: which version strings we hook, which
// ones must be reported as unsupported, and which are none of our business.

TEST(SpikeInterfaceClassification, ExactVersionsWeBuildAgainstAreHooked)
{
    EXPECT_EQ(spike::classifyInterface("IVRServerDriverHost_006", "IVRServerDriverHost_006",
                                       "IVRDriverInput_003"),
              spike::InterfaceAction::HookServerDriverHost);
    EXPECT_EQ(spike::classifyInterface("IVRDriverInput_003", "IVRServerDriverHost_006",
                                       "IVRDriverInput_003"),
              spike::InterfaceAction::HookDriverInput);
}

TEST(SpikeInterfaceClassification, OtherVersionsOfTheSameInterfacesAreUnsupportedNotIgnored)
{
    // Older and newer alike: the vtable layout is not guaranteed, so we refuse to hook
    // and say so — silence here would mean invisible devices.
    EXPECT_EQ(spike::classifyInterface("IVRServerDriverHost_005", "IVRServerDriverHost_006",
                                       "IVRDriverInput_003"),
              spike::InterfaceAction::UnsupportedVersion);
    EXPECT_EQ(spike::classifyInterface("IVRServerDriverHost_008", "IVRServerDriverHost_006",
                                       "IVRDriverInput_003"),
              spike::InterfaceAction::UnsupportedVersion);
    EXPECT_EQ(spike::classifyInterface("IVRDriverInput_002", "IVRServerDriverHost_006",
                                       "IVRDriverInput_003"),
              spike::InterfaceAction::UnsupportedVersion);
}

TEST(SpikeInterfaceClassification, UnrelatedInterfacesAreNotNeeded)
{
    for (const char* version : {"IVRSettings_003", "IVRProperties_001", "IVRDriverLog_001",
                                "IVRResources_001", "IVRIOBuffer_002", "(null)", ""})
        EXPECT_EQ(spike::classifyInterface(version, "IVRServerDriverHost_006",
                                           "IVRDriverInput_003"),
                  spike::InterfaceAction::NotNeeded)
            << version;
}

TEST(SpikeInterfaceClassification, FamilyMatchingIsExactNotAPrefixMatch)
{
    // Names that merely start like ours must not be mistaken for our interfaces.
    EXPECT_EQ(spike::classifyInterface("IVRServerDriverHostExtras_001",
                                       "IVRServerDriverHost_006", "IVRDriverInput_003"),
              spike::InterfaceAction::NotNeeded);
    EXPECT_EQ(spike::classifyInterface("IVRServerDriverHost", "IVRServerDriverHost_006",
                                       "IVRDriverInput_003"),
              spike::InterfaceAction::NotNeeded);
}

TEST(SpikeInterfaceClassification, ClassificationFollowsTheVersionsWeAreBuiltAgainst)
{
    // Same input, different build-time versions: what counts as hookable moves with us.
    EXPECT_EQ(spike::classifyInterface("IVRServerDriverHost_008", "IVRServerDriverHost_008",
                                       "IVRDriverInput_003"),
              spike::InterfaceAction::HookServerDriverHost);
    EXPECT_EQ(spike::classifyInterface("IVRServerDriverHost_006", "IVRServerDriverHost_008",
                                       "IVRDriverInput_003"),
              spike::InterfaceAction::UnsupportedVersion);
}

TEST(SpikeInterfaceClassification, FamilyIsTheNameUpToTheLastUnderscore)
{
    EXPECT_EQ(spike::interfaceFamily("IVRDriverInput_003"), "IVRDriverInput_");
    EXPECT_EQ(spike::interfaceFamily("IVRServerDriverHost_006"), "IVRServerDriverHost_");
    EXPECT_EQ(spike::interfaceFamily("NoUnderscore"), "NoUnderscore");
    EXPECT_EQ(spike::interfaceFamily(""), "");
}

// ---------------------------------------------------------- composition math ----

constexpr double kTolerance = 1e-12;

void expectPoseNear(const spike::RigidPose& actual, const spike::V3& position,
                    const spike::Q& rotation)
{
    EXPECT_NEAR(actual.pos.x, position.x, kTolerance);
    EXPECT_NEAR(actual.pos.y, position.y, kTolerance);
    EXPECT_NEAR(actual.pos.z, position.z, kTolerance);
    EXPECT_NEAR(actual.rot.w, rotation.w, kTolerance);
    EXPECT_NEAR(actual.rot.x, rotation.x, kTolerance);
    EXPECT_NEAR(actual.rot.y, rotation.y, kTolerance);
    EXPECT_NEAR(actual.rot.z, rotation.z, kTolerance);
}

TEST(SpikePoseMath, IdentityComposesToTheOtherOperand)
{
    const spike::RigidPose identity{};
    const spike::RigidPose pose{{1.0, 2.0, 3.0}, {0.5, 0.5, 0.5, 0.5}};
    expectPoseNear(spike::compose(identity, pose), pose.pos, pose.rot);
    expectPoseNear(spike::compose(pose, identity), pose.pos, pose.rot);
}

TEST(SpikePoseMath, RotateMatchesRightHandedYUpConvention)
{
    // +90 deg about Y takes +X to -Z in a right-handed Y-up space.
    const double s = std::sqrt(0.5);
    const spike::Q yaw90{s, 0.0, s, 0.0};
    const spike::V3 rotated = spike::rotate(yaw90, {1.0, 0.0, 0.0});
    EXPECT_NEAR(rotated.x, 0.0, kTolerance);
    EXPECT_NEAR(rotated.y, 0.0, kTolerance);
    EXPECT_NEAR(rotated.z, -1.0, kTolerance);
}

TEST(SpikePoseMath, ComposeAppliesTheSecondOperandFirst)
{
    const double s = std::sqrt(0.5);
    const spike::RigidPose outer{{0.0, 1.0, 0.0}, {s, 0.0, s, 0.0}}; // +90 deg yaw, up 1 m
    const spike::RigidPose inner{{1.0, 0.0, 0.0}, {}};               // 1 m along +X

    // outer o inner: inner's offset is rotated by outer, then translated.
    expectPoseNear(spike::compose(outer, inner), {0.0, 1.0, -1.0}, outer.rot);
    // The other order must differ — this asymmetry is what makes the DriverPose_t
    // composition order a real question rather than a formality.
    expectPoseNear(spike::compose(inner, outer), {1.0, 1.0, 0.0}, outer.rot);
}

TEST(SpikePoseMath, DriverPoseCompositionMatchesTheAnalyticCase)
{
    // WorldFromDriver: 180 deg about Y, 10 m along +X.
    const spike::RigidPose worldFromDriver{{10.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}};
    // Driver-local pose: (1, 2, 3), also 180 deg about Y.
    const spike::RigidPose local{{1.0, 2.0, 3.0}, {0.0, 0.0, 1.0, 0.0}};
    // DriverFromHead: 0.5 m up, no rotation.
    const spike::RigidPose driverFromHead{{0.0, 0.5, 0.0}, {}};

    // 180 deg about Y negates x and z: (1,2,3) -> (-1,2,-3); plus (10,0,0).
    const spike::RigidPose a = spike::compose(worldFromDriver, local);
    expectPoseNear(a, {9.0, 2.0, -3.0}, {-1.0, 0.0, 0.0, 0.0});

    // A's rotation is a full turn, so the head offset stays +Y.
    const spike::RigidPose b = spike::compose(a, driverFromHead);
    expectPoseNear(b, {9.0, 2.5, -3.0}, {-1.0, 0.0, 0.0, 0.0});
}
} // namespace
