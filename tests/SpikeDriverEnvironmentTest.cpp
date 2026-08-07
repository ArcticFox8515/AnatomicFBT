// Tests for the throwaway step-1 spike's vrserver / Win32 glue
// (doc/driver-plan.md, doc/driver-spike-handover.md §2.1).
//
// These three adapters used to be written out inside SpikeDriver.cpp, which no unit test
// can execute — it is compiled only into the driver DLL. Each carried a branch that
// therefore ran nowhere: "does vrserver have IVRProperties", "does it have IVRDriverLog",
// "did the property read succeed". A live SteamVR session takes only the success side of
// all three, so those were exactly the branches with no evidence behind them.
//
// vr::VRProperties() / vr::VRDriverLog() are function-pointer seams here, so the fakes
// are plain openvr interface implementations and nothing touches a real driver context.

#include "spike/SpikeDriverEnvironment.h"

#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

namespace
{
constexpr vr::PropertyContainerHandle_t kContainerBase = 1000;

// ---- fake IVRProperties ----------------------------------------------------------

class FakeProperties : public vr::IVRProperties
{
public:
    vr::ETrackedPropertyError ReadPropertyBatch(vr::PropertyContainerHandle_t container,
                                               vr::PropertyRead_t* batch,
                                               uint32_t count) override
    {
        for (uint32_t i = 0; i < count; ++i)
        {
            vr::PropertyRead_t& read = batch[i];
            reads.push_back({container, read.prop});

            if (container != kContainerBase)
            {
                read.eError = vr::TrackedProp_InvalidDevice;
                continue;
            }

            switch (read.prop)
            {
            case vr::Prop_SerialNumber_String: writeString(read, serial); break;
            case vr::Prop_DeviceClass_Int32: writeInt32(read, deviceClass); break;
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

    struct Read
    {
        vr::PropertyContainerHandle_t container;
        vr::ETrackedDeviceProperty property;
    };

    std::vector<Read> reads;
    uint32_t containerLookups = 0;
    std::string serial = "LHR-ENVIRONMENT";
    int32_t deviceClass = vr::TrackedDeviceClass_GenericTracker;

private:
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

// ---- fake IVRDriverLog -----------------------------------------------------------

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

// The seams are C function pointers (that is what vr::VRProperties / vr::VRDriverLog
// are), so what they return has to live at file scope.
vr::CVRPropertyHelpers* g_helpers = nullptr;
vr::IVRDriverLog* g_driverLog = nullptr;

vr::CVRPropertyHelpers* helpersSeam()
{
    return g_helpers;
}

vr::IVRDriverLog* driverLogSeam()
{
    return g_driverLog;
}

class SpikeDriverEnvironmentTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        g_helpers = &helpers_;
        g_driverLog = &driverLog_;
    }

    void TearDown() override
    {
        g_helpers = nullptr;
        g_driverLog = nullptr;
    }

    FakeProperties properties_;
    vr::CVRPropertyHelpers helpers_{&properties_};
    FakeDriverLog driverLog_;
    spike::OpenVrProperties subject_{&helpersSeam};
};

// ------------------------------------------------------------- properties ----

TEST_F(SpikeDriverEnvironmentTest, PropertiesAreOfferedOnceVrserverHasThem)
{
    EXPECT_EQ(subject_.orNullIfUnavailable(), &subject_);
}

TEST_F(SpikeDriverEnvironmentTest, PropertiesAreWithheldWhenVrserverHasNone)
{
    // Before InitServerDriverContext, and forever in a vrserver that gives us no
    // IVRProperties. The observer must get nullptr rather than a seam that would
    // dereference null on the first metadata read.
    g_helpers = nullptr;
    EXPECT_EQ(subject_.orNullIfUnavailable(), nullptr);
}

TEST_F(SpikeDriverEnvironmentTest, ContainerLooksUpTheDeviceIndex)
{
    EXPECT_EQ(subject_.container(3), kContainerBase + 3);
    EXPECT_EQ(properties_.containerLookups, 1u);
}

TEST_F(SpikeDriverEnvironmentTest, ContainerIsInvalidWithoutProperties)
{
    g_helpers = nullptr;
    EXPECT_EQ(subject_.container(3), vr::k_ulInvalidPropertyContainer);
    EXPECT_EQ(properties_.containerLookups, 0u);
}

TEST_F(SpikeDriverEnvironmentTest, StringPropertyReturnsTheValue)
{
    std::string value = "untouched";
    EXPECT_TRUE(subject_.stringProperty(kContainerBase, vr::Prop_SerialNumber_String, value));
    EXPECT_EQ(value, "LHR-ENVIRONMENT");
    ASSERT_EQ(properties_.reads.size(), 1u);
    EXPECT_EQ(properties_.reads[0].container, kContainerBase);
    EXPECT_EQ(properties_.reads[0].property, vr::Prop_SerialNumber_String);
}

TEST_F(SpikeDriverEnvironmentTest, StringPropertyFailureIsReportedRatherThanLoggedAsBlank)
{
    // A device whose container exists but whose properties are not written yet: openvr
    // answers with an error and an empty string, which must not be reported as metadata.
    std::string value;
    EXPECT_FALSE(subject_.stringProperty(kContainerBase + 99, vr::Prop_SerialNumber_String, value));
}

TEST_F(SpikeDriverEnvironmentTest, StringPropertyWithoutPropertiesFails)
{
    g_helpers = nullptr;
    std::string value = "untouched";
    EXPECT_FALSE(subject_.stringProperty(kContainerBase, vr::Prop_SerialNumber_String, value));
    EXPECT_EQ(value, "untouched");
    EXPECT_TRUE(properties_.reads.empty());
}

TEST_F(SpikeDriverEnvironmentTest, Int32PropertyReturnsTheValue)
{
    properties_.deviceClass = vr::TrackedDeviceClass_Controller;
    EXPECT_EQ(subject_.int32Property(kContainerBase, vr::Prop_DeviceClass_Int32),
              vr::TrackedDeviceClass_Controller);
}

TEST_F(SpikeDriverEnvironmentTest, Int32PropertyWithoutPropertiesIsZero)
{
    g_helpers = nullptr;
    EXPECT_EQ(subject_.int32Property(kContainerBase, vr::Prop_DeviceClass_Int32), 0);
    EXPECT_TRUE(properties_.reads.empty());
}

// -------------------------------------------------------------- log sink ----

TEST_F(SpikeDriverEnvironmentTest, DriverLogSinkCopiesLinesToVrserverTxt)
{
    const spike::LogSink sink = spike::driverLogSink(&driverLogSeam);
    sink("hello");
    sink("world");

    ASSERT_EQ(driverLog_.lines.size(), 2u);
    EXPECT_EQ(driverLog_.lines[0], "hello");
    EXPECT_EQ(driverLog_.lines[1], "world");
}

TEST_F(SpikeDriverEnvironmentTest, DriverLogSinkDropsLinesWhenThereIsNoDriverLog)
{
    // vrwatchdog, and the window before the context is initialized. The sink is already
    // installed by then, so this branch is reached on every early line.
    const spike::LogSink sink = spike::driverLogSink(&driverLogSeam);
    g_driverLog = nullptr;
    sink("dropped");
    EXPECT_TRUE(driverLog_.lines.empty());
}

TEST_F(SpikeDriverEnvironmentTest, DriverLogSinkReadsTheSeamOnEveryLine)
{
    // The sink is built once, at Init, but IVRDriverLog appears and disappears with the
    // context — so it must not be captured by value.
    const spike::LogSink sink = spike::driverLogSink(&driverLogSeam);
    g_driverLog = nullptr;
    sink("dropped");
    g_driverLog = &driverLog_;
    sink("kept");

    ASSERT_EQ(driverLog_.lines.size(), 1u);
    EXPECT_EQ(driverLog_.lines[0], "kept");
}

// ------------------------------------------------------------ module path ----

class FakeModuleApi : public spike::ModuleApi
{
public:
    int moduleFromAddress(void* address, void** module) override
    {
        requestedAddress = address;
        *module = this->module;
        return found;
    }

    unsigned long moduleFileName(void* requested, char* buffer, unsigned long size) override
    {
        requestedModule = requested;
        offeredSize = size;
        const unsigned long written =
            static_cast<unsigned long>(fileName.size() < size ? fileName.size() : size);
        std::memcpy(buffer, fileName.c_str(), written);
        return written;
    }

    void* requestedAddress = nullptr;
    void* requestedModule = nullptr;
    unsigned long offeredSize = 0;
    void* module = nullptr;
    int found = 1;
    std::string fileName;
};

TEST(SpikeModulePath, ResolvesTheFileNameOfTheModuleHoldingTheAddress)
{
    int somethingInThisModule = 0;
    FakeModuleApi api;
    api.module = &api;
    api.fileName = "C:\\build\\driver_00trackingcorrector.dll";

    // The string must be trimmed to what Win32 wrote — a buffer-sized std::string full
    // of NULs would log the path followed by 200 zero bytes.
    EXPECT_EQ(spike::modulePathOfAddress(api, &somethingInThisModule),
              "C:\\build\\driver_00trackingcorrector.dll");
    EXPECT_EQ(api.requestedAddress, &somethingInThisModule);
    EXPECT_EQ(api.requestedModule, &api);
    EXPECT_EQ(api.offeredSize, spike::kMaxModulePath);
}

TEST(SpikeModulePath, IsEmptyWhenTheLookupFails)
{
    // GetModuleHandleEx failing must not turn into a GetModuleFileName call on nullptr,
    // which would report the *executable* — i.e. vrserver.exe instead of our DLL.
    int somethingInThisModule = 0;
    FakeModuleApi api;
    api.found = 0;
    api.module = &api;
    api.fileName = "should not be asked";

    EXPECT_EQ(spike::modulePathOfAddress(api, &somethingInThisModule), "");
    EXPECT_EQ(api.requestedModule, nullptr);
}

TEST(SpikeModulePath, IsEmptyWhenTheLookupSucceedsWithNoModule)
{
    int somethingInThisModule = 0;
    FakeModuleApi api;
    api.found = 1;
    api.module = nullptr;

    EXPECT_EQ(spike::modulePathOfAddress(api, &somethingInThisModule), "");
    EXPECT_EQ(api.requestedModule, nullptr);
}

TEST(SpikeModulePath, IsEmptyWhenTheNameCannotBeRead)
{
    int somethingInThisModule = 0;
    FakeModuleApi api;
    api.module = &api;
    api.fileName = "";

    EXPECT_EQ(spike::modulePathOfAddress(api, &somethingInThisModule), "");
}

TEST(SpikeModulePath, KeepsOnlyWhatFitWhenThePathIsTruncated)
{
    int somethingInThisModule = 0;
    FakeModuleApi api;
    api.module = &api;
    api.fileName = std::string(spike::kMaxModulePath + 50, 'x');

    const std::string path = spike::modulePathOfAddress(api, &somethingInThisModule);
    EXPECT_EQ(path.size(), spike::kMaxModulePath);
    EXPECT_EQ(path, std::string(spike::kMaxModulePath, 'x'));
}
} // namespace
