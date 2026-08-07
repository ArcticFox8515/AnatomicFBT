#include "SpikeDriverEnvironment.h"

namespace spike
{
// ---- device metadata --------------------------------------------------------------

DeviceProperties* OpenVrProperties::orNullIfUnavailable()
{
    return helpers_() ? this : nullptr;
}

vr::PropertyContainerHandle_t OpenVrProperties::container(uint32_t deviceIndex)
{
    if (!helpers_())
        return vr::k_ulInvalidPropertyContainer;
    return helpers_()->TrackedDeviceToPropertyContainer(deviceIndex);
}

bool OpenVrProperties::stringProperty(vr::PropertyContainerHandle_t container,
                                     vr::ETrackedDeviceProperty property, std::string& value)
{
    if (!helpers_())
        return false;

    vr::ETrackedPropertyError error = vr::TrackedProp_Success;
    value = helpers_()->GetStringProperty(container, property, &error);
    // A device whose container exists but whose properties are not written yet answers
    // with an error and an empty string; reporting that as success would log blanks.
    return error == vr::TrackedProp_Success;
}

int32_t OpenVrProperties::int32Property(vr::PropertyContainerHandle_t container,
                                       vr::ETrackedDeviceProperty property)
{
    if (!helpers_())
        return 0;
    return helpers_()->GetInt32Property(container, property);
}

// ---- log routing ------------------------------------------------------------------

LogSink driverLogSink(DriverLogFn driverLog)
{
    return [driverLog](const char* message) {
        if (vr::IVRDriverLog* log = driverLog())
            log->Log(message);
    };
}

// ---- module path ------------------------------------------------------------------

std::string modulePathOfAddress(ModuleApi& api, void* address)
{
    void* module = nullptr;
    // A failed lookup must not fall through to GetModuleFileName(nullptr), which
    // answers with the *executable* — vrserver.exe instead of our DLL.
    if (!api.moduleFromAddress(address, &module) || !module)
        return {};

    std::string path(kMaxModulePath, '\0');
    const unsigned long written = api.moduleFileName(module, &path[0], kMaxModulePath);
    // 0 on failure, kMaxModulePath when the path was truncated: both are simply the
    // number of characters we may keep.
    path.resize(written);
    return path;
}
} // namespace spike
