#pragma once

// Throwaway step-1 spike (doc/driver-plan.md): the vrserver / Win32 glue the server
// provider needs, as testable code.
//
// These three things used to be spelled out inside SpikeDriver.cpp — which no unit test
// can reach, because that file is compiled only into the driver DLL (see
// doc/driver-spike-handover.md §2.1). Each of them had a branch that consequently ran in
// no test at all: "does vrserver have IVRProperties", "does it have IVRDriverLog", "did
// the property read succeed". They are here instead, with the one thing that genuinely
// cannot be faked — the global accessor, the Win32 call — injected:
//
//   * OpenVrProperties      — DeviceProperties over vr::VRProperties(), which is a
//                             CVRPropertyHelpers* that may be null before the context
//                             is initialized. Tests pass a helper over a fake
//                             IVRProperties.
//   * driverLogSink         — the LogSink that copies lines to IVRDriverLog (one of the
//                             destinations the adapter's composite sink fans out to).
//   * modulePathOfAddress   — GetModuleHandleEx + GetModuleFileName, composed over a
//                             seam so the composition is tested and the DLL keeps only
//                             two argument-marshalling calls.

#include "SpikeLog.h"
#include "SpikeObserver.h"

#include <openvr_driver.h>

#include <cstdint>
#include <string>

namespace spike
{
// ---- device metadata --------------------------------------------------------------

// vr::VRProperties(). Null until InitServerDriverContext has run, and null forever in
// a vrserver that hands us no IVRProperties.
using PropertyHelpersFn = vr::CVRPropertyHelpers*(*)();

class OpenVrProperties final : public DeviceProperties
{
public:
    explicit OpenVrProperties(PropertyHelpersFn helpers) : helpers_(helpers) {}

    // What ServerEnvironment::properties() answers: the seam itself, or nullptr when
    // vrserver has no properties interface for us (the observer then logs no metadata
    // rather than dereferencing null).
    DeviceProperties* orNullIfUnavailable();

    vr::PropertyContainerHandle_t container(uint32_t deviceIndex) override;
    bool stringProperty(vr::PropertyContainerHandle_t container,
                        vr::ETrackedDeviceProperty property, std::string& value) override;
    int32_t int32Property(vr::PropertyContainerHandle_t container,
                          vr::ETrackedDeviceProperty property) override;

private:
    PropertyHelpersFn helpers_;
};

// ---- log routing ------------------------------------------------------------------

// vr::VRDriverLog(). Null in vrwatchdog, and null before the context is initialized.
using DriverLogFn = vr::IVRDriverLog*(*)();

// A sink that copies every line to IVRDriverLog (so the spike's output also lands in
// SteamVR's vrserver.txt) and silently drops it when there is no driver log. The seam
// is read on every line because IVRDriverLog appears and disappears with the context.
LogSink driverLogSink(DriverLogFn driverLog);

// ---- module path ------------------------------------------------------------------

// MAX_PATH, so this header needs no windows.h.
constexpr unsigned long kMaxModulePath = 260;

// Deliberately as primitive as the Win32 calls themselves: out parameter and caller's
// buffer, so the DLL's implementation is one call each and the buffer handling — which
// is where the mistakes are — happens here, under test.
class ModuleApi
{
public:
    virtual ~ModuleApi() = default;

    // GetModuleHandleExA(FROM_ADDRESS | UNCHANGED_REFCOUNT): nonzero on success, and
    // *module is the module containing `address`.
    virtual int moduleFromAddress(void* address, void** module) = 0;
    // GetModuleFileNameA: characters written, excluding the terminator; 0 on failure,
    // and `size` when the path did not fit.
    virtual unsigned long moduleFileName(void* module, char* buffer, unsigned long size) = 0;
};

// Which DLL file are we running from — logged at Init so a live session proves *which*
// build SteamVR loaded. Empty when the module cannot be identified.
std::string modulePathOfAddress(ModuleApi& api, void* address);
} // namespace spike
