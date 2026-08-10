#pragma once

// The driver's virtual-tracker emitter (doc/virtual-trackers-plan.md step 6).
//
// One `driver::VirtualTracker` (implements `vr::ITrackedDeviceServerDriver`) exists per
// bone the app emits a `link::VirtualTracker` frame for. The `VirtualTrackerProvider`
// owns the roster and drives registration, property-writing and pose-pushing through two
// injected seams:
//
//   * `DeviceProperties` (Observer.h) — extended with `setStringProperty`. Wraps
//     `vr::VRProperties()` for container lookup and string writes. Reused by the
//     Observer for reads, so the emitter shares the existing seam rather than adding a
//     second VRProperties wrapper.
//   * `ServerDriverHost` (below) — the new seam over `vr::VRServerDriverHost()` for
//     `TrackedDeviceAdded` and `TrackedDevicePoseUpdated`. Those calls live on a
//     different vrserver interface than VRProperties, so they get their own seam.
//
// All logic lives here, in DriverLib; `Driver.cpp` supplies the OpenVR-backed
// implementations of both seams and injects them. Every `ITrackedDeviceServerDriver`
// entry point SteamVR calls back on a `VirtualTracker` is `runGuarded` — an exception
// escaping into vrserver.exe kills SteamVR (the same rule the provider entry points
// follow).
//
// Threading: the provider is driven entirely from `Server::runFrame` (the vrserver
// RunFrame thread) — `onMessages`, `onRunFrame`, `markAllDisconnected`. No foreign
// thread touches the roster, so the device map needs no lock. The device's last pose and
// its assigned index ARE read from SteamVR's own threads (`GetPose` is polled, `Activate`
// is called by vrserver), so those two fields are `std::atomic`.

#include "link/Log.h"
#include "Observer.h"

#include "link/MessageChannel.h"

#include <openvr_driver.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace driver
{
// Seam over `vr::VRServerDriverHost()` for the two calls the emitter makes:
// registering a new device and pushing a pose update. OpenVR-backed implementation
// lives in Driver.cpp; tests pass a fake.
class ServerDriverHost
{
public:
    virtual ~ServerDriverHost() = default;

    // vr::VRServerDriverHost()->TrackedDeviceAdded. SteamVR calls Activate on the
    // driver pointer synchronously (or shortly after); the device records its index
    // there. Returns true when SteamVR accepted the device.
    virtual bool trackedDeviceAdded(const char* serial, vr::ETrackedDeviceClass deviceClass,
                                    vr::ITrackedDeviceServerDriver* driver) = 0;

    // vr::VRServerDriverHost()->TrackedDevicePoseUpdated. Pushes a new pose for a
    // device whose index is already valid.
    virtual void poseUpdated(uint32_t index, const vr::DriverPose_t& pose,
                             uint32_t poseStructSize) = 0;
};

// A virtual tracker for one bone. One instance exists per bone name the app emits;
// SteamVR calls the ITrackedDeviceServerDriver entry points on this object directly.
//
// The device does NOT call vrserver itself: the provider owns the seams and pushes
// registration/props/poses through them. The device only holds state SteamVR needs to
// read back — the assigned index (atomic, written from Activate on SteamVR's thread,
// read by the provider on the RunFrame thread) and the last pose (atomic, written by
// the provider, read by GetPose on SteamVR's poll thread).
class VirtualTracker final : public vr::ITrackedDeviceServerDriver
{
public:
    VirtualTracker(const std::string& serial, const std::string& boneName);

    const std::string& serial() const { return serial_; }
    const std::string& boneName() const { return boneName_; }

    // k_unTrackedDeviceIndexInvalid until Activate assigns an index.
    uint32_t deviceIndex() const { return deviceIndex_.load(); }
    bool isActivated() const { return deviceIndex() != vr::k_unTrackedDeviceIndexInvalid; }

    // Stores the incoming pose for GetPose to return. Called by the provider on the
    // RunFrame thread; reads on SteamVR's poll thread are safe through the atomic.
    void setPose(const vr::DriverPose_t& pose);

    // ---- ITrackedDeviceServerDriver (called by SteamVR via vtable) ----
    vr::EVRInitError Activate(uint32_t unObjectId) override;
    void Deactivate() override;
    void EnterStandby() override;
    void* GetComponent(const char* pchComponentNameAndVersion) override;
    void DebugRequest(const char* pchRequest, char* pchResponseBuffer,
                      uint32_t unResponseBufferSize) override;
    vr::DriverPose_t GetPose() override;

private:
    std::string serial_;
    std::string boneName_;
    std::atomic<uint32_t> deviceIndex_{vr::k_unTrackedDeviceIndexInvalid};
    std::atomic<vr::DriverPose_t> pose_{};
};

// Owns the roster: one `VirtualTracker` per bone name the app has emitted a frame for.
// Drives registration, property-writing and pose-pushing through the injected seams.
//
// Roster identity is the bone name (stable across frames); the device's SteamVR serial
// is a pure function of the bone name (`serialForBone`), so the app always refers to the
// same device across restarts. A bone name seen for the first time creates a device;
// a repeat updates its pose. Leaving Capture (the app stops sending) makes a device's
// pose go stale; `onRunFrame` marks it disconnected after `kVirtualTrackerStaleSeconds`.
// A pipe drop marks all devices disconnected at once via `markAllDisconnected`.
class VirtualTrackerProvider
{
public:
    VirtualTrackerProvider(link::Logger& logger, DeviceProperties& properties,
                           ServerDriverHost& host, NowFn now);

    // Consumes `link::MessageType::VirtualTracker` frames from the channel; other
    // message types are ignored. For each: creates a device for a new bone name,
    // updates the pose on an existing one, writes props once the index is valid, and
    // pushes the pose. Idempotent on a duplicate roster entry.
    void onMessages(const std::vector<link::Message>& messages);

    // Staleness pass: a connected device whose index is valid and that has not been
    // refreshed within `kVirtualTrackerStaleSeconds` is marked disconnected (one pose
    // push with deviceIsConnected=false, pushed once).
    void onRunFrame();

    // Pipe-drop edge: mark every active device disconnected.
    void markAllDisconnected();

private:
    struct Entry
    {
        std::unique_ptr<VirtualTracker> device;
        double lastSeenAt = 0.0;
        bool propsWritten = false;
        bool connected = false;
    };

    // Builds a DriverPose_t for a connected device from the wire pose.
    vr::DriverPose_t buildPose(const link::VirtualTracker& wire) const;
    // Builds the disconnected pose (poseIsValid=false, deviceIsConnected=false).
    vr::DriverPose_t disconnectedPose() const;
    // Writes the four properties on a freshly-activated device.
    void writeProperties(uint32_t index);
    // Pushes a pose for an activated device; no-op when the index is invalid.
    void pushPose(VirtualTracker& device, const vr::DriverPose_t& pose);
    // Marks a single entry disconnected (pushes one disconnected pose, flips the flag).
    void markDisconnected(Entry& entry);

    link::Logger& log_;
    DeviceProperties& properties_;
    ServerDriverHost& host_;
    NowFn now_;
    std::unordered_map<std::string, Entry> devices_;
};

// Staleness window after which a device that received no frame is marked disconnected.
// Mirrors the OverrideCache expiry (`kOverrideStaleSeconds`) — a stopped app must not
// leave trackers frozen on their last pose forever.
constexpr double kVirtualTrackerStaleSeconds = 0.5;

// Serial SteamVR knows the device by. Pure function of the bone name so the app always
// refers to the same device across restarts; `TC-` prefix avoids colliding with real
// devices' serials (LHR-..., MAT-...).
std::string serialForBone(const std::string& boneName);
} // namespace driver
