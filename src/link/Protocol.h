#pragma once

// Wire types for the driver link (doc/driver-plan.md phase A, step 3).
//
// A single downstream message — `DevicePose` — covers everything the app
// consumes from the tracking provider today (see OpenVrTracking pollPoses): the
// device's kind and serial (the only things the app knows a device by besides
// its id) and the per-device pose (id + position + rotation + the one validity
// predicate). The separate metadata message was folded into the pose in step 3
// so the channel carries one frame per device update, not two. Nothing else
// from `DriverPose_t` is read downstream yet, so nothing else is on the wire.
//
// `PoseOverride` is the upstream message: the app's correction of a device's
// pose, sent back through the same duplex pipe so the driver can rewrite the
// pose it hands to SteamVR. It carries a device id and a rigid world-space
// delta (`position`/`rotation`) such that `compose(delta, rawWorld)` yields the
// corrected world pose; the driver applies it by premultiplying
// `worldFromDriver`, so vrserver's pose prediction stays exact.
//
// `VirtualTracker` is the upstream message for virtual trackers
// (doc/virtual-trackers-plan.md step 5): one frame per bone the app emits,
// carrying the bone name (so the driver needs no compile-time slot list — the
// set of names arriving this frame is the roster), a tracking flag, and the
// world pose computed from the retargeted avatar. Sent only while the app is
// in Capture mode with calibration complete; the moment it stops arriving the
// driver marks the device disconnected (staleness), so leaving Capture stops
// them tracking without any extra "drop" frame. The bone name identifies the
// device; a name the driver has not seen before is a new device, a repeat is
// the same device (step 6 dedups).
//
// The wire format is POD structs with explicit-width fields, memcpy'd whole —
// same style as `.tcrec` (Recording.cpp writeRaw/readRaw), except we copy the
// struct in one shot rather than field by field. The driver and the app are
// both MSVC on x86/x64, so the natural-alignment padding is identical on both
// ends; the framing layer keys on `sizeof(struct)`, so a few pad bytes on the
// wire cost nothing. There is no codec: the caller hands `&struct` and
// `sizeof(struct)` to the channel's byte-oriented `send`, and `memcpy`s a
// received payload back into the struct.
//
// A frame header (u32 length, u16 type) prefixes each payload; the framing
// layer (MessageChannel) splits the stream on length alone, so an unknown
// `type` is skipped by length, not rejected — that is how an older app
// survives a newer driver. A `length` above what fits the `Message` payload
// union is a protocol error: the stream is unrecoverable.
//
// `link` links neither model code nor glm nor openvr. `DeviceKind` is this
// layer's own enum with pinned wire values; the driver maps
// `vr::ETrackedDeviceClass` -> it, the app maps it -> `TrackedDeviceKind`.

#include <cstdint>

namespace link
{
// Pinned wire values for the `deviceKind` field of DevicePose. Do not renumber:
// an existing driver speaks the old numbers.
enum class DeviceKind : std::uint8_t
{
    Hmd = 0,
    Controller = 1,
    Tracker = 2,
    Other = 3,
};

// `tracking` field of DevicePose, collapsed from the two booleans the app
// currently ANDs (the old client-API poll: `bPoseIsValid && bDeviceIsConnected`).
// Zero means "drop this device from the snapshot this frame"; the app treats
// a missing device as holding its last pose (Recording.h), so a vanishing
// tracker freezes its target rather than snapping it.
enum class TrackingState : std::uint8_t
{
    Lost = 0,
    Tracking = 1,
};

// Message types. Their wire values are pinned; a future type gets the next
// free number. Unknown types are skipped by the framing layer, not rejected.
// `DeviceMetadata` (1) was removed in step 3 — its fields folded into
// `DevicePose` — so a frame with type 1 from an older driver is now silently
// skipped by a newer app. `VirtualTracker` (4) was added in step 5 for the
// app -> driver virtual-tracker stream.
enum class MessageType : std::uint16_t
{
    DevicePose = 2,
    PoseOverride = 3,
    VirtualTracker = 4,
};

// Largest serial string carried on the wire. OpenVR serials ("LHR-XXXXXXXX")
// are short; 32 bytes is ample and keeps the pose POD compact.
inline constexpr std::uint32_t kMaxSerialBytes = 32;

// Largest bone name carried on a `VirtualTracker` frame. Unity bone names
// ("LeftUpperLeg", "RightLowerArm") are short; 32 bytes matches the serial
// field width so one cap serves both.
inline constexpr std::uint32_t kMaxBoneNameBytes = 32;

// A 3-component vector on the wire (f32 x3, sizeof == 12).
struct WireVec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

// A quaternion on the wire (f32 x4, sizeof == 16). Stored xyzw — the same
// component order OpenVR's `HmdQuaternionf_t` uses and the order glm's
// `quat(w,x,y,z)` constructor accepts positionally. Default = identity.
struct WireQuat
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;
};

// Wire POD. Fields are explicit-width; natural-alignment padding is the same
// on both ends (MSVC x64), so `sizeof(struct)` is the on-wire payload length.
//   DevicePose: u32 deviceId, u8 tracking, u8 deviceKind, 2 pad,
//               WireVec3 position, WireQuat rotation, char serial[32].
struct DevicePose
{
    std::uint32_t deviceId = 0;
    TrackingState tracking = TrackingState::Lost;
    DeviceKind deviceKind = DeviceKind::Other;
    WireVec3 position;
    WireQuat rotation;
    char serial[kMaxSerialBytes] = {};
};

// Upstream (app -> driver) correction for one device. A rigid world-space delta
// from the device's raw pose to its corrected pose: `compose(delta, rawWorld)`
// yields the corrected world pose. The driver applies it by premultiplying
// `worldFromDriver` (the only `DriverPose_t` fields it touches), so vrserver's
// pose prediction — which runs in the driver-local frame on the untouched local
// pose and velocities — stays exact.
//
// Layout: u32 deviceId, WireVec3 position, WireQuat rotation
// -> sizeof == 32 on MSVC x86/x64, memcpy'd whole like DevicePose.
struct PoseOverride
{
    std::uint32_t deviceId = 0;
    WireVec3 position;
    WireQuat rotation;
};

// Upstream (app -> driver) virtual-tracker frame for one bone
// (doc/virtual-trackers-plan.md step 5). The app sends one per ticked eligible
// bone each frame while in Capture mode with calibration complete; the set of
// names arriving in a frame is the roster, so the driver has no compile-time
// slot list. `tracking` is `Tracking` on every frame the app sends (the app
// simply stops sending when it leaves Capture, and the driver's staleness
// timeout disconnects the device); the field is kept for symmetry with
// `DevicePose` and forward use.
//
// Layout: char name[32], u8 tracking, 3 pad, WireVec3 position, WireQuat rotation
// -> sizeof == 64 on MSVC x86/x64, memcpy'd whole like the others.
struct VirtualTracker
{
    char name[kMaxBoneNameBytes] = {};
    TrackingState tracking = TrackingState::Tracking;
    WireVec3 position;
    WireQuat rotation;
};

// A frame on the wire and in memory: a u32 payload length, a u16 type, then the
// payload union. The length is the payload length (not the whole frame) and is
// filled by the sender. The framing layer reads the header, then exactly `size`
// payload bytes; a `size` larger than the union is a protocol error. Three
// payload shapes exist today (`DevicePose` downstream, `PoseOverride` and
// `VirtualTracker` upstream); a future type adds a union member.
struct Message
{
    std::uint32_t size = 0;
    MessageType type = MessageType::DevicePose;
    union
    {
        DevicePose devicePose{};
        PoseOverride poseOverride;
        VirtualTracker virtualTracker;
    };
};
} // namespace link