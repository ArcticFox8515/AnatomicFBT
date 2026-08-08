#pragma once

// Wire types for the driver link (doc/driver-plan.md phase A, step 3).
//
// A single message — `DevicePose` — covers everything the app consumes from the
// tracking provider today (see OpenVrTracking pollPoses): the device's kind
// and serial (the only things the app knows a device by besides its id) and the
// per-device pose (id + position + rotation + the one validity predicate). The
// separate metadata message was folded into the pose in step 3 so the channel
// carries one frame per device update, not two. Nothing else from
// `DriverPose_t` is read downstream yet, so nothing else is on the wire.
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
// skipped by a newer app.
enum class MessageType : std::uint16_t
{
    DevicePose = 2,
};

// Largest serial string carried on the wire. OpenVR serials ("LHR-XXXXXXXX")
// are short; 32 bytes is ample and keeps the pose POD compact.
inline constexpr std::uint32_t kMaxSerialBytes = 32;

// Wire POD. Fields are explicit-width; natural-alignment padding is the same
// on both ends (MSVC x64), so `sizeof(struct)` is the on-wire payload length.
//   DevicePose: u32 deviceId, u8 tracking, u8 deviceKind, 2 pad,
//               f32 pos[3], f32 rot[4] (xyzw), char serial[32].
struct DevicePose
{
    std::uint32_t deviceId = 0;
    TrackingState tracking = TrackingState::Lost;
    DeviceKind deviceKind = DeviceKind::Other;
    float position[3] = {0.0f, 0.0f, 0.0f};
    float rotation[4] = {0.0f, 0.0f, 0.0f, 1.0f};  // xyzw, identity
    char serial[kMaxSerialBytes] = {};
};

// A frame on the wire and in memory: a u32 payload length, a u16 type, then the
// payload union. The length is the payload length (not the whole frame) and is
// filled by the sender. The framing layer reads the header, then exactly `size`
// payload bytes; a `size` larger than the union is a protocol error. Only one
// payload shape exists today (`DevicePose`); a future type adds a union member.
struct Message
{
    std::uint32_t size = 0;
    MessageType type = MessageType::DevicePose;
    union
    {
        DevicePose pose{};
    };
};
} // namespace link