#pragma once

// Wire types for the driver link (doc/driver-plan.md phase A, step 2).
//
// Two messages cover everything the app consumes from the tracking provider
// today (see OpenVrTracking.cpp pollPoses): device metadata (its kind — the
// only thing the app knows a device by besides its id), and the per-device
// pose (id + position + rotation + the one validity predicate). Nothing else
// from `DriverPose_t` is read downstream yet, so nothing else is on the wire.
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
// survives a newer driver. A `length > kMaxPayloadBytes` is a protocol error:
// the stream is unrecoverable.
//
// `link` links neither model code nor glm nor openvr. `DeviceKind` is this
// layer's own enum with pinned wire values; the driver maps
// `vr::ETrackedDeviceClass` -> it, the app maps it -> `TrackedDeviceKind`.

#include <cstdint>

namespace link
{
// Pinned wire values for the `kind` field of DeviceMetadata. Do not renumber:
// an existing driver speaks the old numbers.
enum class DeviceKind : std::uint8_t
{
    Hmd = 0,
    Controller = 1,
    Tracker = 2,
    Other = 3,
};

// `tracking` field of DevicePose, collapsed from the two booleans the app
// currently ANDs (OpenVrTracking.cpp: `bPoseIsValid && bDeviceIsConnected`).
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
enum class MessageType : std::uint16_t
{
    DeviceMetadata = 1,
    DevicePose = 2,
};

// Largest payload the framing layer will accept. Frames above this are a
// protocol error rather than a buffer to grow into — a pose is ~36 bytes,
// metadata is ~8, and a 64 KiB cap leaves headroom without admitting a runaway
// allocation.
inline constexpr std::uint32_t kMaxPayloadBytes = 65536;

// Wire PODs. Fields are explicit-width; natural-alignment padding is the same
// on both ends (MSVC x64), so `sizeof(struct)` is the on-wire payload length.
//   DeviceMetadata: u32 deviceId, u8 kind.
//   DevicePose:     u32 deviceId, u8 tracking, f32 pos[3], f32 rot[4] (xyzw).
struct DeviceMetadata
{
    std::uint32_t deviceId = 0;
    DeviceKind kind = DeviceKind::Other;
};

struct DevicePose
{
    std::uint32_t deviceId = 0;
    TrackingState tracking = TrackingState::Lost;
    float position[3] = {0.0f, 0.0f, 0.0f};
    float rotation[4] = {0.0f, 0.0f, 0.0f, 1.0f};  // xyzw, identity
};
} // namespace link