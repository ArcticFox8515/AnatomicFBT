#pragma once

#include "GripOffsets.h"
#include "TrackedDevice.h"

#include <cstddef>
#include <iosfwd>
#include <vector>

// Binary capture-session recording: the exact device snapshots that were fed
// into the mode state machine, so a replay is indistinguishable from a live
// session to the calibration and the solver.
//
// File format (little-endian, packed, extension .tcrec):
//   header:  u32 magic 'TCR1', u16 version (2)
//   roster:  u32 deviceCount, then per device:
//              i32 id, u8 kind, f32x3 gripPos, f32x4 gripRot (xyzw)
//              — the per-controller grip offset (identity for non-controllers)
//   frame*:  f32 time (absolute seconds, frame 0 = 0),
//            then deviceCount x (f32x3 position, f32x4 rotation xyzw)
//            in roster order — until EOF
// Frames are fixed-size, so the frame count and total length are derivable
// from the file size and the last frame's time without scanning; the absolute
// per-frame time also makes timeline seeking trivial. Frame 0 is the exact
// calibration input (the roster paired with frame 0's poses), which lets a
// replay recalibrate from the file with different skeletons/solvers.
// A device that loses tracking mid-session is written with its last known
// pose (the live effect of a dropout — the target does not move), so every
// frame is self-contained and randomly seekable.
//
// Frames are grip-applied: each loaded frame's controller poses are already
// composed with the roster's `deviceToGrip` offset (same as the live path
// does via `applyGripOffsets` before feeding the mode controller), so replay
// reproduces the live shift exactly without SteamVR. The roster carries the
// raw-to-grip transform per device so a future tool can recover the raw pose.
//
// Version 1 (the original format, no grip offsets) is still accepted on load:
// the roster entry was `i32 id, u8 kind`, and every grip offset reads as
// identity. Old recordings keep working; new recordings are v2.

// One loaded frame: the full roster with poses, ready to feed to
// ModeController::update as if it came from the tracking provider. Controller
// poses are already grip-applied (see above).
struct RecordingFrame
{
    float time = 0.0f;  // seconds since the calibration frame (non-decreasing)
    std::vector<TrackedDevice> devices;  // full roster, in roster order
};

// File extension recordings are saved with and scanned for.
constexpr char kRecordingFileExtension[] = ".tcrec";

// Default recording file path (overwritten per capture session).
constexpr char kRecordingPath[] = "recording.tcrec";

struct Recording
{
    std::vector<RecordingFrame> frames;
    // Per-roster-device grip offset (identity for non-controllers). Kept on
    // the loaded recording for inspection/testing; the frames themselves
    // already carry grip-applied poses.
    std::vector<GripOffset> gripOffsets;

    float duration() const { return frames.empty() ? 0.0f : frames.back().time; }
};

// Streaming writer: nothing is buffered beyond the roster's last known poses,
// so a crash loses at most the trailing frame. The first writeFrame call
// establishes the roster (device ids + kinds + grip offsets, in the order
// given) and writes the header; devices absent from a later frame are
// written with their last known pose, devices not in the roster are ignored
// (nothing downstream of calibration consumes them). The grip offsets are
// only consulted on frame 0 (the roster freeze) — later frames carry only
// poses. Throws Error when the stream fails.
class RecordingWriter
{
public:
    explicit RecordingWriter(std::ostream& out) : out_(out) {}

    void writeFrame(float time, const std::vector<TrackedDevice>& devices,
                    const std::vector<GripOffset>& gripOffsets);

private:
    std::ostream& out_;
    std::vector<TrackedDevice> roster_;  // ids + kinds + last known poses
};

// Parses a recording stream. Throws Error on bad magic, an unsupported
// version, a malformed roster, or when not a single complete frame is
// present (a recording without its calibration frame is useless). A
// truncated trailing frame (crashed session) is silently dropped. Accepts
// both v1 (no grip offsets in the roster — read as identity) and v2.
Recording loadRecording(std::istream& in);

// Index of the frame whose time is closest to `time` (ties resolve to the
// earlier frame). Frames must be non-decreasing in time. Throws Error when
// the recording has no frames.
size_t nearestFrameIndex(const Recording& recording, float time);
