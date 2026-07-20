#pragma once

#include "TrackedDevice.h"

#include <cstddef>
#include <iosfwd>
#include <vector>

// Binary capture-session recording: the exact device snapshots that were fed
// into the mode state machine, so a replay is indistinguishable from a live
// session to the calibration and the solver.
//
// File format (little-endian, packed, extension .tcrec):
//   header:  u32 magic 'TCR1', u16 version (1)
//   roster:  u32 deviceCount, then per device: i32 id, u8 kind
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

// One loaded frame: the full roster with poses, ready to feed to
// ModeController::update as if it came from the tracking provider.
struct RecordingFrame
{
    float time = 0.0f;  // seconds since the calibration frame (non-decreasing)
    std::vector<TrackedDevice> devices;  // full roster, in roster order
};

// File extension recordings are saved with and scanned for.
constexpr char kRecordingFileExtension[] = ".tcrec";

struct Recording
{
    std::vector<RecordingFrame> frames;

    float duration() const { return frames.empty() ? 0.0f : frames.back().time; }
};

// Streaming writer: nothing is buffered beyond the roster's last known poses,
// so a crash loses at most the trailing frame. The first writeFrame call
// establishes the roster (device ids + kinds, in the order given) and writes
// the header; devices absent from a later frame are written with their last
// known pose, devices not in the roster are ignored (nothing downstream of
// calibration consumes them). Throws Error when the stream fails.
class RecordingWriter
{
public:
    explicit RecordingWriter(std::ostream& out) : out_(out) {}

    void writeFrame(float time, const std::vector<TrackedDevice>& devices);

private:
    std::ostream& out_;
    std::vector<TrackedDevice> roster_;  // ids + kinds + last known poses
};

// Parses a recording stream. Throws Error on bad magic/version, a malformed
// roster, or when not a single complete frame is present (a recording without
// its calibration frame is useless). A truncated trailing frame (crashed
// session) is silently dropped.
Recording loadRecording(std::istream& in);

// Index of the frame whose time is closest to `time` (ties resolve to the
// earlier frame). Frames must be non-decreasing in time. Throws Error when
// the recording has no frames.
size_t nearestFrameIndex(const Recording& recording, float time);
