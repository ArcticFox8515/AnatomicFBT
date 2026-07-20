#include "Recording.h"

#include "Error.h"

#include <algorithm>
#include <cstdint>
#include <istream>
#include <ostream>

namespace
{
constexpr std::uint32_t kMagic = 0x31524354;  // "TCR1" as little-endian bytes
constexpr std::uint16_t kVersion = 1;

// Sanity bound for the roster count read from a file — anything larger is
// certainly garbage, and rejecting it avoids absurd allocations.
constexpr std::uint32_t kMaxRosterSize = 1024;

template <typename T>
void writeRaw(std::ostream& out, const T& value)
{
    out.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

void writePose(std::ostream& out, const Pose& pose)
{
    writeRaw(out, pose.position.x);
    writeRaw(out, pose.position.y);
    writeRaw(out, pose.position.z);
    writeRaw(out, pose.rotation.x);
    writeRaw(out, pose.rotation.y);
    writeRaw(out, pose.rotation.z);
    writeRaw(out, pose.rotation.w);
}

// Reads one POD value; returns false on a clean or mid-value EOF (the caller
// decides whether that is truncation or the end of the frame list).
template <typename T>
bool readRaw(std::istream& in, T& value)
{
    in.read(reinterpret_cast<char*>(&value), sizeof(T));
    return in.gcount() == sizeof(T);
}

bool readPose(std::istream& in, Pose& pose)
{
    return readRaw(in, pose.position.x) && readRaw(in, pose.position.y)
        && readRaw(in, pose.position.z) && readRaw(in, pose.rotation.x)
        && readRaw(in, pose.rotation.y) && readRaw(in, pose.rotation.z)
        && readRaw(in, pose.rotation.w);
}

TrackedDeviceKind kindFromByte(std::uint8_t byte)
{
    switch (byte)
    {
    case static_cast<std::uint8_t>(TrackedDeviceKind::Hmd):
        return TrackedDeviceKind::Hmd;
    case static_cast<std::uint8_t>(TrackedDeviceKind::Controller):
        return TrackedDeviceKind::Controller;
    case static_cast<std::uint8_t>(TrackedDeviceKind::Tracker):
        return TrackedDeviceKind::Tracker;
    case static_cast<std::uint8_t>(TrackedDeviceKind::Other):
        return TrackedDeviceKind::Other;
    default:
        throw Error("recording roster has an unknown device kind: "
                    + std::to_string(static_cast<int>(byte)));
    }
}
} // namespace

void RecordingWriter::writeFrame(float time, const std::vector<TrackedDevice>& devices)
{
    if (roster_.empty())
    {
        // First frame: freeze the roster and write header + roster. This
        // frame's devices are the exact calibration input.
        roster_ = devices;
        writeRaw(out_, kMagic);
        writeRaw(out_, kVersion);
        writeRaw(out_, static_cast<std::uint32_t>(roster_.size()));
        for (const TrackedDevice& device : roster_)
        {
            writeRaw(out_, static_cast<std::int32_t>(device.id));
            writeRaw(out_, static_cast<std::uint8_t>(device.kind));
        }
    }
    else
    {
        // Later frames: refresh last known poses; absent devices keep theirs
        // (the live dropout behavior), unknown devices are ignored.
        for (TrackedDevice& entry : roster_)
            for (const TrackedDevice& device : devices)
                if (device.id == entry.id)
                {
                    entry.pose = device.pose;
                    break;
                }
    }

    writeRaw(out_, time);
    for (const TrackedDevice& entry : roster_)
        writePose(out_, entry.pose);

    if (!out_)
        throw Error("recording stream write failed");
}

Recording loadRecording(std::istream& in)
{
    std::uint32_t magic = 0;
    if (!readRaw(in, magic) || magic != kMagic)
        throw Error("not a recording: bad magic");
    std::uint16_t version = 0;
    if (!readRaw(in, version) || version != kVersion)
        throw Error("unsupported recording version: " + std::to_string(version));

    std::uint32_t rosterSize = 0;
    if (!readRaw(in, rosterSize) || rosterSize > kMaxRosterSize)
        throw Error("recording roster is malformed");
    std::vector<TrackedDevice> roster(rosterSize);
    for (TrackedDevice& device : roster)
    {
        std::int32_t id = 0;
        std::uint8_t kind = 0;
        if (!readRaw(in, id) || !readRaw(in, kind))
            throw Error("recording roster is truncated");
        device.id = id;
        device.kind = kindFromByte(kind);
    }

    Recording recording;
    for (;;)
    {
        RecordingFrame frame;
        if (!readRaw(in, frame.time))
            break;  // clean end of the frame list (or a dropped partial time)
        frame.devices = roster;
        bool complete = true;
        for (TrackedDevice& device : frame.devices)
            if (!readPose(in, device.pose))
            {
                complete = false;  // truncated trailing frame: drop it
                break;
            }
        if (!complete)
            break;
        recording.frames.push_back(std::move(frame));
    }

    if (recording.frames.empty())
        throw Error("recording has no complete frames");
    return recording;
}

size_t nearestFrameIndex(const Recording& recording, float time)
{
    const std::vector<RecordingFrame>& frames = recording.frames;
    if (frames.empty())
        throw Error("nearestFrameIndex on an empty recording");

    const auto after = std::lower_bound(
        frames.begin(), frames.end(), time,
        [](const RecordingFrame& frame, float t) { return frame.time < t; });
    if (after == frames.begin())
        return 0;
    if (after == frames.end())
        return frames.size() - 1;
    const auto before = after - 1;
    // Ties resolve to the earlier frame (<=).
    if (time - before->time <= after->time - time)
        return static_cast<size_t>(before - frames.begin());
    return static_cast<size_t>(after - frames.begin());
}
