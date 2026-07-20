#include "ReplaySession.h"

#include "Error.h"

#include <algorithm>
#include <fstream>

void ReplaySession::scan(const std::filesystem::path& directory)
{
    reset();
    directory_ = directory;
    try
    {
        for (const auto& entry : std::filesystem::directory_iterator(directory))
        {
            if (entry.is_regular_file() && entry.path().extension() == kRecordingFileExtension)
                files_.push_back(entry.path().filename().string());
        }
    }
    catch (const std::exception& e)
    {
        files_.clear();
        throw Error(std::string("cannot scan for recordings: ") + e.what());
    }
    std::sort(files_.begin(), files_.end());
}

void ReplaySession::load(size_t index, ModeController& controller, IkRig& rig)
{
    recording_ = {};
    loadedIndex_ = -1;
    frameIndex_ = 0;
    if (index >= files_.size())
        throw Error("recording index " + std::to_string(index) + " out of range (have "
                    + std::to_string(files_.size()) + " files)");

    const std::filesystem::path path = directory_ / files_[index];
    std::ifstream file(path, std::ios::binary);
    if (!file)
        throw Error("cannot open recording file: " + path.string());
    Recording recording = loadRecording(file);

    // Frame 0 is the exact device snapshot the live session calibrated from;
    // re-running the live calibration path reproduces its offsets exactly.
    controller.calibrateFromFrame(rig, recording.frames.front().devices);
    recording_ = std::move(recording);
    loadedIndex_ = static_cast<int>(index);
}

void ReplaySession::reset()
{
    directory_.clear();
    files_.clear();
    loadedIndex_ = -1;
    recording_ = {};
    frameIndex_ = 0;
}

void ReplaySession::seek(float time)
{
    if (recording_.frames.empty())
        return;
    frameIndex_ = nearestFrameIndex(recording_, time);
}

float ReplaySession::frameTime() const
{
    return recording_.frames.empty() ? 0.0f : recording_.frames[frameIndex_].time;
}

std::vector<TrackedDevice> ReplaySession::currentDevices() const
{
    return recording_.frames.empty() ? std::vector<TrackedDevice>{}
                                     : recording_.frames[frameIndex_].devices;
}
