#pragma once

#include "ModeController.h"
#include "Recording.h"
#include "TrackedDevice.h"

#include <filesystem>
#include <string>
#include <vector>

class IkRig;

// Replay-mode session state, UI- and hardware-free: which recording files
// exist, which one is loaded, and which frame the timeline points at.
// main.cpp only binds the UI to this and logs the Errors it throws (the
// model layer is log-free).
class ReplaySession
{
public:
    // Scans `directory` for recording files (kRecordingFileExtension, sorted
    // by name), replacing the file list and dropping any loaded recording.
    // Throws Error when the directory cannot be read (the list is then empty).
    void scan(const std::filesystem::path& directory);

    // Loads files()[index] and recalibrates the controller/rig from the
    // recording's first frame — the exact device snapshot the live session's
    // calibration froze offsets from — then seeks to frame 0. Throws Error
    // on failure; the session then holds no recording (the file list stays).
    void load(size_t index, ModeController& controller, IkRig& rig);

    // Drops the file list and any loaded recording.
    void reset();

    // Snaps the timeline to the frame nearest to `time`; no-op without a
    // recording.
    void seek(float time);

    const std::vector<std::string>& files() const { return files_; }
    int loadedIndex() const { return loadedIndex_; }  // -1 = nothing loaded
    bool hasRecording() const { return !recording_.frames.empty(); }
    const Recording& recording() const { return recording_; }
    size_t frameIndex() const { return frameIndex_; }

    // Time of the current frame (the timeline slider value); 0 without a
    // recording.
    float frameTime() const;

    // Devices of the current frame, ready to feed to ModeController::update;
    // empty without a recording (the skeleton idles at rest).
    std::vector<TrackedDevice> currentDevices() const;

private:
    std::filesystem::path directory_;
    std::vector<std::string> files_;
    int loadedIndex_ = -1;
    Recording recording_;
    size_t frameIndex_ = 0;
};
