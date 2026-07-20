#pragma once

#include "ModeController.h"
#include "Recording.h"
#include "TrackedDevice.h"

#include <functional>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

// Automatic capture-session recording lifecycle, UI- and file-system-free:
// streams come from an injected factory, so tests run against string streams
// and the app supplies "open recording.tcrec truncated". The model layer is
// log-free, so outcomes are reported through the returned Event and the
// caller (main.cpp) logs them.
class SessionRecorder
{
public:
    // Opens a fresh stream for a new session. Returning null or a failed
    // stream (or throwing) aborts that session's recording, reported via
    // Event::error.
    using StreamFactory = std::function<std::shared_ptr<std::ostream>()>;

    // What happened this frame — for the caller to log.
    struct Event
    {
        bool started = false;  // a new recording began (frame 0 written)
        bool stopped = false;  // the recording closed cleanly
        std::string error;     // non-empty: recording failed and stopped
    };

    explicit SessionRecorder(StreamFactory openStream) : openStream_(std::move(openStream)) {}

    // Advances the recorder by one frame; call right after
    // ModeController::update with that frame's inputs. The calibration ->
    // capture transition (capturedOffsets true, mode already Capture) starts
    // a new recording whose frame 0 holds exactly `devices` — the snapshot
    // calibration froze offsets from; every further Capture frame appends
    // `devices` at time `now - start`; leaving Capture stops the recording.
    // `now` is an absolute clock in seconds (the caller's frame time).
    // Failures never throw — a broken recording must not break the capture —
    // they stop the recording and surface in the returned Event.
    Event update(Mode mode, bool capturedOffsets, double now,
                 const std::vector<TrackedDevice>& devices);

    bool isRecording() const { return writer_.has_value(); }

private:
    void stop();

    StreamFactory openStream_;
    std::shared_ptr<std::ostream> stream_;   // outlives writer_ (declared first)
    std::optional<RecordingWriter> writer_;
    double startTime_ = 0.0;
};
