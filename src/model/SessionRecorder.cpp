#include "SessionRecorder.h"

#include "Error.h"

SessionRecorder::Event SessionRecorder::update(Mode mode, bool capturedOffsets, double now,
                                                const std::vector<TrackedDevice>& devices,
                                                const std::vector<GripOffset>& gripOffsets)
{
    Event event;

    if (capturedOffsets)
    {
        stop();  // a stale session must never survive a new calibration
        try
        {
            stream_ = openStream_();
            if (!stream_ || !*stream_)
                throw Error("cannot open the recording stream");
            writer_.emplace(*stream_);
            startTime_ = now;
            writer_->writeFrame(0.0f, devices, gripOffsets);
            event.started = true;
        }
        catch (const std::exception& e)
        {
            event.error = e.what();
            stop();
        }
    }
    else if (mode == Mode::Capture && writer_)
    {
        try
        {
            writer_->writeFrame(static_cast<float>(now - startTime_), devices, gripOffsets);
        }
        catch (const std::exception& e)
        {
            event.error = e.what();
            stop();
        }
    }

    if (mode != Mode::Capture && writer_)
    {
        stop();
        event.stopped = true;
    }

    return event;
}

void SessionRecorder::stop()
{
    writer_.reset();
    stream_.reset();
}
