#pragma once

// Shim: the logger moved into the link layer (src/link/Log.h, namespace `link`). The
// spike keeps its historical `namespace spike` spellings through these using-declarations
// so existing callers and tests compile unchanged. A forward declaration cannot
// coexist with a using-declaration (SpikeClientReport.h includes this shim instead).

#include "link/Log.h"

namespace spike
{
using link::LogSink;
using link::compositeSink;
using link::Logger;
using link::log;
using link::loggingTo;
} // namespace spike
