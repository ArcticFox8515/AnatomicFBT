# Handover: SteamVR driver spike (step 1 of `doc/driver-plan.md`)

Written at the end of session 4, which **ran the spike driver inside a live SteamVR
session** and analysed the telemetry. Step 1 has now served its purpose: the questions
it existed to answer are answered (§3), and what remains is a defect list (§5), not
exploration.

Read `doc/driver-plan.md` first, then this file, then `AGENTS.md`.

**The test suite is green as of session 7.** The three deliberate failures below
were fixed in session 5 (§5.2 install race + §5.4 invalid-pose gate); §5.1 (input
hooks → `PollNextEvent`) and §5.3 (logger rewrite) landed in session 5, then §5.1
was reversed in session 6 (input capture removed from the driver DLL entirely —
see §5.1). Kept here for the history of what the spike once violated:

- `SpikeHooksTest.AnInstallReenteredDuringCreateKeepsAnOriginalToCallThrough`
- `SpikeHooksTest.AnInstallReenteredDuringCreateHooksTheFunctionOnlyOnce`
- `SpikeObserverTest.APoseSteamVRReportsAsInvalidIsNotComposed`

Debug: 266 passed, 0 failed, 266 total. Release: 266 passed, 0 failed.

Session 7 closed the last §2.1a violations the §5.3 logger rewrite had left in the
adapters: the log-file path (basename scan, extension strip, unknown-process and
`%LOCALAPPDATA%`-unset fallbacks — duplicated in `SpikeClient.cpp`), the two-sink
composition, and the install-the-sink-once decision are now `SpikeLogFile.*` /
`compositeSink` / `loggingTo` in `SpikeLib`, with unit tests over a `ProcessApi` fake.
`HookApi::isAlreadyInitialized` became `alreadyInitializedStatus()` for the same
reason: the comparison it hid was the adapter's only one. `std::filesystem::
create_directories` is gone from both adapters — spdlog's file sink creates the
directory when it opens the file.

---

## 1. Why this file exists

The spike DLL runs **inside `vrserver.exe`**, which SteamVR may run elevated. A crash, a
hang or a data race in our code takes down the user's whole VR session, and a `throw`
escaping into vrserver kills it outright. Code review alone is not an acceptable gate
for that. The rules in §2 were set by the repo owner and are non-negotiable for
everything under `src/spike/` and, later, `src/driver*/`.

---

## 2. Hard rules for this work

### 2.1 Line coverage bar

> Every line of code must be executed at least once, with at least one set of
> parameters, during the test run. The only exemption is pass-through/pimpl-style
> forwarding (`void f(int a) { impl->f(a); }`).

Not "every case" — every **line**, including error branches, early returns, log
branches and the standby/watchdog entry points. If a line cannot be reached from a test,
that is a design smell: restructure until it can.

This must be **measured**, not argued. Proposed tool: **OpenCppCoverage** (free, MSVC
PDBs, HTML + Cobertura), driven by a `run-coverage.bat` next to `run-tests.bat`. It is
not a Conan package, so installing it is the owner's decision. Until then, every coverage
claim in this repo is reading plus mutation spot-checks, and must be labelled as such.

### 2.1a The adapter exemption is narrow

`SpikeDriver.cpp` is compiled into `driver_00trackingcorrector` **and nothing else**. No
test target compiles it, so no unit test can execute a line of it and a coverage run will
report it as 0%. `SpikeDriverTest` reaches it only through `LoadLibrary`, which is an
integration test and therefore not evidence about decisions.

> Every function in `SpikeDriver.cpp` is **one instruction**. Two at most, and only when
> the first creates the implementation object and the second calls a method on it. No
> branch, no loop, no comparison, no arithmetic — if you are writing one, it belongs in
> `SpikeLib` with a test.

`SpikeClient.cpp` is the same kind of file, and is not executed by any test at all.

### 2.1b Unit tests must not depend on the machine

A spike test may not touch the **filesystem, the wall clock, the environment, or any real
thread**. The result of a test that does is a property of the machine it ran on and proves
nothing about code inside `vrserver.exe`. Concretely: streams are `std::ostringstream`,
time is an injected `double`, MinHook is a fake `HookApi`, the environment arrives as a
value. The one exception is `SpikeDriverTest`, which loads the real DLL and so lets the
DLL open its own log file.

**Corollary, established in session 4:** a concurrency defect that can only be shown with
real threads is a defect in *our* design, not a limit of testing. Both hook-install races
below reproduce single-threaded, by letting the fake `HookApi` re-enter `install()` from
inside `create()` — which is exactly the interleaving a second thread produces. No
threads in tests.

### 2.2 Concurrency: proof, not tests

For every piece of shared mutable state, the handover/PR must contain a written
enumeration of **every** access path (which thread, which entry point, holding which
lock) and an argument for why no interleaving is harmful. §6 holds that document. It is
incomplete.

### 2.3 Process rules (violations happened; do not repeat)

- Answer the question that was asked **before** changing code. Refactors and experiments
  need agreement first.
- **One topic per message.** If a reply starts on issue N it ends on issue N.
- Edit source files **only** with the edit/write tools. Never shell `-replace` /
  `sed`-style munging: a regex escape wrote `\&\&` into `SpikeInterfaces.h` and broke the
  build in session 1, and a PowerShell `Set-Content` round-trip re-encoded an em dash in
  `AGENTS.md` into `U+FFFD` in session 2.
- Never write outside the workspace. The only sanctioned external path is
  `C:\Users\ARCTIC~1\AppData\Local\Temp\opencode`. A stray backup is still at
  `%TEMP%\SpikeInterfaces.bak` — delete it (pending, needs owner's OK).
- Do not say "verified" for something that was only compiled or only loaded. State
  exactly what was executed.
- **Close Steam, not just SteamVR, before building the Release driver** (§8).

---

## 3. What the live run established

Session 4, ~86 s session, one Meta Quest 3 (`oculus` driver) plus 7 lighthouse devices
(2 Index controllers, a Tundra Tracker, a VIVE Tracker 3.0, 3 base stations), with
`spike_client` sampling alongside. Logs:
`%LOCALAPPDATA%\TrackingCorrector\driver-spike-vrserver.log`,
`client-spike-spike_client.log`, `driver-spike-Steam.log`.

- **The pose composition is `B`**: `world = worldFromDriver ∘ (vecPosition, qRotation) ∘
  driverFromHead`, i.e. what the `DriverPose_t` comments say. Evidence: device 1 driver
  `B` = (1.1037, 1.3424, 0.6518) @18:26:50.267 against client raw
  (1.0915, 1.3168, 0.6379) @18:26:50.979, while `A` = (1.2051, 1.2201, 0.6546) and its
  quaternion is unrelated. The candidates differ by the whole `driverFromHead`
  translation, 0.10-0.15 m on a controller, an order of magnitude above sampling skew and
  hardware noise. Same verdict independently on device 4. **Record this in
  `doc/driver-plan.md` "Verified groundwork" — step 3's `DriverPoseMath` depends on it.**
- **Cross-driver visibility works.** All 8 devices, belonging to two foreign drivers,
  arrived through hooks we installed; the driver-side device set and ids match what
  `spike_client` saw client-side. The `00` prefix plus the `GetGenericInterface` detour
  does what `doc/spacecalibrator-notes.md` promised.
- **Driver-side metadata by index works** for devices we do not own: serial, class, role
  hint, model, tracking system, container handle. Mid-session arrivals are picked up
  within one housekeeping second.
- **Rates**: `RunFrame` ~95-104 Hz (7710 calls); HMD poses 71.8 Hz; each lighthouse
  device 370-410 Hz, so roughly **1.6 k pose callbacks/s across 4 devices**, on driver
  threads. This is the number the concurrency and locking work has to survive.
- **`unPoseStructSize`** was always `sizeof(DriverPose_t)` = 280; the truncated-struct
  warning never fired.
- **Hook lifecycle is clean**: six hooks installed on the plan's vtable indices, removed
  in reverse order at `Cleanup`, no crash, no hang, no SteamVR complaint.
- **Standby happens** (`EnterStandby` at 18:27:04) and matters: afterwards devices keep
  sending poses with `poseIsValid=0` and `poseTimeOffset` around **-80 s**. Stale poses
  are a real input, not a hypothetical.
- **`poseIsValid=1` does not mean sane.** The Tundra Tracker reported
  (1305.50, 7837.01, -4230.18) — ~9 km — with `result=Running_OK`, driver-side *and*
  client-side, for its whole valid stretch. Base stations send `qRotation` all zeros,
  which is not a rotation. The real driver needs validity, quaternion and magnitude gates
  before anything reaches the solver.
- **Interface versions in the wild are not the ones we build against.** vrserver handed
  out `IVRServerDriverHost_005` and `IVRDriverInput_004` alongside the `_006` / `_003` we
  hook. No device was lost to `_005`. `_004` (no calls on the hooked `_003`) is what
  first drove §5.1; the §5.1 reversal removed input capture from the driver entirely.
- **`raw` ≈ `standing` on this machine** (differences ~1e-4), so the run did *not*
  discriminate the two universes. Do not read it as confirmation of the coordinate-space
  assumption in `doc/driver-plan.md`; that assumption is still an assumption.

---

## 4. Claims from session 4 that were investigated and withdrawn

Recorded so they are not raised again:

- *"The spike cannot settle A vs B from its own logs because the two samplers are
  unsynchronized."* The samplers are unsynchronized and it does not matter: the
  separation being measured is 0.10-0.15 m, the sampling skew costs centimetres. The
  claim also leaned on the HMD's logged identity transforms as if that were a property of
  the design.
- *"Our `RunFrame` work caused the `RunFrame` collapse to 11.5 Hz at 18:26:34."* It did
  not. We emitted 180 log lines in that 10 s window and ~490 lines per 10 s afterwards
  while `RunFrame` sat at 100 Hz, and the per-second property scan covers all 64 slots
  regardless of device count. The dip tracks device *activation* — vrserver's own work —
  and `doc/driver-plan.md` already reasons that a slow `RunFrame` costs us nothing but
  delayed metadata.

---

## 5. Work for the next session

In this order.

### 5.1 Remove the input hooks; then remove input capture from the driver entirely — DONE (session 5), reversed (session 6)

**Session 5 — `PollNextEvent` substitute:** the `IVRDriverInput_003` hooks observed
nothing. `IVRDriverInput_003` was hooked from our own eager fetch at 18:25:43.877;
the drivers that matter asked for `IVRDriverInput_004` (18:25:44.182, logged
`NOT HOOKED`). In 86 s with two Index controllers: zero `CreateBooleanComponent`,
zero `CreateScalarComponent`, zero trigger lines, and zero "components created
before our hook" counts — so nothing at all used the version we hooked. Chasing
`_004` would mean a newer `openvr_driver.h` than conan's 1.16.8 and three more
detours inside vrserver for data that is available without any hook: **VR events
(`PollNextEvent`)**. Session 5 removed the three `IVRDriverInput` detours and their
entry points and added an `EventPoller` seam on `SpikeObserver`, polled each
`RunFrame` (no detour), wired in the adapter to
`vr::VRServerDriverHost()->PollNextEvent`.

**Session 6 — reversal, input capture leaves the driver DLL:** after a live run,
`PollNextEvent` proved no more reliable than the input hooks — neither path
delivered button events to the driver. The decision is to stop trying: input
capture is removed from the driver DLL **completely**. The `EventPoller` seam,
`SpikeObserver::setEventPoller` / `pollButtonEvents` / the `buttonEvents_`
counters and the button log lines (per-press, rate, summary) are gone;
`ServerEnvironment::eventPoller` and the adapter's `OpenVrEventPoller` are gone;
the `FakeServerDriverHost::PollNextEvent` override in `SpikeDriverTest` is a
no-op stub (the pure virtual still has to be implemented) and the five
`SpikeObserverTest` button cases are deleted. A separate **background client
app** will capture buttons going forward (it links `openvr_api` as a client and
reads controller state the way `src/vr/OpenVrTracking` does today). The driver
DLL's job is poses + metadata only.

**What stays:** the reason driver-side input was wanted (phase A deletes the
app's OpenVR client session, so the app can no longer read buttons itself) is
now served by a *second* client app dedicated to input, leaving the main app
OpenVR-free. `doc/driver-plan.md` "Buttons" / `DeviceButtons` / threading
contract / verified-groundwork vtable index list / risks are updated to the
client-app path. `IVRDriverInput` remains unhooked and `NotNeeded` in the
`classifyInterface` table — that classification is unchanged by the reversal.

### 5.2 Fix the hook-install race (reproduced, two failing tests) — DONE (session 5)

**Evidence it is live, not theoretical:** `hook IVRServerDriverHost::TrackedDeviceAdded:
installed` at 18:25:43.854 (the eager install in `SpikeServer::init`) and `interface
"IVRServerDriverHost_006": hooking` at 18:25:43.958 — two callers on one hook object,
104 ms apart, one of them on an arbitrary thread, with nothing serializing them. Same for
`IVRDriverInput_003` at 43.877. `install()` publishes `target_` only at its very end
(`SpikeHooks.cpp:42`), so the second caller does not see `installed()` and walks into
`MH_CreateHook` on an already-hooked function.

**Reproductions** (`tests/SpikeHooksTest.cpp`, single-threaded — the fake's `create()`
re-enters `install()`, and `*original` is written before the re-entry exactly as
`MH_CreateHook` publishes its trampoline before returning):

- `AnInstallReenteredDuringCreateKeepsAnOriginalToCallThrough` — first create OK, second
  returns `MH_ERROR_ALREADY_CREATED`. Result today: `installed()` true, `original()`
  **null**, because the loser's rollback (`SpikeHooks.cpp:30`) cleared the winner's
  trampoline. Every detour body opens with `hooks.x.original()(...)`, so that state is a
  call through null inside vrserver, at 1.6 kHz.
- `AnInstallReenteredDuringCreateHooksTheFunctionOnlyOnce` — both creates accepted: two
  create/enable pairs on one function, a hook chained onto a hook, and `removeAll()`
  unwinds one of them. 4 recorded calls instead of 2.

**Fix direction:** one serialization point for all installs, dedupe by hook object rather
than by version string, and stop clearing `original_` on a failure path that can run
after another caller has published the hook. The `HookApi` seam means the fix is
unit-testable. Do not add threads to the tests.

### 5.3 Rework `Logger`: formatting and dispatch to a sink, nothing else. The sink is spdlog — DONE (session 5)

`Logger` currently carries a second, unplanned output mechanism next to the `LogSink` the
plan specifies (`doc/driver-plan.md` §"Logging"): a `shared_ptr<ostream>`, a path, an
`isOpen`/`filePath`/`close` surface, its own timestamp source and line formatter, and a
`flush()` on every line — under a mutex.

**Keep:** `LogSink = std::function<void(const char*)>`, `setSink`, `logf`, `log()`.

**Delete:** `setStream`, `stream_`, `path_`, `isOpen()`, `filePath()`, `close()`,
`TimestampFn`/`setTimestampSource`/`timestamp_`, `localTimestamp`, `formatTimestamp`,
`formatLogLine`, `LogEnvironment`, `currentLogEnvironment`, `logPath`, `logDirectory`,
`processBaseName`, `openProcessLog`, and `mutex_` — plus the call sites
(`SpikeServer.cpp:68` logs the file path, `:115` and `:160` close the log,
`SpikeClient.cpp:83,98,113`) and the `SpikeLogTest` cases that exist only for the deleted
code. Tests that use `setStream` (`SpikeHooksTest`, `SpikeDriverHooksTest`,
`SpikeServerTest`) move to `setSink`, which most spike tests already use.

Dropping the mutex is only sound with one rule attached: **the sink is installed once and
never cleared.** Today `close()` clears it during `Cleanup` while pose threads are still
running, which is defect §6.3 in another guise. With the spdlog logger leaked like every
other process-wide object in the DLL, `sink_` is written before any hook is installed
(`SpikeServer::init` wires logging at `:65`, installs hooks at `:85-91`) and only read
afterwards.

spdlog then owns destinations, timestamps, buffering and flushing. `SpikeLib` stays
spdlog-free so the test binary does not link it; the spdlog logger is created in the
adapter side (`SpikeDriver.cpp` / `SpikeClient.cpp`), which needs `spdlog::spdlog` added
to `driver_00trackingcorrector` and `spike_client` in `CMakeLists.txt`.

**Session 7 follow-up.** The rewrite moved the *path* logic into the adapters, where no
test can reach it — a §2.1a violation the session-5 review missed. `logDirectory` and a
`processNameFromPath` successor to `processBaseName` are therefore back in `SpikeLib`, as
`SpikeLogFile.*`: pure path computation over a `ProcessApi` seam (environment block +
running executable), no filesystem, no `Logger` coupling, no spdlog. The other two
adapter decisions moved with them: `compositeSink` (file sink + `IVRDriverLog` fan-out,
skipping an empty sink instead of throwing `bad_function_call` inside a detour) and
`loggingTo` (install the sink only if none is installed, so a later provider `Init` cannot
replace the composite with the plain file sink).

**Open, for the owner:** whether the spdlog logger keeps a sink that copies lines to
`IVRDriverLog` (SteamVR's `vrserver.txt`). If not, `driverLogSink()`,
`ServerEnvironment::routeLogToDriverLog()` and their three tests go as well.

### 5.4 `logPoseSamples` must not compose a pose SteamVR calls invalid (reproduced) — DONE (session 5)

`logPoseSamples` gates on `lastPoseValid` (`SpikeObserver.cpp:284`), which means "a pose
struct was stored", not `pose.poseIsValid` — the field name is itself the bug. So it
composed and printed four transform lines per untracked device: base-station zero
quaternions, the 9001 sentinel, the Tundra Tracker's 9 km position, `timeOffset=-80.9`.
That is most of the tail of the log, in the same format as the `A`/`B` lines the spike
exists to compare.

**Reproduction:** `SpikeObserverTest.APoseSteamVRReportsAsInvalidIsNotComposed`, built
from the actual 18:27:09.364 pose. It asserts only that the four composition lines are
absent, so it holds under either contract below.

**Contract decision for the owner:** for an invalid pose, drop all six lines for that
device, or keep the one-line header (`valid=0 result=1 timeOffset=-80.92440` — how the
standby staleness became visible at all) and drop only the four composition lines? Once
decided, add the matching header assertion to the test.

---

## 6. Postponed to a later iteration: the concurrency defects

The owner's decision at the end of session 4: fix §5 first, then this. The state
inventory and the analysis are still valid and still incomplete.

Threads in play (`doc/spacecalibrator-notes.md` §8): the vrserver main thread (`Init`,
`RunFrame`, `Cleanup`, standby); arbitrary per-driver threads (`TrackedDevicePoseUpdated`
and other drivers' `GetGenericInterface`, measured at ~1.6 k calls/s in §3); and whatever
thread calls `HmdDriverFactory`.

| State | Owner | Accessed from | Lock |
|---|---|---|---|
| `devices_`, `interfaces_`, `poseSizeWarned_`, `properties_` | `SpikeObserver` | hook threads + main | `SpikeObserver::mutex_`, held on every access |
| `runFrameCount_`, `runFrameCountAtStats_`, `startedAt_`, `housekeepingAt_`, `statsAt_` | `SpikeObserver` | main only | none — invariant commented, not enforced |
| the six `VTableHook`s (`target_`, `original_`, `name_`) | `SpikeDriver.cpp` file scope, leaked | installed from main **and** from detours; `original_` read by every detour | **none** |
| `Logger` state | `SpikeLog.cpp`, leaked | all threads | `Logger::mutex_` on every access |

1. **Double-install race — reproduced, fix scheduled as §5.2.**
2. **Unsynchronized publication of `original_` — open.** Detours read it on arbitrary
   threads with no acquire while `install()` writes it. Practically survivable on x86-64,
   formally a data race. Fix: `std::atomic<void*>`, loaded once at detour entry. **Not
   reproducible by any test** — an aligned pointer store is not torn and MSVC has no
   ThreadSanitizer; this one is an argument, per §2.2.
3. **Teardown race in `Cleanup` — open, worst consequence.** `MH_RemoveHook` rewinds
   threads out of the patched target, but a thread already inside our detour body can
   read `original_` after `remove()` nulled it. `Cleanup` runs on the main thread with
   pose threads live. The null-out must go regardless of which fix is chosen.
4. **DLL-unload race — mostly addressed.** Every process-wide object is created with
   `new` in a function-static accessor and never destroyed, so a late detour touches no
   destroyed object. The DLL's *code* being unmapped while hooks point into it remains
   exposed; SpaceCalibrator has the same exposure. Document it, do not pretend it away.
5. **Main-thread-only invariant — partly addressed.** Commented in `SpikeObserver.h`, not
   enforced. A debug thread-id assert is the remaining work.
6. **Blocking under the lock — open, and now measured.** `logPoseSamples` (`:280`),
   `logRates` (`:327`) and `onCleanup` (`:195`) log while holding
   `SpikeObserver::mutex_`, and pose threads take that same lock 1.6 k times a
   second. §5.3 makes each line cheap; it does not fix the lock discipline. The
   structural fix is snapshot under lock, release, then log.

Also still to be argued: lock ordering (`SpikeObserver::mutex_` → `Logger::mutex_` is the
only nesting today, and the driver-log sink calls into vrserver, which must be assumed not
to re-enter us), and the sequencing around `MH_Initialize`/`MH_Uninitialize` while another
driver could be loading.

---

## 7. Documentation that the live run made wrong

- `AGENTS.md`: "**`vrserver.exe` holds the DLL open — close SteamVR before rebuilding**"
  and the log process list "`vrserver`, `vrwatchdog`, `TrackingCorrectorTests`".
- `build-driver.bat` header comment: same claim about vrserver.
- This file's §2.3 now carries the correct rule; the two above still need fixing.

Correct facts: the watchdog provider is loaded by **`steam.exe`** on this SteamVR version
(`driver-spike-Steam.log`, `HmdDriverFactory("IVRWatchdogProvider_001")` at 18:27:15,
5 s after vrserver's `Cleanup`), `watchdog Cleanup` is never called, and the DLL was still
mapped in `steam.exe` with SteamVR closed. Since `CMakeLists.txt:130-131` points the
Release linker straight at the staged path, a Release driver build fails with `LNK1104`
until **Steam itself** exits.

---

## 8. Discoveries worth not re-learning

- `InitServerDriverContext` fetches and **caches** `IVRServerDriverHost`, `IVRSettings`,
  `IVRProperties`, `IVRDriverLog`, `IVRDriverManager`, `IVRResources` *before* our
  `GetGenericInterface` detour can exist. The detour only ever sees later requests. That
  is why those interfaces must also be hooked eagerly from the pointers the context
  already holds — and it is the source of the double-install race in §5.2.
- Hooking works globally because MinHook patches the *function body* the vtable slot
  points at, shared by every instance of vrserver's concrete class. That is why other
  drivers' devices become visible, and why deduping installs by version string is safe for
  coverage but not for correctness.
- Vtable indices, from conan `openvr/1.16.8`: `IVRDriverContext::GetGenericInterface` 0;
  `IVRServerDriverHost_006` `TrackedDeviceAdded` 0, `TrackedDevicePoseUpdated` 1.
- `k_InterfaceVersions` lists the interfaces a *driver implements*, not the host
  interfaces — returning it verbatim is the ABI check SteamVR demands.
- Client-side raw universe enum is `vr::TrackingUniverseRawAndUncalibrated`.
- The Conan toolchain sets `CMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded[Debug]`, i.e. static
  CRT — the DLL depends on `KERNEL32.dll` only, in any config.
- `openvr.h` and `openvr_driver.h` cannot be included in one translation unit but can be
  linked into one binary; hence the openvr-free headers in `SpikeNames.h` /
  `SpikeClientReport.h` and why `spike_client` can link `SpikeLib`.
- MinHook's `MH_OK` is 0, which is what lets `HookApi` speak plain `int`.
- **A hook test must force real virtual dispatch.** Calling a fake directly lets a Release
  build devirtualize and inline it, skipping the patched body: forwarding assertions still
  pass while observation assertions fail, which reads exactly like a broken hook. Hence
  `throughVtable()` in `SpikeDriverTest`, and hence **run `ctest -C Release` as well as
  Debug for anything touching the hooks.**
- Log lines contain non-ASCII em dashes (`SpikeObserver.cpp:50`, `SpikeServer.cpp:75`),
  which reach the log file as raw UTF-8 bytes and read as mojibake in ANSI viewers. MSVC
  is not given `/utf-8`; it works by accident. Keep log strings ASCII.
- Housekeeping deliberately runs on the **first** `RunFrame`, not one second in.
- `%LOCALAPPDATA%\TrackingCorrector\driver-spike-TrackingCorrectorTests.log` is written by
  `SpikeDriverTest` loading the real DLL, and is the only filesystem side effect the test
  suite has (§2.1b).

---

## 9. Build, run, install

- `build-driver.bat` — Release DLL + `spike_client`. **Exit Steam first** (§7).
- `install-driver.bat` / `uninstall-driver.bat` — `vrpathreg adddriver|removedriver` on
  `build/driver/00trackingcorrector`; nothing is copied, SteamVR loads the driver out of
  the build tree. Restart SteamVR after either.
- `run-tests.bat` — Debug `ctest`. Expect the three intentional failures listed at the
  top until §5.2 and §5.4 land.
- `ctest -C Release` — required for anything touching the hooks (§8).
- Live-run procedure, if another one is needed: hold every device still (session 4 did
  not, which is why the composition evidence is a margin argument rather than an
  equality), run `spike_client <seconds>` alongside, then grep the driver log for
  `NOT HOOKED` and `returned NULL`.

---

## 10. Open decisions for the owner

1. **OpenCppCoverage** (§2.1) — still the biggest open item; until it exists the coverage
   bar is claimed, not measured.
2. **The invalid-pose contract** in §5.4: drop all six lines, or keep the header.
3. **`IVRDriverLog` output** in §5.3: does the spdlog logger still copy lines into
   SteamVR's `vrserver.txt`?
4. **`%TEMP%\SpikeInterfaces.bak`** — still there, still needs an OK to delete.
5. Whether `TrackedDeviceAdded` — a hook added beyond the plan's step-1 list —
   stays. It earned its place (it is how device arrival was observed at all).
   (`CreateScalarComponent` was an `IVRDriverInput` detour removed with §5.1.)
