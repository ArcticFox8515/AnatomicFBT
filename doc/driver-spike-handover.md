# Handover: SteamVR driver spike (step 1 of `doc/driver-plan.md`)

Status: the spike DLL compiles, loads, and **now meets the line-coverage bar of §2.1**
(session 2 restructured `src/spike` for it — see §6). The **concurrency proof of §2.2
is still incomplete and its known defects are still unfixed** (§7), and **no live
SteamVR run has happened** (§8). Read `doc/driver-plan.md` first, then this file, then
`AGENTS.md`.

---

## 1. Why this file exists

The spike DLL runs **inside `vrserver.exe`**, which SteamVR may run elevated. A crash,
a hang, or a data race in our code takes down the user's whole VR session, and a
`throw` escaping into vrserver kills it outright. Code review alone is not an
acceptable gate for that. The rules in §2 were set by the repo owner during the first
session and are non-negotiable for everything under `src/spike/` and, later,
`src/driver*/`.

---

## 2. Hard rules for this work

### 2.1 Line coverage bar

> Every line of code must be executed at least once, with at least one set of
> parameters, during the test run. The only exemption is pass-through/pimpl-style
> forwarding (`void f(int a) { impl->f(a); }`).

Not "every case" — but every **line**, including error branches, early returns, log
branches, and the standby/watchdog entry points. If a line cannot be reached from a
test, that is a design smell: restructure until it can (see §6).

This must be **measured**, not argued. Proposed tool: **OpenCppCoverage** (free, works
with MSVC PDBs, HTML + Cobertura output), driven by a `run-coverage.bat` next to
`run-tests.bat`. It is not a Conan package — installing it is a decision for the repo
owner. Until it is in place, coverage claims are estimates and must be labelled as
such. **The claim in §6 is exactly such an unmeasured claim: it is line-by-line
reading of the code plus mutation spot-checks, not a coverage report.**

#### 2.1b The adapter exemption is narrower than it was being read (session 3)

`SpikeDriver.cpp` is compiled into `driver_00trackingcorrector` **and into nothing
else**. No test target compiles it, so:

- no unit test can execute a single line of it, and
- an OpenCppCoverage run over `TrackingCorrectorTests` will report it as 0%, because
  `SpikeDriverTest` reaches it only through `LoadLibrary` — a side effect, i.e. an
  integration test, which under §2.1a is not evidence about decisions at all.

Session 2 leaned on "most of `SpikeDriver.cpp` is executed by `SpikeDriverTest`". That
is true and it is not the bar. The rule the owner set in session 3, and the one that now
holds:

> Every function in `SpikeDriver.cpp` is **one instruction**. Two at most, and only when
> the first creates the implementation object and the second calls a method on it. No
> branch, no loop, no comparison, no arithmetic — if you are writing one, it belongs in
> `SpikeLib` with a test.

What that forced out of the DLL, all of it previously unreachable (§6a):
the MinHook status classification, the vtable index table, the install/removal order,
every detour body, the `IVRProperties` / `IVRDriverLog` availability checks, the property
read error check, the module-path buffer handling, and `HmdDriverFactory`'s dispatch.

### 2.1a Unit tests must not depend on the machine

Added in session 2, after a first attempt at §6 wrote tests that opened real files
under `%TEMP%`: a spike test may not touch the **filesystem, the wall clock, the
environment, or any real thread**. The result of a test that does is a property of the
machine it ran on, and proves nothing about code that runs inside `vrserver.exe`.
Concretely: streams are `std::ostringstream`, time is an injected `double`, MinHook is
a fake `HookApi`, `LOCALAPPDATA` / the module path arrive as a `LogEnvironment` value.
The one exception is `SpikeDriverTest`, which loads the real DLL — the DLL opens its
own log file, and that is the price of testing the real binary.

### 2.2 Concurrency: proof, not tests

Races in the hook threads cannot be caught by unit tests. For every piece of shared
mutable state, the handover/PR must contain a written enumeration of **every** access
path (which thread, which entry point, holding which lock) and an argument for why no
interleaving is harmful. §7 starts that document — it is incomplete and it already
found real defects.

### 2.3 Process rules (violations happened; do not repeat)

- Answer the question that was asked **before** changing code. Refactors and
  experiments need agreement first.
- Edit source files **only** with the edit/write tools. Never with shell
  `-replace` / `sed`-style munging: a regex escape wrote `\&\&` into
  `SpikeInterfaces.h` and broke the build in session 1. **It happened again in session
  2** — a PowerShell `Set-Content` round-trip re-encoded one em dash in `AGENTS.md` into
  `U+FFFD`; caught by scanning the file for `U+FFFD` and repaired with the edit tool.
  The rule is not advice.
- Never write outside the workspace. The only sanctioned external path is
  `C:\Users\ARCTIC~1\AppData\Local\Temp\opencode`. A stray backup was left at
  `%TEMP%\SpikeInterfaces.bak` — **delete it** (pending, needs owner's OK).
- Do not say "verified" for something that was only compiled or only loaded. State
  exactly what was executed.

---

## 3. What exists right now

### Targets (`CMakeLists.txt`)

| Target | Kind | Notes |
|---|---|---|
| `SpikeLib` | STATIC | **All** spike logic. openvr **include dirs only**, no MinHook, no openvr_api. Linked by the three targets below. |
| `driver_00trackingcorrector` | SHARED | Spike driver DLL = `SpikeDriver.cpp` (adapter) + `SpikeLib` + `minhook::minhook`. |
| `spike_client` | console exe | `SpikeClient.cpp` (adapter) + `SpikeLib` + `openvr::openvr`. No glm any more (the matrix→pose conversion moved into `SpikePoseMath.h`). |
| `TrackingCorrectorTests` | exe | Links `SpikeLib`; builds the seven spike unit-test files plus `SpikeDriverTest.cpp`, which gets `SPIKE_DRIVER_DLL="$<TARGET_FILE:driver_00trackingcorrector>"` and `add_dependencies` on the DLL. It does **not** compile `SpikeDriver.cpp` — see §2.1b. |

Staging: **only the Release DLL** goes to
`build/driver/00trackingcorrector/bin/win64/` (the path SteamVR loads) plus the
manifest copied to `build/driver/00trackingcorrector/`. Other configs keep the default
output dir, so a Debug build cannot silently replace the DLL SteamVR is using, and the
tests load the DLL of their own config.

### Files

```
SpikeLib (every line of these runs in a unit test):
src/spike/SpikeObserver.h/.cpp   All observation and bookkeeping: interface dispatch,
                                 device table, component table, trigger edges, pose
                                 sampling, 1 s housekeeping / 5 s statistics, summary.
                                 Seams: Logger&, InterfaceHooks&, DeviceProperties*,
                                 NowFn (monotonic seconds).
src/spike/SpikeServer.h/.cpp     Provider lifecycle behind ServerEnvironment /
                                 WatchdogEnvironment; classifyFactoryRequest();
                                 serveFactoryRequest() (HmdDriverFactory's body);
                                 componentWasCreated().
src/spike/SpikeHooks.h/.cpp      VTableHookBase install/remove state machine over the
                                 HookApi seam (MinHook-free) + typed VTableHook<F> +
                                 initializeHookLibrary() (the MH_Initialize status
                                 classification, incl. ALREADY_INITIALIZED).
src/spike/SpikeDriverHooks.h/.cpp
                                 The driver's six hooks: the function-pointer types, the
                                 vtable index table + log labels as named constants,
                                 DriverHookSet (installs, reverse-order removeAll,
                                 implements InterfaceHooks) and the six observe*
                                 functions that ARE the detour bodies — forward
                                 unchanged, then notify the observer under runGuarded.
                                 Moved out of SpikeDriver.cpp in session 3 (§2.1b).
src/spike/SpikeDriverEnvironment.h/.cpp
                                 The vrserver/Win32 glue that has decisions in it:
                                 OpenVrProperties (DeviceProperties over a
                                 vr::VRProperties() function-pointer seam, incl. the
                                 "no IVRProperties" and "property read failed"
                                 branches), driverLogSink() (the IVRDriverLog copy,
                                 incl. "no driver log"), ModuleApi + modulePathOfAddress()
                                 (GetModuleHandleEx/GetModuleFileName composition and
                                 buffer handling). Also session 3.
src/spike/SpikeLog.h/.cpp        Pure path/timestamp/line formatting + Logger (stream,
                                 sink and timestamp source injected). The four
                                 adapter functions at the bottom (log(),
                                 currentLogEnvironment(), localTimestamp(),
                                 openProcessLog()) are branch-free forwarders.
src/spike/SpikeNames.h/.cpp      Enum labels + isTriggerClick(); header is openvr-free
                                 so the client can use it too.
src/spike/SpikeInterfaces.h      classifyInterface() + interfaceFamily().
src/spike/SpikePoseMath.h        openvr-free double-precision rigid-pose math +
                                 poseFromRowMajor34() (HmdMatrix34 -> pose).
src/spike/SpikeGuard.h           runGuarded(): the one place an exception is swallowed.
src/spike/SpikeClientReport.h/.cpp
                                 Client sampling loop behind ClientPoseSource + the
                                 line formatting.

Adapters (compiled into no test target, therefore one instruction per function — §2.1b):
src/spike/SpikeDriver.cpp        MinHookApi, Win32ModuleApi, nowSeconds, the six detour
                                 entry points, OpenVrServerEnvironment,
                                 OpenVrWatchdogEnvironment, the two providers, the
                                 leaked-singleton accessors, HmdDriverFactory.
src/spike/SpikeClient.cpp        OpenVrClientSource + main().

src/driverdll/00trackingcorrector/driver.vrdrivermanifest
tests/SpikeObserverTest.cpp      29 tests: the whole observation surface, plus the enum
                                 labels, the component-name predicate and runGuarded.
tests/SpikeServerTest.cpp        18 tests: provider lifecycle, watchdog, standby policy,
                                 factory classification, serveFactoryRequest and
                                 component-creation decisions.
tests/SpikeHooksTest.cpp         10 tests: the hook state machine against a fake MinHook
                                 + the hook-library status classification.
tests/SpikeDriverHooksTest.cpp   15 tests: the vtable index table, which detour goes in
                                 which slot, reverse-order removal, and every detour
                                 body (unchanged forwarding, observe-before vs
                                 observe-after, failed component creation).
tests/SpikeDriverEnvironmentTest.cpp
                                 16 tests: properties over a fake IVRProperties, the
                                 driver-log sink, the module-path composition.
tests/SpikeLogTest.cpp           12 tests: path/format purity + Logger behaviour.
tests/SpikeClientReportTest.cpp  7 tests: matrix->pose branches + the sampling loop.
tests/SpikeDriverTest.cpp        12 tests: the fake-vrserver harness on the real DLL (2)
                                 + interface classification (6) + composition math (4).
                                 INTEGRATION, not unit: it LoadLibrary's the DLL.
build-driver.bat                 Release build of DLL + client (close SteamVR first).
install-driver.bat               vrpathreg adddriver on the staged dir.
uninstall-driver.bat             vrpathreg removedriver.
```

`conanfile.py` gained `minhook/1.3.4`. `AGENTS.md` documents all of the above.

### Logs

- Driver: `%LOCALAPPDATA%\TrackingCorrector\driver-spike-<process>.log`
  (`vrserver`, `vrwatchdog`, `TrackingCorrectorTests`) **and** `IVRDriverLog` →
  SteamVR's `vrserver.txt`.
- Client: `%LOCALAPPDATA%\TrackingCorrector\client-spike-spike_client.log` + stdout.

### Build/test state (end of session 3)

- Debug: `cmake --build build --config Debug` clean; `ctest -C Debug` **268/268 passed**
  (160 before the session-2 restructure, 227 after it; 120 of the 268 are `Spike*`).
- Release: clean; `ctest -C Release` **268/268 passed**. Running the suite in Release
  matters and is part of the routine — it caught a bug the Debug run could not (see the
  devirtualization note in §10).
- The 41 tests added in session 3 all cover code that moved out of `SpikeDriver.cpp`
  (§2.1b): 15 in `SpikeDriverHooksTest`, 16 in `SpikeDriverEnvironmentTest`, 3 in
  `SpikeHooksTest` (hook-library status), 6 in `SpikeServerTest` (`serveFactoryRequest`,
  standby policy). Nothing about the DLL's behaviour changed, and `SpikeDriverTest` —
  which drives the real DLL — still passes unmodified, which is the evidence that the
  extraction was behaviour-preserving.
- **Not re-checked after the session-3 refactor:** `dumpbin /dependents` and `/exports`
  on the staged Release DLL, and whether the staged DLL matches the current source. The
  Release DLL *was* rebuilt (it is what `ctest -C Release` loaded), but the dependency
  and export lists were last inspected in session 2, before the extraction.

State as of end of session 2, still valid unless contradicted above:

- `dumpbin /dependents` on the staged Release DLL showed **only `KERNEL32.dll`**,
  `/exports` exactly `HmdDriverFactory` — the `<filesystem>` / `<fstream>` /
  `std::function` additions did not pull a CRT DLL in (static CRT).
- Non-vacuity spot-check: four deliberate mutations (drop the "first stream wins" guard
  in `Logger::setStream`; drop the `original_ = nullptr` rollback after an
  `MH_EnableHook` failure; double the statistics interval in `onRunFrame`; flip a sign
  in one branch of `poseFromRowMajor34`). Three were caught immediately; the fourth was
  **not**, because the 180° test rotations produce symmetric matrices in which that
  sign cancels — the test was extended with asymmetric 160° rotations per branch, after
  which the mutation is caught. All four mutations were reverted and the suite is green.
  **The session-3 tests have had no such mutation check yet.**

---

## 4. What is actually established, and what is not

Executed and observed (Debug/Release builds on this machine):

- DLL compiles and links with no `openvr_api`; `dumpbin /dependents` shows **only
  `KERNEL32.dll`** (Conan toolchain links the CRT statically, so no CRT DLL is needed
  inside vrserver). `dumpbin /exports` shows exactly `HmdDriverFactory`.
- `LoadLibrary` + `HmdDriverFactory` works; both providers are returned; an unknown
  interface name yields `nullptr` + `VRInitError_Init_InterfaceNotFound` (105).
- Through the fake-vrserver harness (in-process, no SteamVR): MinHook patches the
  vtable slots the plan hardcodes (`GetGenericInterface` 0, `TrackedDeviceAdded` 0,
  `TrackedDevicePoseUpdated` 1, `Create/UpdateBooleanComponent` 0/1,
  `CreateScalarComponent` 2); detour signatures are ABI-correct; hooked calls are
  forwarded unchanged; `IVRProperties` metadata reads work; a boolean component
  resolves to a device id; trigger rising/falling edges are logged; `Cleanup` removes
  the hooks. Mutation-checked once: moving the pose hook to vtable index 2 makes 4
  assertions fail, so the harness is not vacuous.
- `vrpathreg.exe` located via `%LOCALAPPDATA%\openvr\openvrpaths.vrpath` →
  `C:\Program Files (x86)\Steam\steamapps\common\SteamVR`.
- `spike_client` without SteamVR: `VR_Init failed: Not starting vrserver for background
  app (121)` — the intended behaviour.

**Not established (needs the live run):** which composition formula (`A` = WorldFromDriver∘local,
`B` = A∘DriverFromHead) matches the client raw pose; real device classes/serials; real
pose rate and `RunFrame` cadence; real trigger component paths and whether any
controller is scalar-only; whether other drivers' interfaces route through the same
vrserver implementation (the load-order/`00`-prefix assumption); behaviour under device
connect/disconnect.

---

## 5. Test inventory (79 spike tests)

**Unit tests against fakes (SpikeLib, no DLL / no MinHook / no vrserver / no
filesystem / no clock):**

- `SpikeObserverTest` (24) — interface dispatch (hook host, hook input, unsupported
  version ×2, not needed, vrserver returned NULL, null version string, dedupe across
  repeated requests); device-added logging; first pose per device; poses for
  out-of-range indices dropped; truncated `unPoseStructSize` warned once and never
  read; boolean/scalar component creation incl. null names; trigger edges (down,
  repeat, up, unknown handle counted, unresolved device logged as `?`); housekeeping on
  the first frame then once per second; no property reader; containers without a
  device; metadata re-read only when the serial changes; component→device resolution
  and the never-resolved case; both candidate compositions against `spike::compose`
  computed in the test; the 5 s rate report as a delta; the cleanup summary incl. the
  skip for untouched device slots and a metadata-only device; a throwing property
  reader contained by `runGuarded`.
- `SpikeNames` (4) + `SpikeGuard` (1) — every enum label arm, the trigger-name suffix
  rule, and that neither a `std::exception` nor a non-exception throw escapes a hook.
- `SpikeServerTest` (7) — Init's hook order; a failing driver context passed back
  unchanged with nothing hooked; MinHook failing → `VRInitError_Driver_Failed`; a throw
  during Init failing the load instead of reaching vrserver; Cleanup's ordering
  (unhook → MinHook down → context released → log closed) and that it still releases
  the context when observation throws; RunFrame/standby observed and unable to throw.
- `SpikeWatchdogTest` (2) + `SpikeFactory` (3) — the watchdog's success and
  context-failure paths, and the factory's three decisions incl. `nullptr`.
- `SpikeHooksTest` (7) — install lands on the requested vtable slot with the requested
  detour; double install is a no-op; null object refused; `MH_CreateHook` failure;
  `MH_EnableHook` failure rolls back and clears `original`; remove is idempotent;
  reinstall after remove.
- `SpikeLogTest` (12) — `processBaseName` / `logDirectory` / `logPath` /
  `formatTimestamp` / `formatLogLine` edge cases; line format; sink content; first
  stream wins; close silences both outputs; unusable and null streams refused; writing
  with no stream and no sink; the default timestamp source's shape.
- `SpikeClientReportTest` (7) — `poseFromRowMajor34` round-trips for every
  largest-component branch, including asymmetric rotations (see the mutation note in
  §3); the client line format matching the driver's columns; the duration predicate;
  the sampling loop with and without devices.

**Integration proof on the real DLL (`SpikeDriverTest`, 2 + 10 pure tests):**

`SpikeDriverLifecycle` loads the DLL and drives a fake vrserver:
- factory serves both providers, rejects unknown interfaces, tolerates a `nullptr`
  interface name and a `nullptr` return code;
- `GetInterfaceVersions()[0] == IVRSettings_Version`;
- `Init` → all six context interfaces requested, `IVRDriverInput` fetched through the
  detour, all six hooks report "installed" on the plan's vtable indices;
- `TrackedDeviceAdded` / `TrackedDevicePoseUpdated` observed + forwarded unchanged;
- component creation, trigger-name recognition, the detour's five branches, dedupe;
- `RunFrame` → metadata line, component→device resolution, pose sample lines whose
  `A`/`B` strings equal `spike::compose` computed independently in the test;
- trigger DOWN/repeat/up, unknown handle tolerated; standby entry points;
- `Cleanup` → "removed" lines, summary lines, and no further observation.

Mutation-checked once (session 1): moving the pose hook to vtable index 2 makes 4
assertions fail, so the harness is not vacuous.

`SpikeInterfaceClassification` (6) and `SpikePoseMath` (4): pure functions, direct
assertions, analytic expected values.

---

## 6. Coverage: how the restructure closed the backlog (session 2)

Session 1's list of never-executed lines (watchdog Init/Cleanup, the standby callbacks,
every failure path in `Init`, the `catch (...)`s, `onPose`'s early return and truncated
branch, the component-creation error paths, `refreshDeviceMetadata`'s three skips, the
whole 5 s `logRates` path, most `deviceClassName`/`roleHintName` arms, `onCleanup`'s
skip, `SpikeLog`'s failure branches, `VTableHook`'s failure branches, and
`SpikeClient.cpp` in its entirety) is **closed**, by doing what §6 of the previous
handover proposed:

1. **`SpikeLib`** (static) holds every spike source except the two adapters, so the
   logic is constructible in a test instead of only reachable through `LoadLibrary`.
   openvr_driver.h is used for its *types* only.
2. **Seams instead of globals.** `SpikeObserver` takes `Logger&`, `InterfaceHooks&`,
   `DeviceProperties*` and a `NowFn` (monotonic seconds — the 1 s / 5 s branches are
   tested by advancing a `double`, no sleeping). `SpikeServer` / `SpikeWatchdog` take a
   `ServerEnvironment` / `WatchdogEnvironment` covering `InitServerDriverContext`,
   `MH_Initialize`, the module path, the pid, the hook installs and the teardown.
   `VTableHookBase` talks to a `HookApi` instead of MinHook. `Logger` is handed a
   stream, a sink and a timestamp source.
3. **Decisions extracted from the adapters** so nothing that branches lives in an
   untested file: `classifyFactoryRequest` (incl. `nullptr`), `componentWasCreated`
   (the `result != None` / `handle == nullptr` guard the detours use),
   `clientShouldContinue`, `poseFromRowMajor34`, `logPath` / `logDirectory` /
   `processBaseName` / `formatTimestamp`, and `runGuarded` (the single `catch (...)`).
4. **The two adapter files are the whole exemption.** `SpikeDriver.cpp` and
   `SpikeClient.cpp` contain only forwarders — the six detours (observe through
   `runGuarded`, forward unchanged), `MinHookApi`, `OpenVrDeviceProperties`,
   `OpenVrServerEnvironment`, `OpenVrWatchdogEnvironment`, the two providers,
   `HmdDriverFactory`, `OpenVrClientSource`, `main`. Most of `SpikeDriver.cpp` is still
   executed by `SpikeDriverTest` through the real DLL; `SpikeClient.cpp` is not
   executed at all (it needs SteamVR), which is precisely why nothing but forwarding
   may live in it.
5. **The DLL-boundary lifecycle test is kept** as the integration proof on top.

Not covered, deliberately, and all of it inside the two adapter files: the
`MH_ERROR_ALREADY_INITIALIZED` branch of `initHookLibrary`, the `vr::VRProperties()`
null check in `properties()`, `MinHookApi::statusName` (no MinHook call fails in the
DLL test), and all of `SpikeClient.cpp`.

**This is still an unmeasured claim** (§2.1): it is reading plus the mutation
spot-checks in §3. Installing OpenCppCoverage and adding `run-coverage.bat` remains the
open item, and is the only thing that turns "every line" into a fact.

---

## 7. Concurrency proof — STARTED, INCOMPLETE, DEFECTS FOUND

Threads in play (per `doc/spacecalibrator-notes.md` §8):
1. vrserver main thread — `Init`, `RunFrame`, `Cleanup`, standby callbacks.
2. Arbitrary per-driver threads — `TrackedDevicePoseUpdated`,
   `Create/UpdateBooleanComponent`, `CreateScalarComponent`, and any other driver's
   `GetGenericInterface` calls, all arriving through our detours concurrently.
3. Whatever thread SteamVR calls `HmdDriverFactory` on (once per process).

### Shared state inventory (updated for the session-2 structure)

| State | Owner | Accessed from | Lock |
|---|---|---|---|
| `devices_`, `components_`, `componentByHandle_`, `interfaces_`, `poseSizeWarned_`, `unknownBooleanUpdates_`, `properties_` | `SpikeObserver` | hook threads + main thread | `SpikeObserver::mutex_` — **held on every access as written today** |
| `runFrameCount_`, `runFrameCountAtStats_`, `startedAt_`, `housekeepingAt_`, `statsAt_` | `SpikeObserver` | main thread only (`onInit`, `onRunFrame`, `onCleanup`) | none — relies on a main-thread-only invariant that is now **commented in the header but still not enforced** |
| `HookSet` in `hooks()` (six `VTableHook` objects: `target_`, `original_`, `name_`) | `SpikeDriver.cpp` file scope, leaked | installed from the main thread **and** from detours; `original_` read by every detour on arbitrary threads | **none** |
| `Logger` in `log()` (`stream_`, `sink_`, `timestamp_`, `path_`) | `SpikeLog.cpp`, leaked | all threads | `Logger::mutex_` on **every** member access, `filePath()` included |

### Defect status

1. **Double-install race on `VTableHook` (real, STILL UNFIXED).**
   `SpikeObserver::onInterfaceRequested` serializes installs behind `mutex_` + the
   `interfaces_` dedupe, **but `SpikeServer::init` also calls
   `environment_.hookServerDriverHost()` / `hookDriverInput()` eagerly, bypassing the
   dedupe entirely** (it has to: the context caches those interfaces before our detour
   exists). Another driver's thread can be inside the detour path for the same version
   string at the same moment, so two threads can call `install()` on the same hook
   object concurrently: `installed()` check → `create` → writes to `target_`/`original_`
   are all unsynchronized. Outcome ranges from a duplicate hook to a torn `original_` —
   i.e. a bad call target inside vrserver.
   *Fix direction:* one serialization point for all installs (install only under
   `mutex_`, dedupe by hook object, not by version string), or make `VTableHookBase`
   internally synchronized. The restructure moved this code behind the `HookApi` seam,
   which means a fix can now be unit-tested — but no fix has been made.
2. **Unsynchronized publication of `original_` (real, STILL UNFIXED).** Detours do
   `hooks().x.original()(...)` on arbitrary threads with no atomic/acquire, while
   `install()` writes it. Practically survivable on x86-64, formally a data race.
   *Fix direction:* `std::atomic<void*>` in `VTableHookBase`, loaded once at detour
   entry.
3. **Teardown race in `Cleanup` (real, worst consequence, STILL UNFIXED).**
   `MH_RemoveHook` rewinds threads out of the *patched target*, but a thread already
   executing **inside our detour body** can then read `original_` after `remove()`
   nulled it → call through null. `Cleanup` runs on the main thread while pose threads
   are still live.
   *Fix direction:* never null `original_`; gate detour bodies on an atomic `enabled`
   flag; or accept SpaceCalibrator's "remove all, then never touch" ordering and prove
   it — but the null-out must go either way.
4. **DLL-unload race (mostly addressed).** Every piece of process-wide state is now
   created with `new` in a function-static accessor and **never destroyed** (`log()`,
   `hookApi()`, `hooks()`, `interfaceHooks()`, `observer()`, and both providers), so a
   detour that runs after the DLL's static destructors would have fired no longer
   touches destroyed objects. The remaining exposure is the DLL's *code* being unmapped
   while hooks still point into it — nothing our code can do about that; it is the same
   exposure SpaceCalibrator has, and it must stay documented rather than pretended away.
5. **`logFilePath()` returning a reference without the lock — FIXED.** It is now
   `Logger::filePath()`, returning `std::string` by value under `mutex_`.
6. **Main-thread-only invariant (partly addressed).** The `runFrameCount_` / tick
   fields are grouped under a comment in `SpikeObserver.h` saying they are main-thread
   only. Still not *enforced*: a debug thread-id assert is the remaining work.

Also to be proved, not yet analysed:
- Lock ordering: `SpikeObserver::mutex_` → `Logger::mutex_` is the only nesting today
  (logging under the state lock in `resolveComponents`, `logPoseSamples`, `logRates`,
  `onCleanup`); the log sink calls into vrserver's `IVRDriverLog`, which must be assumed
  not to re-enter us. Confirm and state it as an invariant.
- **Blocking under the lock:** `Logger::write` does stream I/O with a `flush()` on every
  line **while `SpikeObserver::mutex_` is held** in several places, and pose threads take
  that same lock. This violates the plan's "a hook thread never blocks" contract and
  will matter more once the pipe write joins the picture. Move logging outside the lock.
- `MH_Initialize` / `MH_Uninitialize` being called from `Init`/`Cleanup` while another
  driver could be loading (MinHook itself is thread-safe; our sequencing around it is
  what needs the argument).

**Defects 1-3 and 6 are still open.** The concurrency work should land *before* the live
SteamVR run, because the failure mode is "SteamVR dies and we don't know why".

---

## 8. The live run (still pending — this is the point of step 1)

1. Close SteamVR. `build-driver.bat` (Release).
2. `install-driver.bat`, restart SteamVR.
3. Hold every device still. Run `build\Release\spike_client.exe 30`.
4. Compare, at equal timestamps, `driver-spike-vrserver.log` against
   `client-spike-spike_client.log`:
   - whichever of `A = wFd o local` / `B = A o dFh` equals the client's `raw` line
     settles the composition formula for `src/driver/DriverPoseMath` (step 3);
   - the `raw` vs `standing` difference confirms the coordinate-space assumption in
     `doc/driver-plan.md` §"Coordinate space".
5. Then: pull both triggers (expect `trigger DOWN` lines with the right device),
   power a tracker off/on (device table refresh), check `pose rate:` and `RunFrame:`
   lines for cadence, and grep for `NOT HOOKED` / `returned NULL` — either means the
   real driver's assumptions are wrong.
6. Record the findings **in `doc/driver-plan.md`** (§"Verified groundwork" and the
   risks list), because steps 3-7 are designed against them.
7. `uninstall-driver.bat` when done.

Gotcha: `vrserver.exe` holds the DLL open — SteamVR must be closed before every driver
rebuild.

---

## 9. Open decisions for the owner

1. ~~**Scope:** do the coverage restructure on the throwaway spike?~~ **Decided in
   session 2: yes.** `src/spike` was restructured so that every function either runs in
   a unit test or is a pure forwarder (§6). The same shape is what steps 3-7 need
   anyway, so it is not wasted.
2. **Keep or revert `src/spike/SpikeInterfaces.h`** and the `onInterfaceRequested`
   switch — added without being asked in session 1. Still open; it is kept for now and
   is unit-tested, and its `classifyInterface` is the reason the "unsupported version"
   path can be tested at all.
3. **Install OpenCppCoverage** (or another MSVC-compatible coverage tool) so the line
   bar is measured rather than claimed. **Still the biggest open item** — §6's claim is
   reading + mutation spot-checks, not a report.
4. Whether the extra hooks I added beyond the plan's step-1 list
   (`TrackedDeviceAdded`, `CreateScalarComponent`) stay — each is extra risk inside
   vrserver for extra information.
5. ~~Delete `%TEMP%\SpikeInterfaces.bak`.~~ Not touched in session 2 (writing or
   deleting outside the workspace needs the owner's OK); still pending.
6. **Concurrency (§7 defects 1-3, 6):** fix on the spike before the live run, or accept
   the risk for a deliberate, supervised run? The failure mode is SteamVR dying with no
   explanation, so the recommendation is to fix first.

---

## 10. Discoveries worth not re-learning

- `VR_INIT_SERVER_DRIVER_CONTEXT` / `InitServerDriverContext` fetches and **caches**
  `IVRServerDriverHost`, `IVRSettings`, `IVRProperties`, `IVRDriverLog`,
  `IVRDriverManager`, `IVRResources` *before* our `GetGenericInterface` detour can
  exist. So the detour only ever sees later requests — ours (`IVRDriverInput`) and
  other drivers'. That is why the interfaces we care about must also be hooked eagerly
  from the pointers the context already holds. `SpikeDriver.cpp` calls
  `vr::InitServerDriverContext` directly instead of the macro so the failure can be
  logged.
- Hooking works globally because MinHook patches the *function body* the vtable slot
  points to, which is shared by every instance of vrserver's concrete class — that is
  the whole reason other drivers' devices become visible.
- Vtable indices confirmed against the conan `openvr/1.16.8` header (`IVRDriverContext`
  0; `IVRServerDriverHost_006` 0/1; `IVRDriverInput_003` 0/1/2/3).
- `k_InterfaceVersions` lists the interfaces a *driver implements* (Settings,
  ITrackedDeviceServerDriver, …), not the host interfaces — returning it verbatim is
  the ABI check SteamVR demands.
- Client-side raw universe enum is `vr::TrackingUniverseRawAndUncalibrated` (not
  `TrackingUniverseRaw`).
- The Conan toolchain sets `CMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded[Debug]`, i.e.
  **static CRT** — a driver DLL with no CRT DLL dependency in any config.
- `unPoseStructSize` may be smaller than `sizeof(DriverPose_t)` for a caller built
  against an older header; fields must not be read in that case (handled, and now
  tested — `SpikeObserverTest.ATruncatedPoseStructIsWarnedAboutOnceAndNeverRead`).
- Housekeeping deliberately runs on the **first** `RunFrame`, not one second in.
- Tests writing to `%LOCALAPPDATA%\TrackingCorrector\driver-spike-TrackingCorrectorTests.log`
  is a side effect of the DLL-boundary test — and the *only* filesystem side effect the
  test suite has (§2.1a).
- `openvr.h` and `openvr_driver.h` cannot be included in the same translation unit, but
  they can be linked into the same binary: the types they share come from the same
  inlined `vrtypes.h` with the same include guard. That is why `SpikeNames.h` /
  `SpikeClientReport.h` are openvr-free headers with the openvr include confined to
  their `.cpp`, and why `spike_client` can link `SpikeLib`.
- MinHook's `MH_OK` is 0, which is what lets `HookApi` speak plain `int` and keep
  `SpikeLib` MinHook-free.
- **A hook test must force a real virtual dispatch.** MinHook patches the *function* a
  vtable slot points to, so calling a fake directly (`context.host.TrackedDeviceAdded(…)`)
  lets a Release build devirtualize and inline it, skipping the patched body — the
  forwarding assertions still pass (the inlined body has the same side effects) while
  the observation assertions fail, which reads exactly like a broken hook. This is why
  `SpikeDriverTest` routes every such call through `throughVtable()` (a `volatile`
  pointer). It cost one Release-only failure to find; **run `ctest -C Release` as well
  as Debug for anything touching the hooks.**

---

## 11. Suggested order of work in the next session

1. Rebuild Debug + full `ctest` (expect 227) **and Release + `ctest -C Release`**;
   confirm green (§3, and the devirtualization note in §10).
2. Settle the remaining §9 decisions with the owner (coverage tooling, the extra hooks,
   `SpikeInterfaces.h`, the stray `%TEMP%` file).
3. Concurrency: fix §7 defects 1-3 and 6, update the proof in §7 as part of the same
   change. The `HookApi` seam means the install/remove synchronization can now be
   unit-tested rather than argued.
4. Coverage tooling (OpenCppCoverage + `run-coverage.bat`), then re-check §6's claim
   against the report instead of against reading.
5. Only then the live SteamVR run (§8), and record the findings in
   `doc/driver-plan.md`.
