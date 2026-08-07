# OpenVR driver — phase A: move device poses to the driver

Design doc for the driver half of milestone 4 (`doc/plan.md`). Written before
implementation; read this first, then `AGENTS.md` for the existing structure.

## Why a driver at all

Corrections have to be applied at driver level (that is the only place SteamVR's
own view of a tracker's pose can be changed), which means the app can no longer
trust what the OpenVR *client* API reports — it would be reading back our own
corrections. The unmodified poses have to come out of the driver instead.

Phase A is therefore the migration: the driver becomes the app's only source of
device **poses and metadata**, pushed over a named pipe. Button / input capture
is NOT part of the driver DLL — the live run showed neither `IVRDriverInput` hooks
nor `IVRServerDriverHost::PollNextEvent` delivered button events reliably, so a
separate background client app captures input instead (see "Buttons" below).
Pose modification and virtual trackers are NOT in this phase, but the pose hook is
installed now, because it is simultaneously the source of unmodified poses and
the exact place corrections will later be applied. Protocol type codes >= 128
are reserved for the app -> driver direction.

## Coordinate space

Everything stays in raw driver space. Room setup's floor height / yaw /
translation (the standing universe) is chaperone data, available client-side
only, and we do not care about the space origin: nothing in solving or
calibration reads absolute Y (`Skeleton::makeDefault` places landmarks relative
to the skeleton's own ankles, `updateCalibrationFrame` derives the root from the
HMD and the T-pose hand landmark). The only visible effect is that the skeleton
may float above or sink below `Scene`'s y=0 grid and be yawed relative to it.
Accepted; a floor calibration can be added later on its own merits.

Consequence: the app needs no OpenVR client session at all. `src/vr/OpenVrTracking.*`
is deleted and `openvr::openvr` leaves the exe target. `src/driverdll/OpenVrGlue.cpp`
becomes the only place OpenVR headers are included.

## Verified groundwork

- `openvr_driver.h` ships in the existing conan `openvr/1.16.8` package, and the
  driver context helpers are `inline` — **the driver DLL links no `openvr_api.lib`**,
  headers only.
- Vtable indices for MinHook, from that header: `IVRDriverContext::GetGenericInterface`
  = 0; `IVRServerDriverHost_006`: `TrackedDeviceAdded` = 0, `TrackedDevicePoseUpdated`
  = 1 (matches the index OpenVR-SpaceCalibrator hardcodes). `PollNextEvent` (slot 5)
  is NOT used — see "Buttons" below.
- `IVRProperties_001::TrackedDeviceToPropertyContainer(index)` + `ReadPropertyBatch`
  is the documented way for a driver to look up *any* device's properties by
  device id (stated explicitly in the `GetRawTrackedDevicePoses` docs), so
  serial / class / role hint are obtainable driver-side.
- `minhook/1.3.4` is on conancenter.
- Pose composition, per the `DriverPose_t` comments:
  `world = WorldFromDriver o (vecPosition, qRotation) o DriverFromHead`.
  Confirmed empirically in step 1 before anything depends on it.

## Layout

```
src/ipc/        ByteChannel.h        non-blocking byte-stream interface (the mock seam)
                MemoryChannel.h      test double: loopback, simulated full / disconnect
                Framer.h/.cpp        length-prefixed framing + reassembly
                NamedPipeServerChannel.h/.cpp   overlapped Win32 pipe, driver side
                NamedPipeClientChannel.h/.cpp   overlapped Win32 pipe, app side
src/protocol/   Messages.h           POD wire structs, explicit widths, own raw
                                     read/write helpers
                Codec.h/.cpp         encode / decode
src/driver/     DriverPoseMath.h/.cpp  DriverPose_t POD mirror -> world pose (glm)
                DriverServer.h/.cpp    device table, message production, drop policy,
                                       client lifecycle. OpenVR-free.
src/link/       DriverLink.h/.cpp      app side: protocol -> TrackedDevice, device
                                       table, trigger edges, reconnect
src/driverdll/  DriverFactory.cpp             HmdDriverFactory export: the server
                                              provider + an empty IVRWatchdogProvider
                                              stub (SteamVR asks for both)
                ServerTrackedDeviceProvider.h/.cpp
                Hooking.h/.cpp, InterfaceHookInjector.h/.cpp   (MinHook)
                OpenVrGlue.h/.cpp             DriverPose_t -> POD mirror,
                                              IVRProperties reads, IVRDriverLog sink
                00trackingcorrector/driver.vrdrivermanifest
```

Hooking follows `doc/spacecalibrator-notes.md` §1-3 as-is: `HmdDriverFactory`
serving `IServerTrackedDeviceProvider_004` + `IVRWatchdogProvider_001` from
function-static singletons, `GetInterfaceVersions()` returning
`vr::k_InterfaceVersions` (ABI validation table — SteamVR rejects the driver
without it), `VR_INIT_SERVER_DRIVER_CONTEXT` first in `Init`, then a
`GetGenericInterface` detour (vtable 0) on the raw context pointer that installs
the per-interface hooks lazily as recognized version strings pass by, and
symmetric teardown in `Cleanup` (stop server -> unhook -> clean up context).

The driver additionally needs `IVRProperties` (metadata), which SpaceCalibrator
never touches; it is acquired through the same detoured `GetGenericInterface`.
Buttons / input are NOT captured by this driver: the live run showed zero calls
on the hooked `IVRDriverInput_003` (every controller asked for `_004`), and
`PollNextEvent` proved equally unreliable, so input capture was removed from the
driver DLL entirely. A separate background client app captures buttons instead —
see "Buttons" below and `doc/driver-spike-handover.md` §5.1.

| Target | Kind | Deps |
|---|---|---|
| `TrackingCorrectorDriverLib` | STATIC | glm only — no model, no GL, no spdlog, no openvr |
| `TrackingCorrectorLink` | STATIC | Model + DriverLib |
| `driver_00trackingcorrector` | SHARED | DriverLib + minhook (+ openvr headers) |
| `TrackingCorrector` | WIN32 exe | existing + Link, minus openvr |
| `TrackingCorrectorTests` | exe | existing + DriverLib + Link |

`src/ipc` + `src/protocol` + `src/driver` are all one static lib
(`TrackingCorrectorDriverLib`) — separate folders, one target.

The model types are deliberately NOT shared with the driver. What crosses the
pipe is the wire protocol, which has its own POD structs (explicit widths,
version-stable, decoupled from how `TrackedDevice` evolves). `Pose` /
`TrackedDevice` appear only on the app side of the pipe, where `DriverLink`
reconstructs them. This is why the driver DLL never links the model lib and no
existing CMake target has to be restructured — the driver work is additive.

Logging: `DriverLib` takes an injected `LogSink = std::function<void(const char*)>`
(default no-op) so it stays spdlog-free. The DLL wires it to `vr::VRDriverLog()->Log`
(lands in SteamVR's `vrserver.txt`), the exe to spdlog.

## Wire protocol

The stream opens with `Hello { magic "TCIP", u16 version }` from the driver; the
app refuses a version mismatch and surfaces it in the UI — users forget to
reinstall the driver after an update, and that failure must be loud.

Framing: `u32 payloadSize | u8 type | payload`, little-endian, same style as
`.tcrec`.

Driver -> app:

1. `DeviceList` — full **snapshot**, not a delta: `u16 count`, then per device
   `i32 id, u8 kind, u8 roleHint, u16 serialLen, bytes`. Sent on client connect
   and whenever a previously unknown device id resolves. Only HMDs, controllers
   and generic trackers are listed: the kind filter that
   `OpenVrTracking::pollPoses` applies client-side today moves driver-side, so
   base stations / redirected / display-only devices never reach the wire.
   Consequence: a device is on the wire only once its `Prop_DeviceClass_Int32`
   has been read, so `DevicePose` for an id the app has not seen in a
   `DeviceList` cannot occur.
2. `DevicePose` — `i32 id, u8 flags (poseIsValid | deviceIsConnected),
   f32[3] position, f32[4] rotation (xyzw), f32 poseTimeOffset`. One per hooked
   pose update.

Poses are **state, never events**, so dropping a message when the pipe is busy is
pipe is busy is harmless. Serial and role hint stay on the wire and in the app's
UI table only: `TrackedDevice` and the `.tcrec` format are untouched, so
`ModeController`, `TrackerCalibration`, `SessionRecorder`, `ReplaySession` and
all existing recordings and tests keep working unchanged. Stable identity across
SteamVR restarts (serial in the model, recording v2) is a separate future
decision.

## Driver-side threading contract

Messages are produced per update, on the hook threads, and written to the pipe
there — no IPC thread, no ring buffer. The discipline that makes that safe:
**a hook thread never blocks, and no exception ever leaves a hook or a provider
entry point** (an exception escaping into `vrserver.exe` kills SteamVR, so the
boundary catches everything — the one sanctioned `catch (...)` in this codebase).

- Pose hooks (arbitrary vrserver threads) call `DriverServer::onPose`: update the
  device table, serialize the message, write it. The pipe and the device table
  are guarded by one lock; the write is asynchronous, and a message that cannot
  be handed over because the previous write is still in flight is dropped
  **whole** — never partial bytes, framing must stay intact. Poses are state,
  so the next update supersedes a dropped one. `DeviceList` is never dropped:
  it is retried from `RunFrame`.
- One client at a time, newest connection wins (restarting the app reconnects
  cleanly). A write that does not complete within ~1 s drops the client. On
  connect: `Hello` + full `DeviceList`.
- `RunFrame` does the work that must not happen on a pose thread: accept a
  pending connection; for ids flagged "metadata unknown", read
  `Prop_DeviceClass_Int32`, `Prop_SerialNumber_String`,
  `Prop_ControllerRoleHint_Int32` and send a fresh `DeviceList`.

Buttons: NOT captured by this driver. The live run showed zero calls on the
hooked `IVRDriverInput_003` (every controller asked for `_004`), and
`PollNextEvent` (slot 5 of the already-hooked host) proved equally unreliable,
so input capture was removed from the driver DLL entirely
(`doc/driver-spike-handover.md` §5.1). A separate **background client app**
captures buttons going forward: it links `openvr_api` as a client
(`VRApplication_Background` or `_Overlay`) and reads controller state the way
`src/vr/OpenVrTracking` does today, then forwards the "second trigger goes down
while the first is held" edge to the main app. The IPC for that hop is TBD
(named pipe, same style as the driver pipe); the wire protocol from the driver
carries no `DeviceButtons` message. This keeps phase A's goal — the *main* app
is OpenVR-free (poses from the driver, buttons from the input client) — at the
cost of a second helper process that holds an OpenVR session for input only.

## App side

`DriverLink` mirrors `OpenVrTracking`'s surface so `main.cpp` barely changes:

- `poll()` drains **all** complete messages in order and returns
  `Event { connected, disconnected, versionMismatch, error }` for main.cpp to
  log. (Button state no longer arrives here — see "Buttons" above; it comes from
  the separate input client app.)
- `devices()` returns `std::vector<TrackedDevice>`, filtered to valid +
  connected, ascending id — exactly the shape `OpenVrTracking::pollPoses()`
  produces today, so nothing downstream notices the migration. No kind
  filtering here: the driver already sends only HMDs / controllers / trackers.
- `bothTriggersJustPressed()` — same contract as today (stateful, call once per
  frame). With input capture out of the driver DLL this edge is fed from the
  separate input client app, not from `DriverLink`; the `ModeController`-side
  contract (a per-frame bool) is unchanged.
- Reconnect: non-blocking `CreateFile` attempt at most every ~500 ms while
  disconnected (fails instantly when the driver is not loaded). No driver ->
  the app starts in `ManualPose`, exactly like today's "SteamVR unavailable"
  path, with the retry button relabelled.

## Build and install

- `conanfile.py`: add `minhook/1.3.4`. `openvr` stays (headers, for the DLL).
- DLL staged to `build/driver/00trackingcorrector/bin/win64/driver_00trackingcorrector.dll`
  next to `driver.vrdrivermanifest`:
  `{"alwaysActivate": true, "name": "00trackingcorrector", "directory": "", "resourceOnly": false}`.
  The DLL filename is `driver_<manifest name>.dll` — SteamVR looks it up from the
  manifest name, so directory name, manifest `name` and DLL name must all agree
  (SpaceCalibrator: `01spacecalibrator` / `driver_01spacecalibrator.dll`).
  `alwaysActivate` because we add no devices; the `00` prefix makes SteamVR load
  us before other drivers (OpenVR-SpaceCalibrator uses `01spacecalibrator` for
  the same reason).
- `install-driver.bat` / `uninstall-driver.bat` wrapping SteamVR's
  `bin\win64\vrpathreg.exe adddriver|removedriver`, locating SteamVR via
  `%LOCALAPPDATA%\openvr\openvrpaths.vrpath`.
- Gotcha to document in AGENTS.md: `vrserver.exe` holds the DLL open, so SteamVR
  must be closed before rebuilding the driver.

## Steps

1. **Spike DLL (throwaway).** Loads, logs via `IVRDriverLog`, hooks
   `GetGenericInterface`, `TrackedDeviceAdded`, `TrackedDevicePoseUpdated`.
   Input capture is NOT part of the spike (or the real driver): the live run
   showed neither `IVRDriverInput` hooks nor `PollNextEvent` delivered buttons,
   so a separate background client app captures input — see "Buttons" above and
   `doc/driver-spike-handover.md` §5.1. Logs: device ids / classes / serials,
   pose update rate per device, `RunFrame` cadence, every interface version
   string seen in the `GetGenericInterface` detour that we do **not** hook (an
   unhooked version is otherwise a silent no-op, so it must be loud), and our
   composed world pose alongside the client-side `TrackingUniverseRaw` pose for
   the same device (current exe running simultaneously) to **prove the
   `DriverPose_t` composition**. Closes every remaining unknown before real
   code is designed around it.
2. **`src/ipc`**: `ByteChannel`, `MemoryChannel`, `Framer` + `FramerTest` —
   round-trip, messages split across arbitrary read boundaries, several messages
   in one read, garbage / oversized length rejected.
3. **`src/protocol` + `DriverPoseMath`** + `ProtocolTest` (per-type round-trip,
   unknown type skipped for forward compatibility, truncated payload rejected)
   and `DriverPoseMathTest` (identity cases plus the analytic case captured from
   the step-1 log).
4. **`DriverServer`** + `DriverServerTest` against `MemoryChannel` and a fake
   device source: new client gets `Hello` + full `DeviceList`; an unknown id
   flags a metadata refresh; pose state produces the expected bytes; a
   busy channel drops whole poses but never metadata; disconnect / reconnect
   resets cleanly.
5. **`DriverLink`** + `DriverLinkTest`: bytes -> `devices()` (kind mapping,
   valid / connected filtering, ordering); version mismatch; disconnect empties
   devices; reconnect. (Button / trigger edge is no longer part of `DriverLink` —
   it comes from the separate input client app; a dedicated `InputClient` +
   `InputClientTest` covers the trigger edge occurring entirely between two
   samples.)
6. **Real pipe channels** (`NamedPipeServerChannel` / `NamedPipeClientChannel`,
   overlapped, `\\.\pipe\TrackingCorrectorDriver`) — the only part not covered
   by unit tests, kept as thin as possible.
7. **DLL wiring**: `DriverFactory` (server provider + watchdog stub,
   `GetInterfaceVersions` -> `vr::k_InterfaceVersions`),
   `ServerTrackedDeviceProvider` (`Init` -> context + hooks + server,
   `RunFrame` -> housekeeping, `Cleanup` -> stop + unhook) and
   `OpenVrGlue` -> `DriverServer`. End-to-end check that the app shows the same
   devices / poses as before and that calibration, capture and recording still
   work (trigger gesture fed from the input client app).
8. **`main.cpp` migration**: `OpenVrTracking` -> `DriverLink` (poses) +
   `InputClient` (buttons), delete `src/vr/`, drop openvr from the exe, UI
   status "Driver: connected / not connected / version mismatch". Update
   `AGENTS.md` (new targets and layer rules, driver install and rebuild
   gotchas) and `doc/plan.md` milestone 4.

## Risks

- **Composition formula** — the one thing that fails silently and subtly. Step 1
  exists for it.
- **Hook coverage vs. load order** — vtable patching affects all instances of
  the interface's concrete class, so other drivers' devices should be visible;
  the `00` prefix plus hooking `GetGenericInterface` (the proven
  SpaceCalibrator path) is the belt and braces. Confirmed in step 1.
- **`RunFrame` cadence is unmeasured** — only metadata resolution depends on it,
  so even a slow cadence just means a device shows up a few frames late.
  Measured in step 1.
- **Mixed-driver setups** (e.g. Oculus HMD + Vive trackers) put devices in
  different raw spaces — the problem OpenVR-SpaceCalibrator exists to solve.
  Out of scope; worth a log warning when device drivers differ.
- **Input capture stays on the legacy client path** — the separate background
  input client app reads controller state with `GetDeviceToAbsoluteTrackingPose`
  / the controller-state API `src/vr/OpenVrTracking` uses today, so the
  calibration gesture still depends on SteamVR's legacy binding. That is
  accepted: the live run showed neither `IVRDriverInput` hooks nor
  `PollNextEvent` delivered buttons driver-side, so moving input into the
  driver would have bought reliability it could not deliver. The bonus of
  dropping the deprecated path is deferred until a reliable driver-side input
  source exists. See `doc/driver-spike-handover.md` §5.1.
