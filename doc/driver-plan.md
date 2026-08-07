# OpenVR driver — phase A: move device input to the driver

Design doc for the driver half of milestone 4 (`doc/plan.md`). Written before
implementation; read this first, then `AGENTS.md` for the existing structure.

## Why a driver at all

Corrections have to be applied at driver level (that is the only place SteamVR's
own view of a tracker's pose can be changed), which means the app can no longer
trust what the OpenVR *client* API reports — it would be reading back our own
corrections. The unmodified poses have to come out of the driver instead.

Phase A is therefore the migration: the driver becomes the app's only source of
device data (metadata, poses, button state), pushed over a named pipe. Pose
modification and virtual trackers are NOT in this phase, but the pose hook is
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
  = 1 (matches the index OpenVR-SpaceCalibrator hardcodes); `IVRDriverInput_003`:
  `CreateBooleanComponent` = 0, `UpdateBooleanComponent` = 1.
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

The driver additionally needs `IVRProperties` (metadata) and `IVRDriverInput`
(buttons), which SpaceCalibrator never touches; both are acquired through the
same detoured `GetGenericInterface`.

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
   has been read, so `DevicePose` / `DeviceButtons` for an id the app has not
   seen in a `DeviceList` cannot occur.
2. `DevicePose` — `i32 id, u8 flags (poseIsValid | deviceIsConnected),
   f32[3] position, f32[4] rotation (xyzw), f32 poseTimeOffset`. One per hooked
   pose update.
3. `DeviceButtons` — `i32 id, u32 mask` (bit 0 = trigger). Sent on change.

Poses and buttons are **state, never events**, so dropping a message when the
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

- Pose / button hooks (arbitrary vrserver threads) call `DriverServer::onPose` /
  `onButton`: update the device table, serialize the message, write it. The pipe
  and the device table are guarded by one lock; the write is asynchronous, and a
  message that cannot be handed over because the previous write is still in
  flight is dropped **whole** — never partial bytes, framing must stay intact.
  Poses and buttons are state, so the next update supersedes a dropped one.
  `DeviceList` is never dropped: it is retried from `RunFrame`.
- One client at a time, newest connection wins (restarting the app reconnects
  cleanly). A write that does not complete within ~1 s drops the client. On
  connect: `Hello` + full `DeviceList`.
- `RunFrame` does the work that must not happen on a pose thread: accept a
  pending connection; for ids flagged "metadata unknown", read
  `Prop_DeviceClass_Int32`, `Prop_SerialNumber_String`,
  `Prop_ControllerRoleHint_Int32` and send a fresh `DeviceList`; resolve pending
  `container -> device id` lookups for boolean components.

Buttons: hook `CreateBooleanComponent` to record `handle -> (container, name)`,
keeping only names ending in `/input/trigger/click`; resolve `container -> device id`
by scanning `TrackedDeviceToPropertyContainer(0..k_unMaxTrackedDeviceCount)` in
`RunFrame`. `UpdateBooleanComponent` sets / clears bit 0 for that device. If the
step-1 spike shows a controller that exposes no boolean trigger, fall back to
hooking `UpdateScalarComponent` on `/input/trigger/value` with 0.7 / 0.3
hysteresis.

## App side

`DriverLink` mirrors `OpenVrTracking`'s surface so `main.cpp` barely changes:

- `poll()` drains **all** complete messages in order and returns
  `Event { connected, disconnected, versionMismatch, error }` for main.cpp to
  log. Button messages are processed in order and accumulate the "second trigger
  goes down while the first is held" rising edge, so a press landing entirely
  between two app frames is not lost (strictly better than today's frame-rate
  sampling of client state).
- `devices()` returns `std::vector<TrackedDevice>`, filtered to valid +
  connected, ascending id — exactly the shape `OpenVrTracking::pollPoses()`
  produces today, so nothing downstream notices the migration. No kind
  filtering here: the driver already sends only HMDs / controllers / trackers.
- `bothTriggersJustPressed()` — same contract as today (stateful, call once per
  frame), computed from state, and unit-tested for the first time.
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
   `GetGenericInterface`, `TrackedDevicePoseUpdated`,
   `CreateBooleanComponent` / `UpdateBooleanComponent`. Logs: device ids /
   classes / serials, boolean component paths per device, pose update rate per
   device, `RunFrame` cadence, every interface version string seen in the
   `GetGenericInterface` detour that we do **not** hook (an unhooked version is
   otherwise a silent no-op, so it must be loud), and our composed world pose
   alongside the client-side `TrackingUniverseRaw` pose for the same device
   (current exe running simultaneously) to **prove the `DriverPose_t`
   composition**. Closes every remaining unknown before real code is designed
   around it.
2. **`src/ipc`**: `ByteChannel`, `MemoryChannel`, `Framer` + `FramerTest` —
   round-trip, messages split across arbitrary read boundaries, several messages
   in one read, garbage / oversized length rejected.
3. **`src/protocol` + `DriverPoseMath`** + `ProtocolTest` (per-type round-trip,
   unknown type skipped for forward compatibility, truncated payload rejected)
   and `DriverPoseMathTest` (identity cases plus the analytic case captured from
   the step-1 log).
4. **`DriverServer`** + `DriverServerTest` against `MemoryChannel` and a fake
   device source: new client gets `Hello` + full `DeviceList`; an unknown id
   flags a metadata refresh; pose / button state produces the expected bytes; a
   busy channel drops whole poses but never metadata; disconnect / reconnect
   resets cleanly.
5. **`DriverLink`** + `DriverLinkTest`: bytes -> `devices()` (kind mapping,
   valid / connected filtering, ordering); trigger edge occurring entirely
   between two `poll()` calls; version mismatch; disconnect empties devices;
   reconnect.
6. **Real pipe channels** (`NamedPipeServerChannel` / `NamedPipeClientChannel`,
   overlapped, `\\.\pipe\TrackingCorrectorDriver`) — the only part not covered
   by unit tests, kept as thin as possible.
7. **DLL wiring**: `DriverFactory` (server provider + watchdog stub,
   `GetInterfaceVersions` -> `vr::k_InterfaceVersions`),
   `ServerTrackedDeviceProvider` (`Init` -> context + hooks + server,
   `RunFrame` -> housekeeping, `Cleanup` -> stop + unhook) and
   `OpenVrGlue` -> `DriverServer`. End-to-end check that the app shows the same
   devices / poses / trigger gesture as before and that calibration, capture and
   recording still work.
8. **`main.cpp` migration**: `OpenVrTracking` -> `DriverLink`, delete `src/vr/`,
   drop openvr from the exe, UI status "Driver: connected / not connected /
   version mismatch". Update `AGENTS.md` (new targets and layer rules, driver
   install and rebuild gotchas) and `doc/plan.md` milestone 4.

## Risks

- **Composition formula** — the one thing that fails silently and subtly. Step 1
  exists for it.
- **Hook coverage vs. load order** — vtable patching affects all instances of
  the interface's concrete class, so other drivers' devices should be visible;
  the `00` prefix plus hooking `GetGenericInterface` (the proven
  SpaceCalibrator path) is the belt and braces. Confirmed in step 1.
- **`RunFrame` cadence is unmeasured** — only metadata and component resolution
  depend on it, so even a slow cadence just means a device shows up a few frames
  late. Measured in step 1.
- **Mixed-driver setups** (e.g. Oculus HMD + Vive trackers) put devices in
  different raw spaces — the problem OpenVR-SpaceCalibrator exists to solve.
  Out of scope; worth a log warning when device drivers differ.
- **Legacy input path disappears** — a bonus: hooking `UpdateBooleanComponent`
  replaces the deprecated `GetControllerState` / `ButtonMaskFromId` code, so the
  calibration gesture no longer depends on SteamVR's legacy binding.
