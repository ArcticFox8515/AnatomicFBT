# OpenVR driver — phase A: move device poses to the driver

Driver half of milestone 4 (`doc/plan.md`). Corrections must be applied driver-side, so
the app can no longer read poses from the OpenVR *client* API — it would read back its
own corrections. Phase A migrates poses + metadata to a SteamVR driver that pushes them
over a named pipe. Pose modification and virtual trackers come later, but the pose hook
is installed now: it is both the source of unmodified poses and the place corrections
will be applied.

## Technology choices

- **Driver DLL**: `openvr_driver.h` from conan `openvr/1.16.8`, **headers only** — the
  driver context helpers are `inline`, so no `openvr_api.lib` link.
- **Hooking**: MinHook (`minhook/1.3.4`), vtable detours per `doc/spacecalibrator-notes.md`
  §1-3: `HmdDriverFactory` serving `IServerTrackedDeviceProvider_004` +
  `IVRWatchdogProvider_001`, `GetInterfaceVersions()` = `vr::k_InterfaceVersions`,
  `VR_INIT_SERVER_DRIVER_CONTEXT` first in `Init`, then a `GetGenericInterface` detour
  that installs per-interface hooks lazily. Vtable indices:
  `IVRDriverContext::GetGenericInterface` 0, `IVRServerDriverHost_006::TrackedDeviceAdded`
  0, `TrackedDevicePoseUpdated` 1. MinHook patches the function body, so all instances
  inside vrserver are intercepted — other drivers' devices included.
- **Metadata**: `IVRProperties_001::TrackedDeviceToPropertyContainer` + `ReadPropertyBatch`,
  which works for devices we do not own (class, serial, role hint).
- **Pose composition** (verified in the step-1 spike against client-side raw poses):
  `world = worldFromDriver ∘ (vecPosition, qRotation) ∘ driverFromHead`. Double precision
  throughout the driver, `f32` on the wire, glm only on the app side.
- **IPC**: one named pipe `\\.\pipe\TrackingCorrectorDriver`, overlapped Win32, driver is
  the server. Explicit security descriptor (vrserver may run elevated). Behind a
  `ByteChannel` seam with a `MemoryChannel` test double.
- **Wire format**: own POD structs, explicit widths, little-endian, framed
  `u32 payloadSize | u8 type | payload` (same style as `.tcrec`). Model types are
  deliberately *not* shared with the driver, so `TrackedDevice` / `.tcrec` can evolve
  independently and the driver links no model code.
- **Coordinate space**: raw driver space, no chaperone/room-setup data. Nothing in
  solving or calibration reads absolute Y or yaw; the skeleton may float relative to
  `Scene`'s grid. Accepted.
- **Logging**: `DriverLib` takes an injected `LogSink` (spdlog-free). The DLL wires it to
  a file sink + `vr::VRDriverLog()`, the exe to spdlog.
- **Buttons**: not captured driver-side — the live run showed neither `IVRDriverInput`
  hooks nor `PollNextEvent` deliver button events. The app keeps a background OpenVR
  client session (`VRApplication_Background`) created on entering Calibration and shut
  down on leaving, for the trigger gesture only. Poses always come from the driver.

## Wire protocol

Driver → app (type codes ≥ 128 reserved for app → driver):

1. `Hello` — magic `"TCIP"`, `u16 version`. Sent on connect; the app refuses a mismatch
   loudly (users forget to reinstall the driver after an update).
2. `DeviceList` — full snapshot, never a delta: `u16 count`, then per device
   `i32 id, u8 kind, u8 roleHint, u16 serialLen, bytes`. Only HMDs, controllers and
   generic trackers; the kind filter moves driver-side. Never dropped, retried from
   `RunFrame`.
3. `PoseBatch` — `u16 count`, then per entry `i32 id, u8 flags (valid | connected),
   f32[3] position, f32[4] rotation (xyzw), f32 poseTimeOffset`. Poses are state, not
   events, so a dropped batch is superseded by the next one.

Driver-side sanity gate before publishing a pose: `poseIsValid`, finite values, non-zero
quaternion, position magnitude bound. (Live run: a tracker reported ~9 km with
`result=Running_OK`; base stations send all-zero quaternions.)

## Structure

```
src/ipc/       ByteChannel.h  MemoryChannel  Framer  NamedPipeServerChannel  NamedPipeClientChannel
src/protocol/  Messages.h  Codec
src/driver/    PoseSink.h  DriverPoseMath.h  DriverServer  Hooks  DriverHooks  DriverProvider
               DriverEnvironment  DriverLog  DriverLogFile  Guard  Interfaces  Names
src/driverdll/ DriverMain.cpp  00trackingcorrector/driver.vrdrivermanifest
src/link/      DriverLink     app side: protocol -> TrackedDevice, device table, reconnect
src/vr/        OpenVrInput    triggers only, lazy init/shutdown
```

| Target | Kind | Deps |
|---|---|---|
| `TrackingCorrectorDriverLib` | STATIC | openvr headers + Win32 only — no model, GL, glm, spdlog, openvr_api |
| `driver_00trackingcorrector` | SHARED | DriverLib + minhook + spdlog |
| `TrackingCorrectorLib` | STATIC | existing + `src/link`, links DriverLib |
| `TrackingCorrector` | WIN32 exe | existing (keeps openvr for input only) |

`DriverServer` is OpenVR-free: the hooks convert `vr::DriverPose_t` into a POD mirror and
call `PoseSink`, so whether poses are written immediately or coalesced and published from
`RunFrame` (the chosen policy: ~100 Hz batch, no IO on the ~1.6 kHz pose threads) is an
implementation detail behind the seam.

Driver-side threading contract: a hook thread never blocks, and no exception ever leaves a
hook or provider entry point (the one sanctioned `catch (...)`). Pose hooks only stamp a
device table under a short lock. `RunFrame` does everything else: accept a pending client,
resolve unknown metadata, publish. One client at a time, newest connection wins.

## Steps

1. **Spike DLL** — done. Established the composition formula, cross-driver hook coverage,
   driver-side metadata by index, pose/`RunFrame` rates, and that buttons are not
   available driver-side.
2. **`src/ipc`** — `ByteChannel`, `MemoryChannel`, `Framer` + tests: round-trip, messages
   split across arbitrary read boundaries, several per read, garbage/oversized length.
3. **`src/protocol` + `DriverPoseMath`** (promoted from `SpikePoseMath`) + tests: per-type
   round-trip, unknown type skipped, truncated payload rejected; identity cases plus the
   numeric case captured from the spike's live run.
4. **`DriverServer`** + `PoseSink` / metadata seams + tests against `MemoryChannel`: new
   client gets `Hello` + full `DeviceList`; unknown id triggers a metadata refresh; batch
   bytes exact; busy channel drops poses but never metadata; every sanity-gate branch;
   disconnect/reconnect resets. Plus the written concurrency argument for the new state.
5. **`DriverLink`** + tests: bytes → `devices()` (kind mapping, valid+connected filter,
   ascending id), version mismatch, disconnect empties, reconnect. `TrackedDevice`,
   `.tcrec`, `ModeController`, `TrackerCalibration`, `SessionRecorder`, `ReplaySession`
   and existing recordings stay untouched.
6. **Real pipe channels** — overlapped `NamedPipeServerChannel` / `NamedPipeClientChannel`,
   the only part not covered by unit tests, kept as thin as possible. App reconnects with
   a non-blocking `CreateFile` at most every ~500 ms.
7. **Promote the spike into the real driver** — move `src/spike` to `src/driver` +
   `src/driverdll`, delete the observer and the spike client, point the detours at
   `PoseSink`, keep the adapter's one-instruction rule, restructure CMake. Close the two
   open concurrency defects: `original_` becomes `std::atomic<void*>` read once per detour,
   and `remove()` stops clearing it (teardown race). Live SteamVR run: same devices and
   poses as before, calibration/capture/recording still work. Run `ctest -C Release` too.
8. **`main.cpp` migration** — poses from `DriverLink`, trigger gesture from `OpenVrInput`,
   start in ManualPose and switch to Calibration on the first connect, UI status
   `Driver: connected / not connected / version mismatch`. Update `AGENTS.md` and
   `doc/plan.md` milestone 4.

## Risks

- **Pipe ACL** when vrserver runs elevated — the app must not be locked out; distinguish
  `ERROR_ACCESS_DENIED` from `ERROR_PIPE_BUSY` in the log.
- **Interface versions in the wild differ** from the ones we build against (`_005` and
  `_004` were seen alongside our `_006` / `_003`). Every unhooked version string must be
  logged loudly — a missed one is a silent no-op.
- **Mixed-driver setups** (e.g. Oculus HMD + Vive trackers) put devices in different raw
  spaces — what OpenVR-SpaceCalibrator exists to solve. Out of scope; warn when device
  drivers differ.
- **`raw` ≈ `standing` on the test machine**, so the live run did not actually
  discriminate the two universes. The coordinate-space assumption is still an assumption.
- **Trigger capture stays on the legacy client binding path**, since no reliable
  driver-side input source exists.
