# OpenVR driver — phase A: move device poses to the driver

Driver half of milestone 4 (`doc/plan.md`). Corrections must be applied driver-side, so
the app can no longer read poses from the OpenVR *client* API — it would read back its
own corrections. Phase A migrates poses and device metadata to a SteamVR driver that
pushes them to the app. Pose modification and virtual trackers come later, but the pose
hook is installed now: it is both the source of unmodified poses and the place
corrections will be applied.

## Technology choices

- **SteamVR driver DLL**, `openvr_driver.h` from conan `openvr/1.16.8`, **headers only**:
  the driver context helpers are `inline`, so nothing links `openvr_api.lib`.
- **MinHook** (`minhook/1.3.4`) vtable detours, per `doc/spacecalibrator-notes.md` §1-3:
  hook `IVRDriverContext::GetGenericInterface` (slot 0) and install per-interface hooks
  lazily as version strings pass by; `IVRServerDriverHost_006::TrackedDeviceAdded` is
  slot 0, `TrackedDevicePoseUpdated` slot 1. MinHook patches the function body, so every
  instance inside vrserver is intercepted — other drivers' devices included.
- **Device metadata via `IVRProperties`** (`TrackedDeviceToPropertyContainer` +
  `ReadPropertyBatch`), which works for devices we do not own.
- **Pose composition** — verified in the step-1 spike against client-side raw poses:
  `world = worldFromDriver ∘ (vecPosition, qRotation) ∘ driverFromHead`. Double precision
  driver-side, `f32` on the wire, glm only in the app.
- **Raw driver space**, no chaperone / room-setup data: nothing in solving or calibration
  reads absolute Y or yaw. The skeleton may float relative to the render grid. Accepted.
- **Transport: one overlapped named pipe**, driver is the server, explicit security
  descriptor (vrserver may run elevated). Behind an interface, with an in-memory
  implementation for tests.
- **Own binary wire format** — POD structs, explicit widths, little-endian,
  length-prefixed frames, same style as `.tcrec`. Model types are deliberately not shared
  with the driver, so the app's types can evolve independently and the driver links no
  model code. Poses are state, not events, so a dropped update is superseded by the next
  one.
- **Buttons stay client-side**: the live run showed neither `IVRDriverInput` hooks nor
  `PollNextEvent` deliver button events to a driver. The app creates a background OpenVR
  client session (`VRApplication_Background`) when entering Calibration and shuts it down
  on leaving, for the trigger gesture only. Poses always come from the driver.
- **Logging**: injected sink, so the shared code stays spdlog-free; the DLL wires it to a
  file plus `IVRDriverLog`, the app to spdlog.

## Steps

1. **Spike DLL** — done. Established the composition formula, cross-driver hook coverage,
   metadata by device index, pose / `RunFrame` rates, and that buttons are unavailable
   driver-side.
2. **IPC layer** — `Pipe` transport seam (one method per Win32 named-pipe file
   handle op: `write`/`read`/`close`, poll-based, never blocks), length-prefixed
   framing + reassembly (`MessageChannel`) over it, and the wire protocol's two
   messages as memcpy'd PODs (`DeviceMetadata`, `DevicePose`). Tests cover split
   reads, several frames per read, header split mid-field, partial writes,
   unknown-type skip, oversize-length failure, and peer-close. The mock pipe
   lives in the test target; the real Win32 pipe is step 5.
3. **Driver-side server** — device table, metadata resolution, publish policy, client
   lifecycle, sanity gates on incoming poses; tested against the in-memory channel, plus
   a written argument for the concurrency of its shared state.
4. **App-side link** — bytes to device snapshots (kind mapping, validity filter,
   ordering), disconnect, reconnect. The app's existing device type,
   recording format, mode/calibration/replay logic and old recordings stay untouched.
5. **Real pipe implementation** — the only part not covered by unit tests, kept as thin
   as possible; the `Pipe` seam is 1:1 with the overlapped winapi call pairs
   (`startConnect`/`completeConnect`, `startRead`/`completeRead`,
   `startWrite`/`completeWrite`, `close`) and `MessageChannel` owns all the
   overlapped state (connection state machine, read buffer, stable write buffer,
   in-flight flags). Server half done (`SpikeDriverPipe.cpp`, driver DLL only);
   client half + app reconnect outstanding. `classifyIo` (pure, tested) maps
   winapi returns so the DLL forwarder has no decisions.
6. **Promote the spike into the real driver** — reuse its hook and provider machinery,
   drop the observation-only code and the spike client, point the detours at the transport
   layer, close the two known concurrency defects (unsynchronized publication of the
   trampoline pointer, and clearing it during teardown while pose threads run). Live
   SteamVR run; `ctest -C Release` as well as Debug.
7. **App migration** — poses from the driver link, trigger gesture from the background
   session, connection status in the UI, docs updated.

## Risks

- **Pipe ACL** when vrserver runs elevated — the app must not be locked out; distinguish
  access-denied from pipe-busy in the log.
- **Interface versions in the wild differ** from the ones we build against (`_005` and
  `_004` seen alongside our `_006` / `_003`). Every unhooked version string must be logged
  loudly — a missed one is a silent no-op.
- **Invalid poses that claim to be valid**: a tracker reported ~9 km with
  `result=Running_OK`, base stations send all-zero quaternions. Gate before publishing.
- **Mixed-driver setups** (e.g. Oculus HMD + Vive trackers) put devices in different raw
  spaces — what OpenVR-SpaceCalibrator exists to solve. Out of scope; warn when device
  drivers differ.
- **`raw` ≈ `standing` on the test machine**, so the live run did not discriminate the two
  universes. The coordinate-space choice is still an assumption.
- **Trigger capture stays on the legacy client binding path**, as no reliable driver-side
  input source exists.
