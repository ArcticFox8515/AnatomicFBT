# AGENTS.md — TrackingCorrector

## What this project is

Windows desktop app (C++20) for VR full-body tracking correction. Loads a humanoid
skeleton from JSON, renders it with OpenGL, drives it with IK from tracked devices,
retargets the pose onto a Unity avatar skeleton. Long-term goal: correct tracking at
SteamVR driver level and emit virtual trackers. Design docs: `doc/plan.md` (milestones),
`doc/driver-plan.md` (SteamVR driver, milestone 4), `doc/spacecalibrator-notes.md`
(reference driver findings), `doc/ik-improvements-plan.md` (IK quality work
packages, long-living).

## Build & test

Conan 2 + Visual Studio 2022. Windows-only (`WIN32` exe, `WinMain` entry).

- `generate-project.bat` — `conan install` (Debug + Release) into `build/`, then CMake
  configure with the Conan toolchain. Run first, and after dependency changes.
- Build: `cmake --build build --config Debug`, or open `build/TrackingCorrector.sln`.
- `run-tests.bat` — `ctest --test-dir build -C Debug --output-on-failure`. Run
  `ctest -C Release` too for anything touching the driver hooks (Release devirtualizes
  the fake vrserver's calls, so only it exercises the patched bodies).
- `CMakeUserPresets.json` and `src/bindings/` are generated — never hand-edit.
- Runtime files created in the working directory: `user-proportions.json`,
  `user-ikrig.json`, `user-avatar-skeleton.json`, `user-settings.json`
  (load-or-create; invalid files fall back to defaults with the error
  logged), `logs/trackingcorrector.log` (rotating 3×5MB), `recording.tcrec`
  (every capture session, overwritten per session).

### SteamVR driver

- `build-driver.bat` — Release build of the driver DLL. Only the Release DLL is staged
  to `build/driver/00trackingcorrector/bin/win64/` (the path SteamVR loads); other
  configs keep the default output dir, so a Debug solution build cannot replace the DLL
  SteamVR is using.
- `install-driver.bat` / `uninstall-driver.bat` — `vrpathreg adddriver|removedriver` on
  `build/driver/00trackingcorrector`; SteamVR located via
  `%LOCALAPPDATA%\openvr\openvrpaths.vrpath`. Nothing is copied — SteamVR loads out of
  the build tree. Restart SteamVR after (un)installing.
- **Close Steam itself, not just SteamVR, before a Release driver build**: `steam.exe`
  loads the watchdog provider and holds the DLL open, so the link fails with `LNK1104`.
- Driver logs: `%LOCALAPPDATA%\TrackingCorrector\driver-<process>.log` (one per
  loading process: `vrserver`, `vrwatchdog`, `TrackingCorrectorTests`), plus the same
  lines in SteamVR's `vrserver.txt` via `IVRDriverLog`.
- Directory name, manifest `name` and DLL name must agree (`00trackingcorrector` /
  `driver_00trackingcorrector.dll`). The `00` prefix makes SteamVR load us first.

## Targets (`CMakeLists.txt`)

| Target | Kind | Contents / deps |
|---|---|---|
| `TrackingCorrectorLib` | STATIC | `src/model` + `src/view`; glm, json, GLEW, glfw, `LinkLib` (for `OpenVrTracking`) |
| `LinkLib` | STATIC | `src/link` IPC seam + framing + protocol + `Logger`; std lib only (Win32 error constants as literals) |
| `TrackingCorrector` | WIN32 exe | `src/main.cpp`, `src/vr`, imgui/imguizmo, spdlog, openvr, `PipeLib` |
| `PipeLib` | STATIC | `src/pipe/Win32Pipe.{h,cpp}` server+client `link::Pipe` impls; links `LinkLib`; Win32 only, untested |
| `DriverLib` | STATIC | all `src/driver` logic; openvr headers only; links `LinkLib` (step 3 server) |
| `driver_00trackingcorrector` | SHARED | `src/driver/Driver.cpp` + DriverLib, minhook, spdlog, `PipeLib` |
| `TrackingCorrectorTests` | exe | `tests/`, gtest; add new suites here |

## Directory map

```
src/model/      Pure data + math + JSON. NO OpenGL/OpenVR — unit-testable.
                `OpenVrTracking` (poses from the driver link) lives here too:
                it owns a `link::MessageChannel` and folds `link::DevicePose`
                frames into `TrackedDevice` snapshots, so the lib links
                `LinkLib` (std-only, no GL).
src/view/       All OpenGL rendering (Scene).
src/vr/         `OpenVrInput` — on-demand OpenVR background client used only in
                Calibration mode for the both-triggers gesture (no driver-side
                input source exists). The only place openvr.h is included.
                main owns it as a `unique_ptr`, created on entering
                Calibration, destroyed on leaving.
src/link/       Driver<->app IPC: `Pipe` transport seam (overlapped call pairs —
                startConnect/completeConnect, startRead/completeRead,
                startWrite/completeWrite, close — one winapi call per method,
                caller owns buffers and in-flight state), `MessageChannel`
                framing/reassembly over it (owns the connection state machine,
                a `Message` per direction, in-flight flags, a `std::mutex`),
                the wire protocol's `Message`/`DevicePose` (memcpy'd POD), and
                the `Logger`/`log()` used by the driver. `classifyIo`
                maps winapi returns to `IoStatus` (literals, no windows.h).
                Standard library only — no model, no glm, no openvr, no
                spdlog — so the driver DLL links no model code. Driver code
                uses `link::` symbols directly.
src/pipe/       Win32 named-pipe implementations of `link::Pipe` (`Win32ServerPipe`,
                `Win32ClientPipe`), each taking its pipe name as a constructor
                argument. Linked by the driver DLL and the tests; the only part
                of the link layer not covered by unit tests.
src/driver/     The SteamVR driver (doc/driver-plan.md, doc/virtual-trackers-plan.md):
                hooks GetGenericInterface / TrackedDeviceAdded /
                TrackedDevicePoseUpdated, reads device metadata via IVRProperties,
                and on each pose forwards it to a `link::MessageChannel`
                (driver-side server) so the app can consume driver-side poses,
                then applies any app-supplied `PoseOverride` by premultiplying
                `worldFromDriver` before forwarding the pose to SteamVR. Own
                virtual trackers are filtered out of the downstream stream by
                `Prop_TrackingSystemName_String == kOurTrackingSystemName` (step 7),
                so the app never sees them as tracked devices. Creates and drives
                virtual trackers (step 6): one `ITrackedDeviceServerDriver` per
                bone the app emits, registered with `TrackedDeviceAdded`, props
                set post-`Activate`, poses pushed each frame, disconnected by
                staleness timeout. All logic in DriverLib; Driver.cpp is a pure
                adapter. Input capture is NOT part of it (neither IVRDriverInput
                hooks nor PollNextEvent deliver buttons).
src/driverdll/00trackingcorrector/driver.vrdrivermanifest  — copied next to bin/win64/.
src/bindings/   Vendored ImGui backends, auto-copied by conanfile.py. Generated.
tests/          GoogleTest suites mirroring src/model, src/link, src/pipe and src/driver.
                `FakePipe.h` is the `link::Pipe` mock used by the link suites, the
                driver server pipe-lifecycle tests (`borrowPipeFactory`) and
                `OpenVrTrackingTest` (app-side pose source over the channel).
                `LinkPipeIntegrationTest.cpp` is the one real Win32-pipe test.
unity/          Standalone Unity Editor tooling (C#), not part of the C++ build.
```

## Model layer (`src/model/`)

- `Error` — exception type for all our errors; `what()` includes the throw site.
- `GlmJson.h` — `to_json`/`from_json` for `glm::vec3`, in `namespace glm` for ADL.
- `BoneNames.h` — every default bone name as `constexpr const char*`. Names follow
  Unity's `HumanBodyBones`, so an exported avatar retargets by name with no translation.
- `BodyProportions` — 9 tape-measurable floats (`user-proportions.json`), symmetric L/R,
  meters. No skeleton knowledge; `validate()` checks positive lengths and landmark order.
- `AppSettings` — app-wide user settings (`user-settings.json`), shaped so
  unrelated settings can be added later without a schema break. Today holds
  only the per-bone virtual-tracker selection
  (`virtualTrackerBones`, names from the step-1 eligible list; empty by
  default — no bone emits on a fresh install). `from_json` is
  forward-tolerant: a missing `virtualTrackers` block is the empty
  selection (so a file written before this feature still loads), and
  unknown top-level/nested keys are ignored; a present-but-malformed block
  throws and the caller falls back to the default. Same load-or-create
  convention as the other configs. The UI (step 4) lists the step-1
  candidate bones with a checkbox each in the ImGui panel; the selection
  is editable only before calibration completes (locked once
  `TrackerCalibration::isCalibrated()`, re-enabled on recalibration) and
  persists immediately to `user-settings.json` on every tick.
- `Skeleton` — flat `vector<Joint>` sorted parent-before-child. `Joint = {name,
  parentIndex, restOffset, localRot}`; `localRot(i)` = rotation of the bone *ending* at
  joint i, relative to the parent's world orientation. `localRot` and `rootPosition` are
  runtime-only (never serialized); rest orientation is always identity.
  `makeDefault(proportions)` = 22-joint head-rooted skeleton (spine downward);
  `makeDefaultHipRooted()` = same rest positions re-rooted at Hips (avatar default).
  `computeWorldTransforms` = hierarchical FK; `computeRestPositions` = the same
  but with `localRot` ignored (root at origin) — pose-independent, used for landmark
  measurements. `restHeight` = Head-to-feet Y span of the rest pose (by bone name;
  0 when Head or both feet are missing). `scaleSkeleton` uniformly scales every
  restOffset + rootPosition; `matchRestHeight(src, dst)` scales dst so
  `restHeight(dst) == restHeight(src)` (returns 1.0, dst untouched, when either
  height is unusable). `computeBoneFrames` = per-bone `{base, rotation, length,
  joint}` for rendering: `rotation = wt.rotations[i] * quatFromTo(+Y, restDir)`,
  the `quatFromTo` factor constant per bone so the `-Y` antipode (every downward
  bone) is a fixed constant instead of a per-frame singularity, and the frame
  carries the joint's real axial twist. JSON schema:
  `{"bones":[{name, parent|null, offset:[x,y,z]}]}`; `from_json` throws `Error` on
  duplicate names, unknown parents, cycles, or not exactly one root.
- `IkMath` — `solveTwoBoneIk` (closed-form, pole vector, stretches on overreach),
  `clampSwingTwist`, `quatFromTo`. No skeleton knowledge.
- `IkRigConfig` — which bones take targets and how (`anchor|chain|two_bone`) plus
  per-bone `JointLimits` (twist range, swing cone, optional `pole` — required on the
  middle bone of a two-bone chain — plus `poleMode` selecting how the bend normal is
  derived per frame: `static` (default for forward compat), `dynamic_foot`
  /`dynamic_hand` — the bend normal is `flexSign * cross(targetRot * lateralAxis,
  aim)`, perpendicular to the chain aim by construction so no pole‖aim degeneracy;
  the static `pole` is retained only as the singularity guard and the flex-sign
  reference). `validate()` throws `Error`.
- `IkSolvers` — `IkTarget{jointIndex, position, rotation}` + `solveAnchor` (pins the
  root), `solveChain` (spine: end bone takes the target rotation exactly, rest arcs),
  `solveTwoBone` (limb socket→j1→j2→tip; `solveTwoBoneIk` returns rest-relative
  rotations that compose onto the socket frame; takes the pole in the socket's frame).
- `IkRig` — owns skeleton + config + targets; no bone names in code (structure derived
  from config + topology). Construction never throws; `loadConfig` validates and throws
  `Error`, keeping the previous config on failure. `solve()` re-derives the pose from
  the targets every call; `solve(goals)` consumes an explicit goal vector. Stage order:
  anchors → chains → two-bone limbs (dynamic bend normals computed per-frame here per
  the binding's `poleMode`: the foot/hand target's medial-lateral axis crossed with the
  chain aim, `flexSign`-corrected; `sideSign`/`flexSign` derived once at bind time from
  the socket rest offset and the static pole) → joint-limit clamp → end-effector re-aim
  (policy: tracked rotation wins over limits on end bones — a limit bends mid-bones
  but never rotates the anchor root / chain end / two-bone tip away from its goal).
- `Retarget` — `buildRetargetMap` matches dst bones to src bones by unordered joint-name
  pair (so head-rooted src drives hip-rooted dst); `retargetPose` copies world rotations
  and shifts `dst.rootPosition` so the anchor joint lands on its src position.
- `Pose` — header-only `{position, rotation}` + `compose`/`inverse`/`yawOnly`.
- `TrackedDevice` — hardware-agnostic snapshot: `{id, kind, pose}`, `kind` =
  `hmd|controller|tracker|other`; `id` is an opaque stable id from the provider.
- `OpenVrTracking` — the app's pose source (implements `IPoseSource`,
  model/FrameTick.h): owns a `link::MessageChannel`
  (client side) and folds `link::DevicePose` frames into `TrackedDevice`
  snapshots. `init()` throws `Error` when the driver pipe is not connected
  (retryable); `pollPoses()` drains and returns the snapshot (ordered by device
  id, `Other` skipped, `Lost` removes — same observable filter as the old
  client-API poll; non-`DevicePose` types are skipped); `isInitialized()` =
  currently connected. `sendOffsets` ships one `PoseOverride` per device per
  frame upstream through the same duplex channel. The pipe factory
  (`Win32ClientPipe` on `kDriverPipeName`) and a clock fn (1/s reconnect
  throttle) are ctor args injected by the exe.
- `TrackerCalibration` — device→target binding, OpenVR-free. Proximity assignment
  (user T-poses), `calibrate` stores `offset = inverse(devicePose) * boneWorldPose`,
  `applyDevicePoses` writes raw device poses into targets (what gets rendered),
  `applyOffsets` produces solver goals on a copy, `deviceInBone` exposes the tracker's
  pose in its bone's local frame (inverse of the offset, the constant strap
  `TrackerCorrection` re-hangs on the avatar). `updateCalibrationFrame` rests the
  skeleton and places the root: in T-pose the midpoint of the two controllers coincides
  with the Chest joint (exact invariant of the rest skeleton), so the root is shifted to
  land FK Chest there, masked in the HMD-yaw frame (height/forward from the hands,
  lateral from the HMD); falls back to plain HMD alignment.
- `TrackerCorrection` — app-side re-placement of tracker poses onto the
  avatar skeleton (milestone 4, existing-tracker correction). `buildCorrectionMap`
  matches each IK target to an avatar joint by name (built once at startup) and
  classifies it into a `CorrectionGroup` (`Hands`, `Feet`, `Hips`, `Knees`,
  `Elbows`, `Chest` — the last three forward-declared for future knee/elbow/
  chest targets) via `correctionGroupForBone`. Enable/rotation toggles are
  **per group**, not per target — there is no real case for correcting one
  leg but not the other, so one checkbox pair (Enable + Rot) controls every
  member of the group. `correctDevicePoses` (takes the live `devices`
  snapshot) places each mapped, bound target whose group is enabled at the
  avatar joint's world pose — the bone center, no strap offset (one FK pass
  over the avatar, called after `retargetPose`). Controllers always keep
  their raw rotation (aiming must not change); trackers keep it too when the
  group's `groupRotationEnabled` is false (useful when avatar bone-roll
  differences produce a visually wrong tracker orientation) —
  `CorrectedPose::rotationLocked` records this. A device bound but absent
  from the snapshot is skipped (no stale marker). The bone-local offset
  captured at calibration is deliberately dropped: the reference and avatar
  skeletons have differently-oriented bone frames (rest rotations / bone roll),
  so re-hanging in the local frame rotates the tracker wrong. No GL/OpenVR; the
  app renders the corrected poses as markers in the right viewport. The avatar
  skeleton is height-scaled to the reference once at startup (`matchRestHeight`,
  before the maps are built) so overall height is preserved and correction only
  redistributes proportions — no 180 cm body squashed into a 150 cm avatar.
  `correctionOffsets` computes the rigid world-space delta per corrected device
  (`delta = compose(corrected, inverse(raw))`, or an exactly identity-rotation
  translation when `rotationLocked`); `OpenVrTracking::sendOffsets` ships one
  `PoseOverride` per device each frame through the same duplex channel the
  driver link uses.
- `VirtualTrackers` — `eligibleVirtualTrackerBones(rig, avatar)`
  (doc/virtual-trackers-plan.md step 1): the set of bones a virtual tracker
  can be emitted for — joints present in BOTH the source and the avatar
  skeleton, minus joints that are IK targets (those are "mapped to
  trackers": Head is the HMD anchor, hands/feet/Hips are tracker targets).
  Exclusion is by target configuration, not device binding, so the list is
  stable across calibration state. Name-only intersection, so a
  height-scaled avatar yields the same set; returned in source-skeleton
  order (parent before child) for stable UI listing. Model layer only, no
  GL/OpenVR. `computeVirtualTrackerPoses(avatar, boneNames)` (step 2)
  produces the emitted pose per selected bone from the **avatar's** current
  pose (run after `retargetPose`): position = the midpoint of the bone's two
  joint world positions (the avatar joint plus its parent), rotation = that
  avatar joint's world rotation. Recomputed every frame — the source is the
  live avatar FK, not the rest pose, so positions track the posed/rescaled
  avatar. A bone not in the avatar or whose avatar joint is the root (no
  parent) is skipped.
- `ModeController` — ManualPose/Calibration/Capture/Replay state machine, hardware-free.
  `update(rig, devices, captureGesture)` mutates the rig and returns
  `FramePlan{solve, goals, capturedOffsets}`. Invariant: Capture frames, including the
  calibration→capture transition, always produce goals derived from that same frame.
  Replay reuses the Capture branch verbatim; `calibrateFromFrame` re-runs the live
  calibration path on a recording's frame 0. `calibration()` exposes the owned
  `TrackerCalibration` for the app's correction pass.
- `Recording` — binary `.tcrec` capture of the device snapshots fed to `update` (inputs
  only). Magic+version, roster frozen on frame 0, then fixed-size self-contained frames
  (absent device = last known pose). Streamed, randomly seekable; loader throws `Error`
  on malformed input but drops a truncated trailing frame.
- `SessionRecorder` — recording lifecycle over an injected `StreamFactory`; never throws,
  reports via `Event{started, stopped, error}`.
- `ReplaySession` — recording file list (`scan`), loaded recording, timeline position;
  `load` recalibrates from frame 0, `currentDevices()` feeds `update`.
- `FrameTick` — the per-frame orchestration shared by the visible and the
  minimized frame paths, extracted so the driver link stays fed (poses polled,
  overrides shipped) whether the window renders or not. Defines the `IPoseSource`
  and `IGestureSource` seams (`OpenVrTracking` / `OpenVrInput` implement them) so
  the sequencing is unit-testable without OpenVR/pipe bindings. `pollAndUpdate`
  (the head: device-source selection by mode — Replay pulls from the loaded
  recording, otherwise from the pose source when connected; reads the
  both-triggers gesture only in Calibration; advances the mode controller; runs
  the recorder) returns `UpdateResult{devices, plan, tearDownGestureSource}` and
  reports events (calibration capture, recording start/stop/error) as
  fully-formatted lines through the injected `link::Logger` — same sink the
  driver link layer uses; the app forwards each line to spdlog exactly as it
  wires `link::Logger`'s sink for `OpenVrTracking`. The caller tears down the
  trigger reader when `tearDownGestureSource` is set (the model owns no VR
  session). `retargetAndShip` (the tail: retargetPose + correctDevicePoses +
  sendOffsets when connected + sendVirtualTrackers when Capture+calibrated)
  returns the corrected poses for rendering. Main calls these in both paths;
  the minimized path skips the GL/ImGui/gizmo work and the Targets (ManualPose)
  solve, which is gizmo-driven and render-interleaved. Virtual-tracker poses
  ship upstream only in Capture after calibration (`mode == Capture &&
  calibration.isCalibrated()`), so no VT traffic reaches the driver in any
  other mode — both the visible and minimized paths gate identically.

## Link layer (`src/link/`)

- `Pipe` — transport seam, one method per Win32 overlapped named-pipe op
  (`startConnect`/`completeConnect`, `startRead`/`completeRead`,
  `startWrite`/`completeWrite`, `close`; `IoStatus` =
  `Ok|Pending|Closed|Failed`). An unconnected byte stream; `PipeFactoryFn`
  (`std::function<std::shared_ptr<Pipe>()>`) constructs an instance, and
  `MessageChannel` drives the connection state machine. The pipe name is a
  constructor argument of the concrete pipe (`Win32ServerPipe` /
  `Win32ClientPipe` in PipeLib), not a `createPipe` argument. Every overlapped
  method takes the `std::vector<unsigned char>&` the channel owns; the
  implementation casts `.data()`. On `Pending` the op is in flight — the
  channel keeps the buffer/state and polls `completeX` on the next frame. At
  most one op in flight per direction; the caller (MessageChannel, under test)
  enforces this. `shared_ptr` so a `send` on the hook thread can snapshot the
  pointer across a concurrent `receive` drop. The real Win32 forwarder
  (`src/pipe/Win32Pipe.cpp`, driver DLL + tests) is the only part not
  unit-tested; `classifyIo` (pure, tested) maps winapi returns to `IoStatus`
  so the forwarder has no decisions.
- `Protocol` — wire types only (no codec): `DevicePose`
  (id+tracking+kind+pos+rot+serial[32]) downstream and `PoseOverride`
  (id+pos+rot) plus `VirtualTracker` (name[32]+tracking+pos+rot) upstream, all
  naturally-aligned PODs, memcpy'd whole — same style as `.tcrec`'s
  `writeRaw`/`readRaw` but one struct copy. `Message` is the frame in memory:
  `u32 size` (payload length, sender-filled), `u16 type`, then a payload union
  (`DevicePose` / `PoseOverride` / `VirtualTracker`). The separate
  `DeviceMetadata` message was folded into `DevicePose` in step 3; wire type 1
  from an older driver is silently skipped (forward compat). `PoseOverride`
  carries a rigid world-space delta: the driver premultiplies
  `worldFromDriver` by it so vrserver's prediction (which runs in driver-local
  space on the untouched local pose and velocities) stays exact.
  `VirtualTracker` (type 4, step 5) is one frame per ticked eligible bone the
  app emits while in Capture after calibration: the bone name is the device
  identity (no compile-time slot list — the set of names arriving this frame
  is the roster), `tracking` is always `Tracking` on the wire (the app simply
  stops sending when it leaves Capture and the driver's staleness disconnects
  the device). Step 6: the driver's `VirtualTrackerProvider` consumes these
  frames — one `ITrackedDeviceServerDriver` per bone name, registered with
  `TrackedDeviceAdded`, serial `TC-<boneName>`, props written post-`Activate`,
  poses pushed each frame with identity `worldFromDriver`/`driverFromHead`
  and zero translation/velocities, disconnected by a `kVirtualTrackerStaleSeconds`
  timeout (0.5 s) or pipe drop. Step 7: the Observer reads
  `Prop_TrackingSystemName_String` in its metadata refresh and marks devices
  whose tracking system matches `kOurTrackingSystemName` as `isOurs`; `onPose`
  skips those devices entirely (no downstream `DevicePose` frame, no override
  application), so the app never sees them as tracked devices. `DeviceKind`/`TrackingState`/`MessageType` are this layer's
  own enums with pinned wire values; the driver maps
  `ETrackedDeviceClass` -> `DeviceKind`, the app maps `DeviceKind` ->
  `TrackedDeviceKind`. `tracking` collapses the two booleans the app ANDs in
  `OpenVrTracking::pollPoses`; zero = drop the device this frame (it then
  holds its last pose).
- `Logger` — `LogSink`, `compositeSink`, `Logger`, `log()`, `loggingTo()` (in
  `namespace link`). Driver code uses these `link::` symbols directly.
  `MessageChannel` takes `Logger&` and logs five lines: pipe
  create/`createEvent` failure with the win32 code, client connected, pipe dropped with
  reason and code, size above the cap, frame of an unexpected type skipped.
- `MessageChannel` — length-prefixed framing (u32 length, u16 type, payload)
  + reassembly over a `Pipe`. Takes a `Logger&` and a `PipeFactoryFn`:
  constructs an *unconnected* pipe via the factory from `receive()`, drives
  `startConnect`/`completeConnect` until `Connected`, then reads/writes. Owns
  a `Message` per direction plus per-direction offset/in-flight flags and a
  `std::mutex` locked as the first statement of `send`/`receive`. `send(const
  Message&)` does one `completeWrite` when a write is in flight, then one
  `startWrite` of the tail if bytes remain; if the tail cleared it copies the
  new message and starts one write; no loop, nothing returned. `receive`
  pumps connect, one write step, then drains the pipe (one `completeRead` or
  `startRead` per iteration into `readMessage_` for exactly the bytes still
  missing — header first, then `size` payload bytes) until Pending or empty;
  each complete frame of a known type is appended to the out vector. Unknown
  types are skipped (consumed, not emitted); `size` above the payload union
  (bounded by `sizeof(Message) - header`, no named cap constant) drops the
  pipe (stream sync unrecoverable for this client, the next one gets a fresh
  start). `lastError()` exposes the drop reason.

## View, VR, entry point

- `Scene` (`src/view/`) — owns ALL GL resources (RAII: created after GLEW init, destroyed
  before GLFW shutdown — keep the scope in `main.cpp` intact). Orbit camera shared by
  both viewports (`setCameraTarget`/`setCameraYaw` for VR modes), `beginFrame` +
  `setViewport` per half, `renderSkeleton` (pyramid per bone via
  `computeBoneFrames` — joint world rotation composed with a constant
  per-bone `+Y -> restDir` factor, so roll tracks real twist and stays
  continuous near downward bones), `renderMarkers`
  (octahedron per pose; `renderTargets(rig)` delegates to it via `targetPoses`),
  two embedded GLSL 330 programs, `viewMatrix`/`projectionMatrix`
  for ImGuizmo.
- `OpenVrTracking` (`src/model/`) — the app's pose source: owns a
  `link::MessageChannel` (client side) and folds `link::DevicePose` frames into
  `TrackedDevice` snapshots. The pipe factory (`Win32ClientPipe` on
  `kDriverPipeName`) and a clock fn (for the 1/s reconnect throttle) are ctor
  args injected by the exe. `init()` creates the channel and pumps once, throws
  `Error` when the driver pipe is not connected (retryable). `pollPoses()`
  drains pending frames and returns the snapshot (ordered by device id,
  `Other` skipped, `Lost` removes — same observable filter as the old
  client-API poll; non-`DevicePose` types are skipped). `isInitialized()` =
  currently connected. `sendOffsets` ships one `PoseOverride` per device each
  frame upstream through the same duplex channel the driver link uses.
  `sendVirtualTrackers` (step 5) ships one `VirtualTracker` frame per ticked
  eligible bone — name (the device identity), tracking, and the avatar-derived
  world pose — only called by `retargetAndShip` in Capture after calibration.
- `OpenVrInput` (`src/vr/`) — on-demand OpenVR background client for the
  both-triggers gesture only (no driver-side input source exists). `init()`
  uses `VRApplication_Background` (never launches SteamVR), throws `Error`.
  Implements `IGestureSource` (model/FrameTick.h) so the frame tick reads the
  gesture through a seam without itself depending on openvr. main owns it as
  a `unique_ptr`, created on entering Calibration, destroyed on leaving
  (including the automatic Calibration→Capture transition). The only place
  openvr.h is included.
- `src/main.cpp` — spdlog → GLFW/GL/GLEW → ImGui + ImGuizmo → load-or-create the three
  configs → `IkRig` → avatar skeleton, `matchRestHeight` (scale avatar to reference
  rest height) + `RetargetMap` + `CorrectionMap` → `link::Logger` (sink into
  spdlog) + `OpenVrTracking` (pipe factory + clock; implements `IPoseSource`) →
  `OpenVrTracking::init` + create `OpenVrInput` (implements `IGestureSource`;
  success: start in Calibration; failure: ManualPose). Per frame: `pollAndUpdate`
  (FrameTick — shared by both paths), tear down `OpenVrInput` when the result's
  `tearDownGestureSource` is set, execute the returned plan, camera follow, left
  viewport (IK skeleton + gizmos in ManualPose), `retargetAndShip` (FrameTick —
  retarget + correct + ship the deltas + ship virtual trackers to the driver)
  + right viewport (avatar)
  + corrected tracker markers, ImGui panel (with per-group correction
  checkboxes + rotation toggle when calibrated + read-only avatar scale
  readout), replay timeline. Minimized frames (no framebuffer) run the same
  `pollAndUpdate` + Goals `solve` + `retargetAndShip`, skipping GL/ImGui/gizmo
  work and the Targets (ManualPose) solve — so the driver link stays fed while
  the window is minimized. All mode logic lives in the model layer.

## Libraries (Conan, see `conanfile.py`)

| Library | Version | Used for |
|---|---|---|
| glfw | 3.4 | Window + input |
| glew | 2.2.0 | GL loading (`IMGUI_IMPL_OPENGL_LOADER_GLEW`) |
| glm | 1.0.1 | vec/mat/quat math |
| imgui | 1.90.5-**docking** | UI; `force=True` overrides imguizmo's pin — read the conanfile comment before changing |
| imguizmo | cci.20231114 | Target gizmos |
| nlohmann_json | 3.11.3 | Config (de)serialization |
| spdlog | 1.15.3 | Logging; exe + driver DLL only, model layer stays log-free |
| openvr | 1.16.8 | Client poses/input (exe, `src/vr` only); `openvr_driver.h` **headers only** in the DLL (no `openvr_api` link) |
| minhook | 1.3.4 | Vtable detours inside `vrserver.exe`; driver DLL only |
| gtest | 1.15.0 | Tests, via `gtest_discover_tests` |

## Conventions

- **Separation**: model layer stays GL-free (tests link only `TrackingCorrectorLib` and
  create no GL context); view/app depend on model, never the reverse.
- Coordinates: right-handed, **Y-up, meters**, left side of the body at +X.
- Driver code (`src/driver`) runs inside `vrserver.exe`: every
  function is either unit-tested or a pure one-instruction forwarder, no exception may
  escape a hook or provider entry point, and unit tests must not touch the filesystem,
  wall clock, environment or real threads — dependencies are injected as seams
  (`HookApi`, `DeviceProperties`, `ServerDriverHost`, `LogSink`, clock fn). Files compiled only into the DLL
  (`Driver.cpp`) hold no branch, loop, comparison or arithmetic.
- Error handling: throw `Error` (never from constructors — use `loadConfig`-style
  functions when validation must throw). Catch blocks live at the app boundary
  (`main.cpp`) and just log `e.what()`. JSON shape errors are left to nlohmann
  (declarative `from_json`, no manual `is_array`/`size` checks). Wrap third-party calls
  at the call site so the log says which call failed. No `catch (...)` outside the
  driver boundary.
- Keep log strings ASCII (MSVC is not given `/utf-8`).
- Indentation: 4 spaces in `src/model`, `src/view`, `src/driver`, `tests`; tabs in
  `src/main.cpp`. Match the file you are editing.
- Edit sources with the edit/write tools only, never shell text munging (regex escapes
  and PowerShell re-encoding have corrupted files here before).
- Update this file when structure, targets, build steps or libraries change.
