# AGENTS.md — TrackingCorrector

## What this project is

Windows desktop app (C++20) for VR full-body tracking correction. It loads a humanoid
skeleton from JSON, renders it in 3D with OpenGL, and lets the user drag IK target
markers (head/hands/feet/hip) with gizmos. Long-term goal solve IK
so the skeleton follows the targets, retarget onto a Unity avatar skeleton, and emit
virtual SteamVR trackers.

## Build & test

Requires Conan 2 and Visual Studio 2022. Windows-only (`WIN32` exe, `WinMain` entry).

- `generate-project.bat` — runs `conan install` (Debug + Release) into `build/`, then
  configures CMake with the Conan toolchain. Run this first / after dependency changes.
- Build: `cmake --build build --config Debug` (or open `build/TrackingCorrector.sln`).
- `run-tests.bat` — `ctest --test-dir build -C Debug --output-on-failure`.
- `CMakeUserPresets.json` only includes the Conan-generated presets file and is
  gitignored — regenerate via the .bat, don't hand-edit.
- Running the exe creates `user-proportions.json` / `user-ikrig.json` /
  `user-avatar-skeleton.json` in the working directory on first start (load-or-create
  pattern); invalid files fall back to defaults with the full error logged to
  `logs/trackingcorrector.log` (rotating, 3×5MB). Every capture session is also
  recorded to `recording.tcrec` in the working directory (overwritten per session —
  copy/rename to keep).

### SteamVR driver (spike, see `doc/driver-plan.md` step 1)

- `build-driver.bat` — builds `driver_00trackingcorrector` + `spike_client`, **Release**
  (the hot pose path is in it, and Release keeps the DLL dependency-free: only
  `KERNEL32.dll`, since the Conan toolchain links the CRT statically).
- Only the **Release** DLL is staged to
  `build/driver/00trackingcorrector/bin/win64/` (the path SteamVR loads); other
  configs keep the default output dir, so a Debug build of the whole solution cannot
  silently replace the driver SteamVR is using. `SpikeDriverTest` loads the DLL of
  its own config (`$<TARGET_FILE:driver_00trackingcorrector>`).
- `install-driver.bat` / `uninstall-driver.bat` — `vrpathreg adddriver|removedriver` on
  `build/driver/00trackingcorrector`, with SteamVR located via
  `%LOCALAPPDATA%\openvr\openvrpaths.vrpath`. Nothing is copied: SteamVR loads the
  driver straight out of the build tree. Restart SteamVR after (un)installing.
- **`vrserver.exe` holds the DLL open — close SteamVR before rebuilding the driver.**
- Spike logs: `%LOCALAPPDATA%\TrackingCorrector\driver-spike-<process>.log`
  (one file per loading process: `vrserver`, `vrwatchdog`, and
  `TrackingCorrectorTests` when the tests run it) and, for the driver
  process, the same lines in SteamVR's `vrserver.txt` via `IVRDriverLog`.
  `spike_client [seconds]` writes `client-spike-spike_client.log` next to it and to
  stdout.
- **Testing bar for driver code** (`doc/driver-spike-handover.md` §2, non-negotiable):
  every function under `src/spike` either runs in a **unit** test or is a pure
  forwarder. `SpikeDriverTest` does not count — it `LoadLibrary`s the DLL, which makes
  it an integration test. The only forwarder files are `SpikeDriver.cpp` and
  `SpikeClient.cpp`; everything else lives in `SpikeLib` with its
  openvr/MinHook/Win32/clock dependencies injected as seams. Spike unit tests must not
  touch the filesystem, the wall clock or the environment — a test whose result depends
  on the machine proves nothing about code that runs inside `vrserver.exe`.
- **`SpikeDriver.cpp` is compiled into the DLL only, never into a test target**, so no
  unit test and no coverage run can reach a single line of it. Therefore every function
  in it is **one instruction**; two only when the first creates the implementation
  object and the second calls a method on it (the leaked-singleton accessors, the
  provider `Init`s). No branch, no loop, no comparison, no arithmetic — those belong in
  `SpikeLib` with a test (`doc/driver-spike-handover.md` §2.1b).
- After touching the hooks, run `ctest -C Release` as well as `run-tests.bat` (Debug):
  `SpikeDriverTest` drives the DLL of its own config, and only the Release build
  devirtualizes the fake vrserver's calls — the reason its call sites go through
  `throughVtable()` (`doc/driver-spike-handover.md` §10).

## Directory / file map

```
CMakeLists.txt        6 targets: TrackingCorrectorLib (static), TrackingCorrector (exe),
                      TrackingCorrectorTests (gtest), SpikeLib (static, all spike
                      logic), driver_00trackingcorrector (spike driver DLL),
                      spike_client (console tool). C++20.
conanfile.py          Dependencies + copies ImGui backends into src/bindings on generate.
doc/plan.md           4-milestone design doc. Read before architectural changes.
doc/driver-plan.md    Phase A design doc for the SteamVR driver (milestone 4): wire
                      protocol, threading contract, 8 implementation steps. Read before
                      touching anything driver-related.
doc/driver-spike-handover.md
                      State of the step-1 spike: the testing/concurrency bar for driver
                      code, what is and is not established, the line-coverage backlog,
                      the started (incomplete) race analysis, and the pending live-run
                      procedure. Read before continuing the spike.
doc/spacecalibrator-notes.md
                      Findings from OpenVR-SpaceCalibrator's driver (hooking, context
                      setup, pose composition) — the reference the plan is built on.
src/main.cpp          Entry point: GLFW/GL/ImGui init, config load-or-create, OpenVR init
                      (falls back to manual mode), main loop, split-screen rendering
                      (IK skeleton left, retargeted avatar right), ImGuizmo
                      manipulation of IK targets, ImGui control panel. The
                      mode state machine and all calibration/capture logic live in
                      the model layer; main.cpp only wires VR input in and
                      executes the returned FramePlan.
src/model/            Pure data + math + JSON serialization. NO OpenGL/OpenVR —
                      unit-testable.
src/view/             All OpenGL rendering (Scene).
src/vr/               OpenVR device tracking (exe-only; converts OpenVR state into
                      model `TrackedDevice` snapshots — the only place OpenVR headers
                      are included in the exe).
src/spike/            THROWAWAY step-1 spike of `doc/driver-plan.md`. Split so that
                      every function either runs in a unit test or is a pure
                      forwarder: all logic lives in the `SpikeLib` static library
                      (linked by the DLL, the client and the tests), and the two
                      files outside it are adapters exempt from the coverage bar
                      because no test target compiles them.
                      `SpikeDriver.cpp` = THE DRIVER ADAPTER, one instruction per
                      function (two only to create-then-call): `MinHookApi`,
                      `Win32ModuleApi`, `nowSeconds`, the six detour entry points,
                      `OpenVrServerEnvironment`, `OpenVrWatchdogEnvironment`, the two
                      providers, the leaked singletons, `HmdDriverFactory`. It hooks
                      `GetGenericInterface`, `TrackedDeviceAdded`,
                      `TrackedDevicePoseUpdated`, `Create/UpdateBooleanComponent`,
                      `CreateScalarComponent`; observes and logs only, forwards every
                      call unchanged.
                      `SpikeClient.cpp` = the client adapter (`IVRSystem` →
                      `ClientPoseSource`) + `main`. In SpikeLib:
                      `SpikeObserver.*` (all observation/bookkeeping; deps injected
                      as `Logger&` / `InterfaceHooks&` / `DeviceProperties*` /
                      clock fn), `SpikeServer.*` (provider lifecycle behind
                      `ServerEnvironment`/`WatchdogEnvironment`, factory
                      classification + `serveFactoryRequest` =
                      `HmdDriverFactory`'s body), `SpikeHooks.*` (`VTableHookBase`
                      state machine over the `HookApi` seam — MinHook-free — plus
                      `initializeHookLibrary`, the `MH_Initialize` status decision),
                      `SpikeDriverHooks.*` (the vtable index table and log labels as
                      named constants, `DriverHookSet` with the install order and the
                      reverse-order `removeAll`, and the six `observe*` functions that
                      ARE the detour bodies), `SpikeDriverEnvironment.*`
                      (`OpenVrProperties` over a `vr::VRProperties()` fn-pointer seam,
                      `driverLogSink`, `ModuleApi` + `modulePathOfAddress`),
                      `SpikeLog.*` (pure path/line formatting + injectable
                      `Logger`), `SpikeNames.*` (enum labels, openvr-free header),
                      `SpikeInterfaces.h` (interface-version decision),
                      `SpikePoseMath.h` (openvr-free `DriverPose_t` composition +
                      matrix→pose conversion), `SpikeGuard.h` (`runGuarded`, the one
                      place an exception is swallowed), `SpikeClientReport.*`
                      (client sampling loop + line formatting).
                      Deleted at step 7 when the real `src/driver*` code lands.
src/driverdll/00trackingcorrector/driver.vrdrivermanifest
                      SteamVR driver manifest (`alwaysActivate`, name
                      `00trackingcorrector`). Directory name, manifest `name` and DLL
                      name must all agree; copied next to `bin/win64/` by a post-build
                      step. The `00` prefix makes SteamVR load us before other drivers.
src/bindings/         Vendored ImGui backends (imgui_impl_glfw/opengl3). AUTO-COPIED by
                      conanfile.py from the imgui package — gitignored, never edit.
tests/                GoogleTest suites mirroring src/model, plus the spike suites.
                      `Spike{Observer,Server,Hooks,DriverHooks,DriverEnvironment,Log,ClientReport}Test.cpp`
                      drive SpikeLib directly against fakes — no DLL, no MinHook, no
                      vrserver, and (deliberately) no filesystem, clock or
                      environment: a driver test whose result depends on the machine
                      proves nothing, so streams are `ostringstream`s, time is an
                      injected `double`, and MinHook is a fake `HookApi` whose
                      failure statuses are set per test. `SpikeDriverTest.cpp` is the
                      DLL-boundary integration proof on top: it loads the spike
                      driver DLL and drives it through a fake vrserver (fake
                      `IVRDriverContext`/`IVRServerDriverHost`/`IVRProperties`/
                      `IVRDriverInput`/`IVRDriverLog`), asserting hook installation on
                      the plan's vtable indices, unchanged forwarding, metadata reads,
                      component resolution, trigger edges and the composition wiring —
                      everything about the spike that does not need SteamVR.
unity/                Standalone Unity Editor tooling (C#), copied into a project's
                      Editor/ folder — NOT part of the C++ build. `AvatarSkeletonExporter.cs`
                      copies a humanoid avatar's skeleton to our skeleton JSON with no
                      interpretation (no per-bone logic): one joint per humanoid-mapped
                      transform, named by the `HumanBodyBones` enum member verbatim
                      (`Hips`, `LeftUpperLeg`, …); parent = nearest mapped ancestor.
                      The one representation gap it bridges: Unity puts a bone's
                      transform at the START of the bone, our format puts a joint at the
                      END, so each joint is placed at the bone end, computed generically
                      — a bone with mapped children sits at the average of the children's
                      positions; a leaf sits at a child transform named `<name>_end`
                      (Blender FBX leaf-bone convention) if present, else at the average
                      of all its child transforms' positions, else default length
                      (0.1×humanScale) along the bone direction; the root (`Hips`) at its
                      own transform. World positions taken (bakes in-scene scale) →
                      Animator-root frame (translation+rotation removed, scale kept) →
                      our axes (x,y,z)→(-x,y,-z); offsets are child−parent. Root special
                      case: Unity's single `Hips` bone (root carries no bone in our
                      format) explodes into one joint per child at the child's own start
                      — `Spine`→`Waist`, `LeftUpperLeg`→`LeftHip`, `RightUpperLeg`→`RightHip`
                      (names avoiding `HumanBodyBones` members) — and those children
                      reparent onto them so their bones pivot correctly.
```

## Classes & responsibilities

### Model layer (`src/model/`) — no GL dependencies

- `Error` (`Error.h`) — exception type for all errors thrown by our code; `what()`
  includes the throw site (file:line) via `std::source_location`. Derives from
  `std::runtime_error`.
- `GlmJson.h` — nlohmann `to_json`/`from_json` for `glm::vec3`, defined in
  `namespace glm` so ADL finds them; shared by all config (de)serialization.
- `BoneNames` (`BoneNames.h`) — `inline constexpr const char*` for every default
  skeleton bone name (`BoneNames::Head`, `BoneNames::LeftHand`, …), shared by
  `Skeleton::makeDefault()` and `IkRigConfig::makeDefault()`. The names follow
  Unity's `HumanBodyBones` naming — exactly what `unity/AvatarSkeletonExporter.cs`
  emits — so an exported avatar skeleton retargets by name with no translation.
- `BodyProportions` (`BodyProportions.h/.cpp`) — user body measurements
  (`user-proportions.json`): 9 tape-measurable floats (meters, symmetric L/R):
  `neckLength`, `shoulderHeight`, `navelHeight`, `shoulderWidth`, `hipWidth`,
  `upperArmLength`, `lowerArmLength`, `upperLegLength`, `lowerLegLength`. Pure
  value type — JSON + `validate()` (positive lengths; `shoulderHeight >
  navelHeight > hip line`), NO skeleton knowledge (one-way dependency:
  Skeleton → BodyProportions). `shoulderHeight`/`navelHeight` are heights above
  the floor of the shoulder line (arm attachment) and the navel (lower-spine
  bend); `neckLength` spans the skull base (Neck) down to the shoulder line
  (Chest). The IK skeleton is never serialized — only proportions
  are; the hierarchy is fixed and rebuilt at runtime.
- `Skeleton` (`Skeleton.h/.cpp`) — flat `std::vector<Joint>`, kept sorted
  parent-before-child. `Joint` = `{name, parentIndex (optional<int>), restOffset (vec3,
  meters), localRot (quat, identity at rest, NOT serialized)}`. The skeleton also has a
  runtime-only `rootPosition` (NOT serialized): the root joint sits at `rootPosition`
  with orientation `localRot`; the root's `restOffset` is not applied by FK, it only
  seeds `rootPosition` at load time. `localRot(i)` always means "rotation of the bone
  ending at joint i, relative to the parent joint's world orientation".
  - `Skeleton::makeDefault(proportions = {})` — 22-joint SlimeVR-style **head-rooted**
    skeleton (Head is root, spine goes downward; left side at +X) scaled to
    `BodyProportions`. Bone lengths equal the proportions; landmark heights hold
    relative to the skeleton's own ankles (Chest at `shoulderHeight`, Waist at
    `navelHeight`, Hips at `upperLeg + lowerLeg` above the ankles — the skeleton
    has no floor frame). Root rest Y = `shoulderHeight + neckLength` only seeds
    `rootPosition` (calibration aligns the root to the HMD at runtime). Bone names
    come from `BoneNames.h`:
    the spine chain runs `Head → Neck → Chest → Spine → Waist → Hips` (our
    shoulder-parent "upper chest" is Unity's `Chest`, our mid-spine "chest" is Unity's
    `Spine`), plus `Left/Right{Hip,UpperLeg,LowerLeg,Foot,Shoulder,UpperArm,LowerArm,Hand}`.
    Head-height (0.15, Head→Neck) and hand/foot bone lengths (0.12/0.08) are internal
    constants — hand/foot lengths provably cancel out of the capture-mode IK
    (calibration offset gains exactly the term `solveTwoBone` subtracts back); the
    fictional mid-spine joint splits its span at the midpoint.
  - `Skeleton::makeDefaultHipRooted()` — same skeleton re-rooted at `Hips` with the
    spine chain reversed (`Hips → Waist → … → Head`), like VRChat/Unity avatars; rest
    world positions are identical. Default for `user-avatar-skeleton.json`. Built by
    an internal `reroot()` helper (reverses the ancestor chain, negating offsets —
    valid only while rest rotations are identity).
  - Free `to_json`/`from_json` — schema `{ "bones": [{name, parent|null, offset:[x,y,z]}] }`.
    `from_json` parses declaratively (nlohmann), then assembles: resolves parent names
    to indices, sorts parent-before-child, seeds `rootPosition` — throws `Error` on
    duplicate names, unknown parents, cycles, or not exactly 1 root.
  - `computeWorldTransforms(skeleton)` — hierarchical FK, one linear pass:
    `worldRot = parentWorldRot * localRot`, `pos = parentPos + worldRot * restOffset`.
    Returns positions + world rotations (`WorldTransforms`).
    `computeWorldPositions(skeleton)` is a positions-only wrapper.
- `IkMath` (`IkMath.h/.cpp`) — pure math helpers for the solver: `solveTwoBoneIk`
  (closed-form two-bone IK with pole vector; overreach stretches straight toward the
  target) and `clampSwingTwist` (swing-twist decomposition + clamp), plus
  `quatFromTo`/`anyPerpendicular` utilities. No skeleton knowledge — unit-testable.
- `IkRigConfig` (`IkRigConfig.h/.cpp`) — which bones accept tracker targets and how
  each is solved (`targets`: `TargetConfig{bone, solver}`, `SolverType` =
  `anchor|chain|two_bone`) + per-bone `JointLimits`
  (`twistMinDeg/twistMaxDeg/swingConeDeg` + optional `pole` vec3 — the bend direction
  of that bone's hinge in the limb socket's frame, required on the middle bone of
  two-bone chains). `makeDefault()`: anchor on Head, chain on Hips, two-bone on
  hands/feet; hinge limits + poles for knees/elbows (knees forward −Z, elbows
  down/back), cones for hips/shoulders. `from_json` parses declaratively (via
  per-struct `from_json` for `TargetConfig`/`JointLimits`), then calls the
  `validate()` member (duplicate bones, pole non-zero, twist/swing ranges; throws
  `Error`). Kept separate from `Skeleton` because the milestone-3 avatar skeleton
  has no IK.
- `IkSolvers` (`IkSolvers.h/.cpp`) — `IkTarget` = `{jointIndex, position, rotation}`
  (a world-space manipulation handle) + the three per-target solver stages, each
  unit-testable in isolation:
  - `solveAnchor` — rigidly pins the root joint (`rootPosition` + root `localRot`).
  - `solveChain` — spine-style chain IK: the end bone rigidly takes the target
    rotation and the remaining segments arc-swing/curl so the end bone's base lands
    on the implied goal (`target.position − target.rotation * restOffset_end`).
    Segment orientations = minimal swing from the root frame onto the solved
    directions + leftover twist about the chain distributed by length (head yaw
    rolls down the spine gradually). End position AND rotation are exact within
    reach; on overreach the chain stretches straight and the end rotation still wins.
  - `solveTwoBone` — two-bone analytic limb IK (socket→j1→j2→tip): places j2 on the
    goal implied by the tip target, tip bone takes the target rotation; pole in the
    socket's frame. `solveTwoBoneIk` returns rest-relative world rotations, so they
    compose on top of the socket frame (`worldRot = result.rot * socketRot`) — do
    not use them as bone world rotations directly (breaks under rotated sockets).
- `IkRig` (`IkRig.h/.cpp`) — owns `Skeleton` + `IkRigConfig` + `std::vector<IkTarget>`.
  No bone names in code: solver structure is derived from config + skeleton topology
  (`SolverBinding` per target). `IkRig(Skeleton)` never throws — the rig starts
  config-less (no targets, `solve()` idles at rest pose). `loadConfig(config)`
  validates (throws `Error`): target/limit bones exist; anchor target is the root
  joint; chain target is not the root (chain = ancestor path root→joint); two_bone
  target has ≥3 ancestors (tip→j2→j1→socket walk) and its middle bone (j2) carries
  a `pole` in the limits. On failure the previous config/targets stay active.
  On success: places targets at the rest pose. `resetTargets()`, `targetName(i)`
  for UI labels.
  - `IkRig::solve()` — re-derives the full pose from current targets every call
    (stateless, call each frame). `solve(goals)` overload consumes an explicit
    goal vector instead of the stored targets (throws `Error` on size mismatch) —
    capture mode solves from offset-corrected copies while `targets` keep the raw
    device poses for rendering. Stages by solver type, regardless of config order:
    all anchors → all chains → all two-bone limbs → joint-limits post-pass
    (`clampSwingTwist` on each `JointLimits` bone's `localRot`).
- `Retarget` (`Retarget.h/.cpp`) — pose transfer from the IK skeleton onto the avatar
  skeleton. `buildRetargetMap(src, dst)` matches bones once by connectivity: a dst
  bone `parent→child` matches the src bone connecting the same two joint names in
  EITHER direction (unordered name pair), so a head-rooted src drives a hip-rooted
  dst exactly despite the spine chain running opposite ways; the dst root joint
  matches by plain name. `retargetPose(src, dst, map)` copies the matched src joint's
  world rotation onto each dst joint (`localRot = inverse(parentWorld) * world`,
  parent-before-child), leaves unmatched dst joints at rest, then shifts
  `dst.rootPosition` so the anchor joint (dst joint named like the src root, i.e.
  `Head` = the fixed HMD) lands exactly on its src world position.
  `unmatchedBones(dst, map)` lists dst joints with no match (logged once at startup).
- `Pose` (`Pose.h`) — header-only `Pose{position, rotation}` + `compose`/`inverse`
  rigid-pose math + `yawOnly(quat)` (heading without pitch/roll; treats -Z as the
  rotation's forward, OpenVR-style). Shared by the VR provider and calibration.
- `TrackedDevice` (`TrackedDevice.h/.cpp`) — hardware-agnostic tracked-device
  snapshot: `TrackedDeviceKind` (`hmd|controller|tracker|other`-style enum) +
  `TrackedDevice{id, kind, pose}` where `id` is an opaque stable id chosen by the
  tracking provider (for OpenVR: the tracked device index). Helpers: `findHmd`,
  `deviceKindName` (UI label), `devicePosePairs` (packs snapshots into the
  `(id, pose)` pair list `TrackerCalibration` binds against).
- `TrackerCalibration` (`TrackerCalibration.h/.cpp`) — device→target binding for
  capture mode, OpenVR-free (devices arrive as `(int id, Pose)` pairs).
  `assignDevicesToTargets(devicePositions, targetPositions)` — greedy nearest-pair
  matching (user T-poses, so each tracker is closest to its own target).
  `calibrate(assignment, devices, boneWorldPoses)` stores per-target
  `offset = inverse(devicePose) * boneWorldPose`. `applyDevicePoses(devices,
  rig.targets)` writes the RAW device pose into each bound target (what gets
  rendered; missing devices leave the target untouched); `applyOffsets(goals)`
  transforms targets into solver goals (`goal = target * offset`, on a copy).
  `clear()` on (re-)entering calibration.
  - `updateCalibrationFrame(rig, devices)` — one calibration-mode frame: resets
    the skeleton to rest, places the root from the HMD, proximity-matches
    devices to targets, mirrors matched raw device poses into the targets.
    Returns `CalibrationFrame{assignment (device-list positions, for UI),
    boneWorldPoses (for calibrate())}`.
    Root placement: the HMD sits an arbitrary offset away from the Head
    joint, so the root is NOT pinned to the HMD position. In T-pose the
    midpoint of the two Controller-kind devices (the hands — the calibration
    gesture itself requires two controllers) coincides with the Chest joint
    (end of the neck bone) — an exact invariant of the rest skeleton — so
    whenever both controllers are tracked, the root is shifted so the FK
    Chest lands on the measured midpoint; the correction is masked in the
    HMD-yaw frame, keeping height and forward from the hands and taking
    lateral from the HMD (assumed centered on the head). Without an HMD or
    both controllers it falls back to plain HMD alignment, and with no HMD
    at all the root is left as-is. The HMD's offset from the Head joint ends
    up in the head target's Binding like every other target.
  - `captureOffsets(calibration, frame, devices)` — freezes a frame into offsets:
    translates the assignment's list positions to stable device ids and calls
    `calibrate()`.
  - `updateCaptureFrame(rig, calibration, devices)` — one capture-mode frame:
    mirrors raw device poses into the targets and returns solver goals (copy +
    `applyOffsets`). Goals always parallel the rig's targets.
- `ModeController` (`ModeController.h/.cpp`) — the ManualPose/Calibration/Capture/
  Replay state machine, hardware-free (fed `TrackedDevice` snapshots + the trigger
  gesture per frame). Owns the `TrackerCalibration` and the live assignment
  (for UI). `update(rig, devices, captureGesture)` mutates the rig per the mode
  and returns a `FramePlan{SolveMode solve (None|Targets|Goals), goals,
  capturedOffsets}`; the render loop just executes it (`rig.solve()` /
  `rig.solve(plan.goals)` / nothing). Key invariant: Capture frames — including
  the calibration→capture transition — always produce goals freshly derived
  from the current targets on that same frame, so `solve(goals)` can never get
  a stale or wrongly-sized vector (regression: this crashed on the transition
  frame when goal production lived in main.cpp's if/else-if chain).
  `switchToCalibration()` clears the offsets; the gesture is ignored outside
  Calibration. Replay shares the Capture branch of `update` verbatim — the
  solver cannot tell recorded devices from live ones; `switchToReplay()` clears
  the offsets and `calibrateFromFrame(rig, devices)` re-runs the exact live
  calibration path (rest + HMD/hand-landmark root placement, proximity
  assignment, `captureOffsets`) on a
  recording's first frame, reproducing the live session's offsets bit for bit.
  `Mode` enum lives here too.
- `Recording` (`Recording.h/.cpp`) — binary capture-session recording of the
  device snapshots fed to `ModeController::update` (inputs only, so different
  skeletons/solvers/calibrations can be tested against the same session).
  Format (`.tcrec`, little-endian, documented in the header): magic+version,
  roster (device ids + kinds, frozen on the first frame), then fixed-size
  frames of absolute time (f32 seconds) + one pose per roster device in roster
  order. Frame 0 is the exact calibration input. A device absent from a frame
  is written with its last known pose (the live dropout effect — the target
  doesn't move), so every frame is self-contained and randomly seekable;
  mid-session devices unknown to the roster are ignored. `RecordingWriter`
  streams to a `std::ostream` (nothing buffered — a crash costs at most the
  trailing frame); `loadRecording(istream)` throws `Error` on malformed input
  or zero complete frames but silently drops a truncated trailing frame;
  `RecordingFrame.devices` is ready to feed to `update`. `nearestFrameIndex`
  snaps a timeline click to the closest frame time. `kRecordingFileExtension`
  (`.tcrec`) lives here too.
- `SessionRecorder` (`SessionRecorder.h/.cpp`) — automatic capture-session
  recording lifecycle, log- and file-system-free: streams come from an injected
  `StreamFactory` (tests use string streams; main.cpp supplies "open
  `recording.tcrec` truncated"). `update(mode, capturedOffsets, now, devices)`
  is called once per frame right after `ModeController::update`: the
  calibration→capture transition starts a new recording whose frame 0 holds
  exactly the devices calibration froze offsets from; every further Capture
  frame appends at `now - start`; leaving Capture stops. Failures never throw
  (a broken recording must not break the capture) — they stop the recording
  and surface once in the returned `Event{started, stopped, error}`, which the
  caller logs.
- `ReplaySession` (`ReplaySession.h/.cpp`) — replay-mode state, UI- and
  hardware-free: the recording file list (`scan(directory)` — sorted
  `*.tcrec`), the loaded recording, and the timeline position.
  `load(index, controller, rig)` opens the file and recalibrates via
  `ModeController::calibrateFromFrame` on frame 0, then seeks to frame 0;
  throws `Error` on failure, leaving the file list intact and no recording
  loaded. `seek(time)` snaps to `nearestFrameIndex`; `currentDevices()` is the
  per-frame `update` input (empty without a recording — skeleton idles);
  `frameTime()/frameIndex()/files()/loadedIndex()/hasRecording()` feed the UI.

### View layer (`src/view/`)

- `Scene` (`Scene.h/.cpp`) — owns ALL GL resources (RAII: created after GLEW init,
  destroyed before GLFW shutdown — keep the scope in `main.cpp` intact):
  - Orbit camera: target (0,1,0) by default, distance 3.5, RMB-drag changes yaw/pitch
    (`update(window, allowInput)` — pass `allowInput=false` when ImGui/gizmo captures
    the mouse). `setCameraTarget` moves the orbit center — VR modes call it per frame
    with the HMD's XZ position so the skeleton stays in view. `setCameraYaw` sets the
    orbit yaw directly — Calibration mode calls it per frame with the HMD heading so
    the camera always looks at the skeleton from the front (RMB yaw-drag is
    overridden there). ONE camera shared by both viewports, so the halves always match.
  - `beginFrame(w,h)` — clears the full framebuffer + depth test only.
    `setViewport(x,y,w,h)` — viewport + camera matrices for its aspect + ground grid;
    call once per viewport (left half: IK skeleton, right half: avatar).
  - `renderSkeleton()` — one stretched gray pyramid per bone (base at parent joint,
    apex at joint). `renderTargets()` — small orange octahedron per IK target.
    Both are rendered unconditionally every frame; the only per-mode render
    difference is the gizmos (ManualPose only).
  - Two shader programs compiled from embedded GLSL 330 strings: flat-color line
    program (grid) and diffuse-lit mesh program (pyramids/octahedra, one directional
    light). Static unit meshes in VBOs, per-instance `uModel` matrix.
  - `viewMatrix()/projectionMatrix()` — from the last `setViewport`; needed by
    ImGuizmo (call right after the left viewport).

### VR layer (`src/vr/`) — exe-only, only place OpenVR headers are included in the exe

- `OpenVrTracking` (`OpenVrTracking.h/.cpp`) — tracked-device provider. Header is
  OpenVR-free: snapshots come out as model `TrackedDevice` values (OpenVR's standing
  universe is RH/Y-up/meters like ours, so `HmdMatrix34` transposes straight into
  glm; the device index doubles as the snapshot's stable `id`). Non-throwing default
  constructor; `init()` (`VRApplication_Background` — never launches SteamVR, fails
  fast when it isn't running) throws `Error` and is retryable. `pollPoses()` —
  connected+valid HMD/controller/tracker poses. `bothTriggersJustPressed()` —
  edge-detected "second trigger goes down while the first is held" (calibration
  gesture). Destructor calls `VR_Shutdown`.

### Entry point (`src/main.cpp`)

`WinMain`: spdlog rotating-file logger (`logs/trackingcorrector.log`) → GLFW window
(OpenGL 3.3 core) → GLEW init → ImGui + ImGuizmo setup → load-or-create all three JSON
configs (one `loadOrCreate<T>` template taking a default-factory function pointer;
any exception logged with type + message + source site) → `IkRig` + `loadConfig`
(falls back to default config if invalid) → avatar `Skeleton` + `RetargetMap`
(unmatched avatar bones logged once) → `OpenVrTracking::init` (success → start in
Calibration mode; failure → log + Manual Pose mode).

Modes (`enum class Mode`, model layer): **ManualPose** — gizmo-dragged targets, `rig.solve()` each
frame, no VR input. **Calibration** — no IK; per frame `updateCalibrationFrame`
(model layer) rests the skeleton with the root placed from the HMD via the
T-pose hand-midpoint landmark (see `TrackerCalibration` above) and mirrors proximity-
matched raw device poses into the targets (`liveAssignment` kept for UI feedback);
both-triggers edge freezes the offsets via `captureOffsets` and switches to Capture.
**Capture** — `updateCaptureFrame` (model layer) writes raw device poses into the targets (so
rendered markers show exactly what OpenVR reports, same as Calibration) and returns solver goals
(raw poses + offsets on a copy), `rig.solve(plan.goals)` consumes
them. Targets are never rewritten for display purposes. Every capture session is
also recorded to `recording.tcrec` — the whole lifecycle lives in
`SessionRecorder` (model layer); main.cpp only supplies the file-stream factory,
passes `glfwGetTime()` as the clock, and logs the returned events. **Replay** —
devices come from a loaded recording's current frame instead of OpenVR (no
polling, no gesture); all state lives in `ReplaySession` (model layer):
entering the mode calls `scan(".")` and loads the first file (recalibrating
from its frame 0); with no files the skeleton idles at rest. The panel lists
`files()` (click = `load()`), and a borderless timeline window pinned across
the bottom holds a full-width slider bound to `frameTime()` that calls
`seek()` — no auto-play, `currentDevices()` is fed to `update` each frame.
All of the above is driven
by `ModeController::update` — main.cpp's only mode logic is: poll poses once per
frame, read the trigger edge in Calibration only (stateful), call `update`,
execute the returned `FramePlan`.

Per frame: `Scene::update/beginFrame`, `controller.update(...)` (also keeps
`lastDevices` for the UI and camera centering via the local device list), camera follows the
HMD's XZ position in VR modes (resets to origin in Manual) and — Calibration only —
yaws to the HMD heading so the skeleton is always seen from the front, then the left pass (`setViewport(0,0,w/2,h)`, `ImGuizmo::SetRect` confined to the
left half, `manipulateTargets()` + `rig.solve()` when the plan says Targets, `rig.solve(plan.goals)`
when it says Goals, `renderSkeleton/renderTargets` unconditionally), then `retargetPose()` and the right pass
(`setViewport(w/2,0,…)`, `renderSkeleton(avatar)`), then the ImGui panel (mode
switch buttons + SteamVR status, live device→target assignment in Calibration,
per-target position/rotation drag fields, reset button).

## Libraries (Conan, see `conanfile.py`)

| Library | Version | Used for |
|---|---|---|
| glfw | 3.4 | Window + input |
| glew | 2.2.0 | OpenGL function loading (`IMGUI_IMPL_OPENGL_LOADER_GLEW` defined) |
| glm | 1.0.1 | vec/mat/quat math everywhere |
| imgui | 1.90.5-**docking** | UI; `force=True` overrides imguizmo's pinned imgui — do not change version without reading conanfile comment |
| imguizmo | cci.20231114 | Translate/rotate gizmos for IK targets |
| nlohmann_json | 3.11.3 | Config (de)serialization via `to_json`/`from_json` free functions |
| spdlog | 1.15.3 | Error/info logging to rotating file; linked to the exe only — model layer stays log-free |
| openvr | 1.16.8 | SteamVR device poses + trigger input; linked to the exe only, headers only in `src/vr/`. The spike driver DLL uses `openvr_driver.h` **headers only** (no `openvr_api` link — vrserver supplies the implementations) |
| minhook | 1.3.4 | Vtable detours inside `vrserver.exe`; linked by the driver DLL only. `SpikeLib` stays MinHook-free behind the `HookApi` seam |
| gtest | 1.15.0 | Tests, wired via `gtest_discover_tests` |

## Conventions & gotchas

- **Separation**: model layer must stay GL-free (tests link only `TrackingCorrectorLib`
  and construct no GL context); view/app code depends on model, never the reverse.
- Coordinates: right-handed, **Y-up, meters**. Left side of the body at +X.
- Serialization: `localRot` and `Skeleton::rootPosition` are runtime-only; rest
  orientation is always identity for this skeleton (bone-roll problem is deferred to
  milestone 3 — see `doc/plan.md`).
- Error handling: our code throws `Error` (never from constructors — use separate
  functions like `IkRig::loadConfig` when validation must throw); its `what()`
  includes the throw site, so catch blocks just log `e.what()`. JSON shape errors
  are left to nlohmann — declarative `from_json` with `GlmJson.h` vec3 serializers,
  no manual `is_array`/`size` checks. Catch blocks live at the app boundary
  (main.cpp). Third-party calls are wrapped at the call site (rethrown as `Error`
  with the original message) so the log says which call threw. No `catch (...)`;
  what we can't report fully, we don't catch.
- Indentation: 4 spaces in `src/model`, `src/view`, `src/spike`, `tests`; tabs in `src/main.cpp`.
  Match the file you're editing.
- `src/bindings/` and `CMakeUserPresets.json` are generated — never edit by hand.
- Tests: add new suites to the `TrackingCorrectorTests` target in `CMakeLists.txt`;
  run with `run-tests.bat`.
- When changing anything described in this file (structure, classes, build steps,
  libraries), update this AGENTS.md accordingly.
