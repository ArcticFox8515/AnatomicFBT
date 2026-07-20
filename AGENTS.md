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
- Running the exe creates `user-skeleton.json` / `user-ikrig.json` /
  `user-avatar-skeleton.json` in the working directory on first start (load-or-create
  pattern); invalid files fall back to defaults with the full error logged to
  `logs/trackingcorrector.log` (rotating, 3×5MB).

## Directory / file map

```
CMakeLists.txt        3 targets: TrackingCorrectorLib (static), TrackingCorrector (exe),
                      TrackingCorrectorTests (gtest). C++20.
conanfile.py          Dependencies + copies ImGui backends into src/bindings on generate.
doc/plan.md           4-milestone design doc. Read before architectural changes.
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
                      are included).
src/bindings/         Vendored ImGui backends (imgui_impl_glfw/opengl3). AUTO-COPIED by
                      conanfile.py from the imgui package — gitignored, never edit.
tests/                GoogleTest suites mirroring src/model.
```

## Classes & responsibilities

### Model layer (`src/model/`) — no GL dependencies

- `Error` (`Error.h`) — exception type for all errors thrown by our code; `what()`
  includes the throw site (file:line) via `std::source_location`. Derives from
  `std::runtime_error`.
- `GlmJson.h` — nlohmann `to_json`/`from_json` for `glm::vec3`, defined in
  `namespace glm` so ADL finds them; shared by all config (de)serialization.
- `Skeleton` (`Skeleton.h/.cpp`) — flat `std::vector<Joint>`, kept sorted
  parent-before-child. `Joint` = `{name, parentIndex (optional<int>), restOffset (vec3,
  meters), localRot (quat, identity at rest, NOT serialized)}`. The skeleton also has a
  runtime-only `rootPosition` (NOT serialized): the root joint sits at `rootPosition`
  with orientation `localRot`; the root's `restOffset` is not applied by FK, it only
  seeds `rootPosition` at load time. `localRot(i)` always means "rotation of the bone
  ending at joint i, relative to the parent joint's world orientation".
  - `Skeleton::makeDefault()` — 22-joint SlimeVR-style **head-rooted** skeleton (head is
    root, spine goes downward; left side at +X). Bone names are snake_case:
    `head, neck, upper_chest, chest, waist, hip, left/right_{hip,upper_leg,lower_leg,
    foot,shoulder,upper_arm,lower_arm,hand}`.
  - `Skeleton::makeDefaultHipRooted()` — same skeleton re-rooted at `hip` with the
    spine chain reversed (`hip → waist → … → head`), like VRChat/Unity avatars; rest
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
  two-bone chains). `makeDefault()`: anchor on head, chain on hip, two-bone on
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
  `head` = the fixed HMD) lands exactly on its src world position.
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
    the skeleton to rest, aligns the root to the HMD (position + `yawOnly`
    heading), proximity-matches devices to targets, mirrors matched raw device
    poses into the targets. Returns `CalibrationFrame{assignment (device-list
    positions, for UI), boneWorldPoses (for calibrate())}`.
  - `captureOffsets(calibration, frame, devices)` — freezes a frame into offsets:
    translates the assignment's list positions to stable device ids and calls
    `calibrate()`.
  - `updateCaptureFrame(rig, calibration, devices)` — one capture-mode frame:
    mirrors raw device poses into the targets and returns solver goals (copy +
    `applyOffsets`). Goals always parallel the rig's targets.
- `ModeController` (`ModeController.h/.cpp`) — the ManualPose/Calibration/Capture
  state machine, hardware-free (fed `TrackedDevice` snapshots + the trigger
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
  Calibration. `Mode` enum lives here too.

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

### VR layer (`src/vr/`) — exe-only, only place OpenVR headers are included

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
(model layer) rests the skeleton with root aligned to the HMD and mirrors proximity-
matched raw device poses into the targets (`liveAssignment` kept for UI feedback);
both-triggers edge freezes the offsets via `captureOffsets` and switches to Capture.
**Capture** — `updateCaptureFrame` (model layer) writes raw device poses into the targets (so
rendered markers show exactly what OpenVR reports, same as Calibration) and returns solver goals
(raw poses + offsets on a copy), `rig.solve(plan.goals)` consumes
them. Targets are never rewritten for display purposes. All of the above is driven
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
| openvr | 1.16.8 | SteamVR device poses + trigger input; linked to the exe only, headers only in `src/vr/` |
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
- Indentation: 4 spaces in `src/model`, `src/view`, `tests`; tabs in `src/main.cpp`.
  Match the file you're editing.
- `src/bindings/` and `CMakeUserPresets.json` are generated — never edit by hand.
- Tests: add new suites to the `TrackingCorrectorTests` target in `CMakeLists.txt`;
  run with `run-tests.bat`.
- When changing anything described in this file (structure, classes, build steps,
  libraries), update this AGENTS.md accordingly.
