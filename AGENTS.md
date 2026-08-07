# AGENTS.md — TrackingCorrector

## What this project is

Windows desktop app (C++20) for VR full-body tracking correction. Loads a humanoid
skeleton from JSON, renders it with OpenGL, drives it with IK from tracked devices,
retargets the pose onto a Unity avatar skeleton. Long-term goal: correct tracking at
SteamVR driver level and emit virtual trackers. Design docs: `doc/plan.md` (milestones),
`doc/driver-plan.md` (SteamVR driver, milestone 4), `doc/spacecalibrator-notes.md`
(reference driver findings).

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
  `user-ikrig.json`, `user-avatar-skeleton.json` (load-or-create; invalid files fall
  back to defaults with the error logged), `logs/trackingcorrector.log` (rotating
  3×5MB), `recording.tcrec` (every capture session, overwritten per session).

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
- Driver logs: `%LOCALAPPDATA%\TrackingCorrector\driver-spike-<process>.log` (one per
  loading process: `vrserver`, `vrwatchdog`, `TrackingCorrectorTests`), plus the same
  lines in SteamVR's `vrserver.txt` via `IVRDriverLog`.
- Directory name, manifest `name` and DLL name must agree (`00trackingcorrector` /
  `driver_00trackingcorrector.dll`). The `00` prefix makes SteamVR load us first.

## Targets (`CMakeLists.txt`)

| Target | Kind | Contents / deps |
|---|---|---|
| `TrackingCorrectorLib` | STATIC | `src/model` + `src/view`; glm, json, GLEW, glfw |
| `LinkLib` | STATIC | `src/link` IPC seam + framing + protocol; std lib only |
| `TrackingCorrector` | WIN32 exe | `src/main.cpp`, `src/vr`, imgui/imguizmo, spdlog, openvr |
| `SpikeLib` | STATIC | all `src/spike` logic; openvr headers only |
| `driver_00trackingcorrector` | SHARED | `src/spike/SpikeDriver.cpp` + SpikeLib, minhook, spdlog |
| `spike_client` | exe | `src/spike/SpikeClient.cpp` + SpikeLib, openvr |
| `TrackingCorrectorTests` | exe | `tests/`, gtest; add new suites here |

## Directory map

```
src/model/      Pure data + math + JSON. NO OpenGL/OpenVR — unit-testable.
src/view/       All OpenGL rendering (Scene).
src/vr/         OpenVR device tracking (exe-only; the only place openvr.h is included).
src/link/       Driver<->app IPC: `Pipe` transport seam (1:1 to a Win32 named-pipe file
                handle: write/read/close, poll-based, never blocks), `MessageChannel`
                framing/reassembly over it, and the wire protocol's two messages as
                memcpy'd PODs. Standard library only — no model, no glm, no openvr, no
                spdlog — so the driver DLL links no model code. The real Win32 pipe is
                step 5 of doc/driver-plan.md; the mock pipe lives in tests/FakePipe.h.
src/spike/      THROWAWAY step-1 spike of doc/driver-plan.md: observation-only SteamVR
                driver that hooks GetGenericInterface / TrackedDeviceAdded /
                TrackedDevicePoseUpdated and logs. All logic in SpikeLib; SpikeDriver.cpp
                and SpikeClient.cpp are pure adapters. Input capture is NOT part of it
                (neither IVRDriverInput hooks nor PollNextEvent deliver buttons).
src/driverdll/00trackingcorrector/driver.vrdrivermanifest  — copied next to bin/win64/.
src/bindings/   Vendored ImGui backends, auto-copied by conanfile.py. Generated.
tests/          GoogleTest suites mirroring src/model, src/link and src/spike.
                `FakePipe.h` is the `link::Pipe` mock used by the link suites.
unity/          Standalone Unity Editor tooling (C#), not part of the C++ build.
```

## Model layer (`src/model/`)

- `Error` — exception type for all our errors; `what()` includes the throw site.
- `GlmJson.h` — `to_json`/`from_json` for `glm::vec3`, in `namespace glm` for ADL.
- `BoneNames.h` — every default bone name as `constexpr const char*`. Names follow
  Unity's `HumanBodyBones`, so an exported avatar retargets by name with no translation.
- `BodyProportions` — 9 tape-measurable floats (`user-proportions.json`), symmetric L/R,
  meters. No skeleton knowledge; `validate()` checks positive lengths and landmark order.
- `Skeleton` — flat `vector<Joint>` sorted parent-before-child. `Joint = {name,
  parentIndex, restOffset, localRot}`; `localRot(i)` = rotation of the bone *ending* at
  joint i, relative to the parent's world orientation. `localRot` and `rootPosition` are
  runtime-only (never serialized); rest orientation is always identity.
  `makeDefault(proportions)` = 22-joint head-rooted skeleton (spine downward);
  `makeDefaultHipRooted()` = same rest positions re-rooted at Hips (avatar default).
  `computeWorldTransforms` = hierarchical FK. JSON schema:
  `{"bones":[{name, parent|null, offset:[x,y,z]}]}`; `from_json` throws `Error` on
  duplicate names, unknown parents, cycles, or not exactly one root.
- `IkMath` — `solveTwoBoneIk` (closed-form, pole vector, stretches on overreach),
  `clampSwingTwist`, `quatFromTo`. No skeleton knowledge.
- `IkRigConfig` — which bones take targets and how (`anchor|chain|two_bone`) plus
  per-bone `JointLimits` (twist range, swing cone, optional `pole` — required on the
  middle bone of a two-bone chain). `validate()` throws `Error`.
- `IkSolvers` — `IkTarget{jointIndex, position, rotation}` + `solveAnchor` (pins the
  root), `solveChain` (spine: end bone takes the target rotation exactly, rest arcs),
  `solveTwoBone` (limb socket→j1→j2→tip; `solveTwoBoneIk` returns rest-relative
  rotations that compose onto the socket frame).
- `IkRig` — owns skeleton + config + targets; no bone names in code (structure derived
  from config + topology). Construction never throws; `loadConfig` validates and throws
  `Error`, keeping the previous config on failure. `solve()` re-derives the pose from
  the targets every call; `solve(goals)` consumes an explicit goal vector. Stage order:
  anchors → chains → two-bone limbs → joint-limit clamp.
- `Retarget` — `buildRetargetMap` matches dst bones to src bones by unordered joint-name
  pair (so head-rooted src drives hip-rooted dst); `retargetPose` copies world rotations
  and shifts `dst.rootPosition` so the anchor joint lands on its src position.
- `Pose` — header-only `{position, rotation}` + `compose`/`inverse`/`yawOnly`.
- `TrackedDevice` — hardware-agnostic snapshot: `{id, kind, pose}`, `kind` =
  `hmd|controller|tracker|other`; `id` is an opaque stable id from the provider.
- `TrackerCalibration` — device→target binding, OpenVR-free. Proximity assignment
  (user T-poses), `calibrate` stores `offset = inverse(devicePose) * boneWorldPose`,
  `applyDevicePoses` writes raw device poses into targets (what gets rendered),
  `applyOffsets` produces solver goals on a copy. `updateCalibrationFrame` rests the
  skeleton and places the root: in T-pose the midpoint of the two controllers coincides
  with the Chest joint (exact invariant of the rest skeleton), so the root is shifted to
  land FK Chest there, masked in the HMD-yaw frame (height/forward from the hands,
  lateral from the HMD); falls back to plain HMD alignment.
- `ModeController` — ManualPose/Calibration/Capture/Replay state machine, hardware-free.
  `update(rig, devices, captureGesture)` mutates the rig and returns
  `FramePlan{solve, goals, capturedOffsets}`. Invariant: Capture frames, including the
  calibration→capture transition, always produce goals derived from that same frame.
  Replay reuses the Capture branch verbatim; `calibrateFromFrame` re-runs the live
  calibration path on a recording's frame 0.
- `Recording` — binary `.tcrec` capture of the device snapshots fed to `update` (inputs
  only). Magic+version, roster frozen on frame 0, then fixed-size self-contained frames
  (absent device = last known pose). Streamed, randomly seekable; loader throws `Error`
  on malformed input but drops a truncated trailing frame.
- `SessionRecorder` — recording lifecycle over an injected `StreamFactory`; never throws,
  reports via `Event{started, stopped, error}`.
- `ReplaySession` — recording file list (`scan`), loaded recording, timeline position;
  `load` recalibrates from frame 0, `currentDevices()` feeds `update`.

## Link layer (`src/link/`)

- `Pipe` — transport seam, one method per Win32 named-pipe file-handle op
  (`write`/`read`/`close`, poll-based, never blocks; `IoStatus` =
  `Ok|Pending|Closed|Failed`). An already-connected byte stream — endpoint
  construction (`CreateNamedPipe`/`ConnectNamedPipe` server, `WaitNamedPipe`/
   `CreateFile` client) is out of scope, it is step 5 of doc/driver-plan.md and
  the only part of this layer not unit-tested. Partial transfers are
  first-class so the real impl can be overlapped-with-owned-buffer or
  `PIPE_NOWAIT` without the interface changing.
- `Protocol` — wire types only (no codec): `DeviceMetadata` (id+kind) and
  `DevicePose` (id+tracking+pos+rot) as naturally-aligned PODs, memcpy'd
  whole — same style as `.tcrec`'s `writeRaw`/`readRaw` but one struct copy.
  `DeviceKind`/`TrackingState`/`MessageType` are this layer's own enums with
  pinned wire values; the driver maps `ETrackedDeviceClass` -> `DeviceKind`,
  the app maps `DeviceKind` -> `TrackedDeviceKind`. `tracking` collapses the
  two booleans the app ANDs in `OpenVrTracking::pollPoses`; zero = drop the
  device this frame (it then holds its last pose).
- `MessageChannel` — length-prefixed framing (u32 length, u16 type, payload)
  + reassembly over a `Pipe`. `send` frames into an outbound buffer capped at
  `kMaxPayloadBytes` (returns false when full — drop policy is the
   publisher's, step 3), `flush` retries the tail, `receive` drains the pipe
  and yields complete frames. Unknown types are skipped (consumed, not
  emitted); `length > kMaxPayloadBytes` is a permanent `Failed` (stream sync
  unrecoverable). Single-threaded, frame-driven.

## View, VR, entry point

- `Scene` (`src/view/`) — owns ALL GL resources (RAII: created after GLEW init, destroyed
  before GLFW shutdown — keep the scope in `main.cpp` intact). Orbit camera shared by
  both viewports (`setCameraTarget`/`setCameraYaw` for VR modes), `beginFrame` +
  `setViewport` per half, `renderSkeleton` (pyramid per bone), `renderTargets`
  (octahedron per target), two embedded GLSL 330 programs, `viewMatrix`/`projectionMatrix`
  for ImGuizmo.
- `OpenVrTracking` (`src/vr/`) — OpenVR-free header; converts OpenVR state into model
  `TrackedDevice` values (device index doubles as `id`). `init()` uses
  `VRApplication_Background` (never launches SteamVR), throws `Error`, retryable.
  `pollPoses()` = connected+valid HMD/controller/tracker poses.
  `bothTriggersJustPressed()` = calibration gesture edge.
- `src/main.cpp` — spdlog → GLFW/GL/GLEW → ImGui + ImGuizmo → load-or-create the three
  configs → `IkRig` → avatar skeleton + `RetargetMap` → `OpenVrTracking::init` (success:
  start in Calibration; failure: ManualPose). Per frame: poll poses once, read the
  trigger edge in Calibration only, `ModeController::update`, execute the returned plan,
  `SessionRecorder::update`, camera follow, left viewport (IK skeleton + gizmos in
  ManualPose), `retargetPose` + right viewport (avatar), ImGui panel, replay timeline.
  All mode logic lives in the model layer.

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
- Driver code (`src/spike`, later `src/driver*`) runs inside `vrserver.exe`: every
  function is either unit-tested or a pure one-instruction forwarder, no exception may
  escape a hook or provider entry point, and unit tests must not touch the filesystem,
  wall clock, environment or real threads — dependencies are injected as seams
  (`HookApi`, `DeviceProperties`, `LogSink`, clock fn). Files compiled only into the DLL
  (`SpikeDriver.cpp`, `SpikeClient.cpp`) hold no branch, loop, comparison or arithmetic.
- Error handling: throw `Error` (never from constructors — use `loadConfig`-style
  functions when validation must throw). Catch blocks live at the app boundary
  (`main.cpp`) and just log `e.what()`. JSON shape errors are left to nlohmann
  (declarative `from_json`, no manual `is_array`/`size` checks). Wrap third-party calls
  at the call site so the log says which call failed. No `catch (...)` outside the
  driver boundary.
- Keep log strings ASCII (MSVC is not given `/utf-8`).
- Indentation: 4 spaces in `src/model`, `src/view`, `src/spike`, `tests`; tabs in
  `src/main.cpp`. Match the file you are editing.
- Edit sources with the edit/write tools only, never shell text munging (regex escapes
  and PowerShell re-encoding have corrupted files here before).
- Update this file when structure, targets, build steps or libraries change.
