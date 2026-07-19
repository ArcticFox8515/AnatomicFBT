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
- Running the exe creates `user-skeleton.json` / `user-ikrig.json` in the working
  directory on first start (load-or-create pattern); invalid files fall back to defaults.

## Directory / file map

```
CMakeLists.txt        3 targets: TrackingCorrectorLib (static), TrackingCorrector (exe),
                      TrackingCorrectorTests (gtest). C++20.
conanfile.py          Dependencies + copies ImGui backends into src/bindings on generate.
doc/plan.md           4-milestone design doc. Read before architectural changes.
src/main.cpp          Entry point: GLFW/GL/ImGui init, config load-or-create, main loop,
                      ImGuizmo manipulation of IK targets, ImGui control panel.
src/model/            Pure data + JSON serialization. NO OpenGL — unit-testable.
src/view/             All OpenGL rendering (Scene).
src/bindings/         Vendored ImGui backends (imgui_impl_glfw/opengl3). AUTO-COPIED by
                      conanfile.py from the imgui package — gitignored, never edit.
tests/                GoogleTest suites mirroring src/model.
```

## Classes & responsibilities

### Model layer (`src/model/`) — no GL dependencies

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
  - Free `to_json`/`from_json` — schema `{ "bones": [{name, parent|null, offset:[x,y,z]}] }`.
    Deserialization validates: non-empty, unique names, exactly 1 root, known parents,
    no cycles; re-sorts parent-before-child. Throws `std::runtime_error`.
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
  down/back), cones for hips/shoulders. JSON validated, throws on errors. Kept
  separate from `Skeleton` because the milestone-3 avatar skeleton has no IK.
- `IkSolvers` (`IkSolvers.h/.cpp`) — `IkTarget` = `{jointIndex, position, rotation}`
  (a world-space manipulation handle) + the three per-target solver stages, each
  unit-testable in isolation:
  - `solveAnchor` — rigidly pins the root joint (`rootPosition` + root `localRot`).
  - `solveChain` — arc-swing/curl interpolation of a root-hanging chain (any length)
    so its end lands on the target; orientations blended root→target — exact in
    position when within reach, otherwise the end-bone orientation wins (documented
    interpolation trade-off).
  - `solveTwoBone` — two-bone analytic limb IK (socket→j1→j2→tip): places j2 on the
    goal implied by the tip target, tip bone takes the target rotation; pole in the
    socket's frame.
- `IkRig` (`IkRig.h/.cpp`) — owns `Skeleton` + `IkRigConfig` + `std::vector<IkTarget>`.
  No bone names in code: solver structure is derived from config + skeleton topology
  at construction (`SolverBinding` per target). Constructor validates (throws
  `std::runtime_error`): target/limit bones exist; anchor target is the root joint;
  chain target is not the root (chain = ancestor path root→joint); two_bone target
  has ≥3 ancestors (tip→j2→j1→socket walk) and its middle bone (j2) carries a `pole`
  in the limits. Places targets at the rest pose. `resetTargets()`, `targetName(i)`
  for UI labels.
  - `IkRig::solve()` — re-derives the full pose from current targets every call
    (stateless, call each frame). Stages by solver type, regardless of config order:
    all anchors → all chains → all two-bone limbs → joint-limits post-pass
    (`clampSwingTwist` on each `JointLimits` bone's `localRot`).

### View layer (`src/view/`)

- `Scene` (`Scene.h/.cpp`) — owns ALL GL resources (RAII: created after GLEW init,
  destroyed before GLFW shutdown — keep the scope in `main.cpp` intact):
  - Orbit camera: fixed target (0,1,0) and distance 3.5, RMB-drag changes yaw/pitch
    (`update(window, allowInput)` — pass `allowInput=false` when ImGui/gizmo captures
    the mouse).
  - `beginFrame(w,h)` — viewport/clear, camera matrices, ground grid.
  - `renderSkeleton()` — one stretched gray pyramid per bone (base at parent joint,
    apex at joint). `renderTargets()` — small orange octahedron per IK target.
  - Two shader programs compiled from embedded GLSL 330 strings: flat-color line
    program (grid) and diffuse-lit mesh program (pyramids/octahedra, one directional
    light). Static unit meshes in VBOs, per-instance `uModel` matrix.
  - `viewMatrix()/projectionMatrix()` — needed by ImGuizmo.

### Entry point (`src/main.cpp`)

`WinMain`: GLFW window (OpenGL 3.3 core) → GLEW init → ImGui + ImGuizmo setup →
load-or-create both JSON configs → construct `IkRig` (falls back to default config if
invalid). Per frame: `Scene::update/beginFrame`, then `manipulateTargets()` (one
`ImGuizmo::Manipulate` per target, `SetID` per index; T/R keys or radio buttons switch
translate/rotate), then `rig->solve()` ("Solve IK targets" checkbox toggles it), then
`renderSkeleton/renderTargets`, then the ImGui panel (status, per-target
position/rotation drag fields, reset button).

## Libraries (Conan, see `conanfile.py`)

| Library | Version | Used for |
|---|---|---|
| glfw | 3.4 | Window + input |
| glew | 2.2.0 | OpenGL function loading (`IMGUI_IMPL_OPENGL_LOADER_GLEW` defined) |
| glm | 1.0.1 | vec/mat/quat math everywhere |
| imgui | 1.90.5-**docking** | UI; `force=True` overrides imguizmo's pinned imgui — do not change version without reading conanfile comment |
| imguizmo | cci.20231114 | Translate/rotate gizmos for IK targets |
| nlohmann_json | 3.11.3 | Config (de)serialization via `to_json`/`from_json` free functions |
| gtest | 1.15.0 | Tests, wired via `gtest_discover_tests` |

## Conventions & gotchas

- **Separation**: model layer must stay GL-free (tests link only `TrackingCorrectorLib`
  and construct no GL context); view/app code depends on model, never the reverse.
- Coordinates: right-handed, **Y-up, meters**. Left side of the body at +X.
- Serialization: `localRot` and `Skeleton::rootPosition` are runtime-only; rest
  orientation is always identity for this skeleton (bone-roll problem is deferred to
  milestone 3 — see `doc/plan.md`).
- Indentation: 4 spaces in `src/model`, `src/view`, `tests`; tabs in `src/main.cpp`.
  Match the file you're editing.
- `src/bindings/` and `CMakeUserPresets.json` are generated — never edit by hand.
- Tests: add new suites to the `TrackingCorrectorTests` target in `CMakeLists.txt`;
  run with `run-tests.bat`.
- When changing anything described in this file (structure, classes, build steps,
  libraries), update this AGENTS.md accordingly.
