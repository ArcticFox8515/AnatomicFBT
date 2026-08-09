# Plan

## Milestone 1 — Skeleton config + visualizer

**Deliverables:** an app that loads a skeleton from JSON and renders it in rest pose with orbit camera.

**Skeleton model.** SlimeVR's hierarchy (from its `BoneType`/skeleton code):

```
HEAD → NECK → UPPER_CHEST → CHEST → WAIST → HIP
HIP → {LEFT,RIGHT}_HIP → UPPER_LEG → LOWER_LEG → FOOT
UPPER_CHEST → {LEFT,RIGHT}_SHOULDER → UPPER_ARM → LOWER_ARM → HAND
```

Note SlimeVR's spine is head-rooted (head is the root, spine goes downward) because HMD is the most reliable tracker. Adopt that — it simplifies milestone 2.

**JSON schema.** Keep it minimal:

```json
{
  "bones": [
    { "name": "neck", "parent": "head", "offset": [0.0, -0.1, 0.0] }
  ]
}
```

`offset` = translation from parent joint in the parent's rest frame, meters. Rest orientation = identity for all joints (pure translation hierarchy). This avoids the bone-roll convention problem entirely on skeleton #1; you only confront it in milestone 3 where it's unavoidable.

**Code structure:**
- `Skeleton`: flat array of joints sorted parent-before-child (`{ name, parentIndex, glm::vec3 restOffset, glm::quat localRot }`), plus `computeWorldTransforms()` — one linear pass.
- Renderer: lines or thin cuboids between joint world positions, octahedron/sphere at joints. Simplest path: one VBO of line segments rebuilt per frame, single shader. Don't build a scene graph.
- ImGui panel: load/reload JSON button, joint tree view, selected-joint world position readout.
- Stack: GLFW + OpenGL 3.3 + glm + Dear ImGui + nlohmann/json. All header-only or trivially vendored.

**Done when:** editing the JSON and hitting reload changes the displayed skeleton; proportions visibly correct against a 1m grid.

## Milestone 2 — IK rig + solver

**Deliverables:** movable tracker targets for head/hands/feet/hips; skeleton follows.

**Tracker targets.** A target = position + orientation, bound to a joint by name. Manipulation: ImGuizmo (translate/rotate gizmo widget for ImGui) — this is exactly its use case. Add ImGui fields for numeric entry as fallback.

**Solver structure**, in order of application:
1. **Head:** rigidly pinned to head target (position + rotation).
2. **Spine (head→hip):** hips target defines hip position/rotation; distribute spine bend between neck/chest/waist. SlimeVR does this with interpolation weights, not general IK — copy that approach. FABRIK on the spine chain is the alternative if interpolation looks bad.
3. **Legs:** two-bone analytic IK per leg (hip→knee→ankle, pole/hint vector for knee direction). Foot target rotation drives foot bone.
4. **Arms:** same two-bone analytic IK (shoulder→elbow→wrist), elbow hint.
5. **Joint limits:** swing-twist decomposition per joint, clamp twist range and swing cone. Apply after solve. Knee/elbow: hinge (1-DOF swing, near-zero twist).

Two-bone analytic IK is a closed-form ~30-line function; write it before considering FABRIK for anything.

**Config additions:** which joints accept targets; per-joint limit parameters (twist min/max, swing cone angle) in the JSON.

**Done when:** dragging targets produces plausible poses, knees/elbows never invert, and pulling a target beyond reach stretches the chain toward it without exploding.

## Milestone 3 — Avatar skeleton + retargeting

**Deliverables:** Unity exporter → second JSON; both skeletons rendered side by side; rotations transferred by name.

**Unity exporter.** Editor script, not a runtime plugin: take the avatar's `Animator`, walk `HumanBodyBones` (this gives you Unity's canonical humanoid bone names for free — use those as the shared naming scheme in *both* skeletons from milestone 1 onward, e.g. `Hips`, `LeftUpperLeg`), and export per bone:
- parent, local rest position, **local rest rotation** (avatars have non-identity rest rotations — this is the difference from skeleton #1's schema), 
- and the T-pose: sample `HumanPoseHandler` or force the avatar into T-pose via the humanoid rig and export world rotations in that pose.

**Retargeting.** By-name rotation copy only works after rest-pose normalization. Standard delta transfer:

```
delta_world = worldRot_src_current * inverse(worldRot_src_tpose)
worldRot_dst = delta_world * worldRot_dst_tpose
localRot_dst = inverse(worldRot_dst_parent) * worldRot_dst
```

i.e., transfer *world-space deltas from T-pose*, not local rotations. This is what makes differing bone rolls and rest poses a non-issue. Requires both skeletons to store their T-pose world rotations — for skeleton #1 that's identity by construction.

Implemented (pre-Unity-exporter stage, both skeletons at identity rest rotations): bones are matched by the *unordered pair* of joint names they connect, not by joint name — a joint's rotation orients the bone **ending** at it in our model, so a head-rooted src and a hip-rooted dst own each spine segment at opposite ends, and naive by-name copying shifts every spine segment by one joint. With rest rotations identity the delta form reduces to copying src world rotations per matched bone (`RetargetMap`/`retargetPose` in `src/model/Retarget.cpp`); the dst root joint matches by plain name, and the dst root position is shifted so the joint named like the src root (`head`, the fixed HMD) lands exactly on its src world position. When the Unity exporter lands, keep the pair matching and add the rest-pose delta normalization above.

Missing bones (avatar lacks `UpperChest`, extra twist bones, etc.): skip unmatched names; deltas being world-space means a skipped intermediate bone degrades gracefully.

**Done when:** posing skeleton #1 with the milestone-2 gizmos produces the same pose on an avatar with different proportions, no limb twisting, verified on at least one avatar with non-trivial rest pose (e.g. A-pose rest).

## Milestone 4 — SteamVR wiring

**Deliverables:** real trackers drive skeleton #1; virtual trackers emitted from skeleton #2's joints.

**Input side (no driver needed):** the app links `openvr_api` as a client (`VRApplication_Background` or `_Overlay`), reads poses via `GetDeviceToAbsoluteTrackingPose`, identifies trackers by serial/role. Replace the gizmo targets with these poses. Keep gizmo mode as a debug toggle.

**Calibration** becomes necessary here: mapping tracker poses to joint poses (tracker is strapped somewhere on the limb, not at the joint). Minimum viable: T-pose capture — record tracker-to-joint offsets in one calibration pose, apply as fixed offsets after. This is what SlimeVR and Standable both do (single-pose calibration).

**Output side:** driver DLL emitting virtual trackers, fed over IPC from the app.
- Base it on SlimeVR-OpenVR-Driver (MIT) — it is precisely "IPC in, `TrackedDeviceAdded` + `TrackedDevicePoseUpdated` out."
- IPC: named pipe or UDP on localhost; SlimeVR-OpenVR-Driver already implements a protocol, cheapest path is speaking it.
- Virtual tracker set: hips, chest, feet, knees, elbows from skeleton #2 joint world transforms. Set `Prop_ControllerType_String` etc. so SteamVR treats them as `vive_tracker`-compatible for role binding.
- Real trackers stay visible; they just remain unassigned in SteamVR tracker roles.
- **Existing-tracker correction (in progress):** the pose hook installed in phase A is also where existing trackers are placed on the avatar. App-side, `TrackerCorrection` places each bound, enabled target's tracker at the avatar joint's world pose — the bone center, no strap offset. The bone-local offset captured at calibration is deliberately dropped: the reference and avatar skeletons have differently-oriented bone frames (rest rotations / bone roll), so re-hanging in the local frame rotates the tracker wrong. Controllers are always rotation-locked: their rotation is kept exactly as the raw device reported it (aiming must not change), so only the position is corrected — the driver delta comes out with an exactly-identity rotation. Trackers honor a per-group rotation toggle (`groupRotationEnabled`): when off, the tracker keeps its raw rotation too (position-only correction) — useful when avatar bone-roll differences produce a visually wrong tracker orientation. The avatar skeleton is height-scaled to the reference once at startup (`matchRestHeight`), so overall height is preserved and correction only redistributes proportions — no 180 cm body squashed into a 150 cm avatar. The corrected poses are rendered as markers in the right viewport next to the avatar; per-group on/off + rotation toggle + a read-only avatar scale readout are in the ImGui panel. Driver-side application — overriding `TrackedDevicePoseUpdated` through a reverse app→driver channel so SteamVR and VRChat see the corrected poses — is the next step.

**Done when:** VRChat with roles bound to the virtual trackers reproduces poses standing/sitting/kneeling, and knees track correctly on an avatar with proportions clearly different from the tracked body.

## Cross-cutting notes

- Keep solver and rendering strictly separated from frame 1 — milestone 4 runs the solver at tracker rate (90+ Hz) independent of the debug window.
- Coordinate systems: OpenVR is right-handed Y-up meters; Unity is left-handed Y-up. The exporter must convert (negate Z on positions, adjust quaternions: negate x,y or z,w depending on convention — verify with a known asymmetric pose, not by eye).
- Write a pose-recording/playback (dump tracker poses to file) as soon as milestone 4 input works — debugging IK against live tracking without replay is miserable.




## Idea stash

- use polar limits (4 half-angles) for better anatomical skeleton
- at calibration auto compensate for arm and foot asymmetry
- build default skeleton based on body proportions, save only proportions, not the whole skeleton into config