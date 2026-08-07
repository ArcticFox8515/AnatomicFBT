# OpenVR-SpaceCalibrator — implementation notes for the driver plan

Source: `OpenVR-SpaceCalibrator`. Read as a reference for phase A of
`doc/driver-plan.md`. These are the findings on how SpaceCalibrator is
implemented, organized by topic.

## What SpaceCalibrator is

SpaceCalibrator is a **pose-rewriting shim** that sits inside `vrserver.exe`
and intercepts poses produced by other drivers, applying a per-device
transform before they reach SteamVR. It:
- never calls `TrackedDeviceAdded`,
- never implements `ITrackedDeviceServerDriver`,
- never reads properties (no `IVRProperties`, no `ReadPropertyBatch`),
- never touches `IVRDriverInput` (no boolean/scalar components),
- has an empty `RunFrame()`.

It hooks `IVRServerDriverHost::TrackedDevicePoseUpdated` and rewrites the
`WorldFromDriver` transform in-flight before forwarding the modified pose.

## 1. Driver DLL structure

- `OpenVR-SpaceCalibratorDriver.cpp:10` exports `HmdDriverFactory`:
  ```cpp
  void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode) {
      static ServerTrackedDeviceProvider server;
      static VRWatchdogProvider watchdog;
      if (std::strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName) == 0) return &server;
      else if (std::strcmp(vr::IVRWatchdogProvider_Version, pInterfaceName) == 0) return &watchdog;
      if (pReturnCode) *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
      return nullptr;
  }
  ```
  Two function-static singletons. SteamVR calls `HmdDriverFactory` for both
  the server and the watchdog provider (SteamVR loads a watchdog-mode copy
  of every driver at startup).
- `ServerTrackedDeviceProvider.h:14` — inherits `IServerTrackedDeviceProvider`,
  overrides only `Init`/`Cleanup`/`GetInterfaceVersions`; the rest are empty.
  `GetInterfaceVersions` returns `vr::k_InterfaceVersions` (the ABI
  validation table from the header).
- Manifest `01spacecalibrator\driver.vrdrivermanifest`:
  ```json
  { "alwaysActivate": true, "name": "01spacecalibrator", "directory": "", "resourceOnly": false }
  ```
  `alwaysActivate: true` causes SteamVR to load the driver even though it
  adds no devices.
- `01` prefix is a directory-name convention for alphabetical load order
  (not a SteamVR feature).
- SteamVR-expected directory layout (from `installer.nsi:127-134`):
  ```
  <SteamVR-runtime>\drivers\01spacecalibrator\
      driver.vrdrivermanifest
      resources\driver.vrresources
      resources\settings\default.vrsettings
      bin\win64\driver_01spacecalibrator.dll
  ```
  The `bin\win64\` subpath and `driver_<name>.dll` filename are SteamVR
  conventions; the manifest name determines the DLL filename.

## 2. Interface hooking with MinHook

`Hooking.{h,cpp}` — a templated `Hook<FuncType>` with
`CreateHookInObjectVTable(object, vtableOffset, detour)`:
```cpp
void **vtable = *((void ***)object);   // MSVC: first word of any object w/ virtuals
targetFunc = vtable[vtableOffset];
MH_CreateHook(targetFunc, detourFunction, (LPVOID *)&originalFunc);
MH_EnableHook(targetFunc);
```
The vtable pointer is **not** patched — the function code that the vtable
slot points to is patched (MinHook trampolines the body). `originalFunc`
is the trampoline that calls the real implementation. An `IHook` registry
tracks all hooks; `IHook::DestroyAll()` removes them; an `IHook::Exists(name)`
guard prevents double-hooking when `GetGenericInterface` returns the same
version twice.

`InterfaceHookInjector.cpp` — the actual hooks:

1. **`IVRDriverContext::GetGenericInterface`** — vtable index **0**,
   installed on the `IVRDriverContext*` passed into `Init()`. Every
   subsequent interface acquisition goes through it; per-interface hooks
   are installed lazily inside the detour when a recognized version string
   passes by.
   ```cpp
   GetGenericInterfaceHook.CreateHookInObjectVTable(pDriverContext, 0, &DetourGetGenericInterface);
   ```

2. **`IVRServerDriverHost::TrackedDevicePoseUpdated`** — vtable index **1**,
   installed inside `DetourGetGenericInterface` when the requested version
   matches a known string:
   ```cpp
   if (iface == "IVRServerDriverHost_005")
       TrackedDevicePoseUpdatedHook005.CreateHookInObjectVTable(originalInterface, 1, &DetourTrackedDevicePoseUpdated005);
   else if (iface == "IVRServerDriverHost_006")
       TrackedDevicePoseUpdatedHook006.CreateHookInObjectVTable(originalInterface, 1, &DetourTrackedDevicePoseUpdated006);
   ```
   Per `openvr_driver.h`, `IVRServerDriverHost` declares virtuals in this
   order — index 0 `TrackedDeviceAdded`, index 1 `TrackedDevicePoseUpdated`,
   index 2 `VsyncEvent`, …, index 9 `GetFrameTimings`. One `Hook` instance
   + one detour **per version string** — an unknown version means the hook
   silently isn't installed (no crash).

- Detour signature uses an explicit `_this` first parameter:
  ```cpp
  static void DetourTrackedDevicePoseUpdated(vr::IVRServerDriverHost * _this, uint32_t unWhichDevice,
                                             const vr::DriverPose_t & newPose, uint32_t unPoseStructSize);
  ```
  MSVC x64 convention — `__thiscall` is the regular calling convention
  with `this` in `rcx`.
- The detour copies the pose (`auto pose = newPose;`), lets the driver
  mutate it, and forwards only if a handler returned true. `newPose` is
  `const&` and the caller may still be reading its own buffer — the copy
  is required.
- `unPoseStructSize` is forwarded unchanged; never inspected. If the struct
  ever grows, new fields pass through untouched.
- `MH_Initialize()` is called once from `InjectHooks` (called from `Init`
  after `VR_INIT_SERVER_DRIVER_CONTEXT`); `MH_Uninitialize()` from
  `DisableHooks` (called from `Cleanup` before `VR_CLEANUP_SERVER_DRIVER_CONTEXT`).

## 3. Driver context setup

`ServerTrackedDeviceProvider.cpp:8-38`:
```cpp
vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) {
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);   // before anything else
    memset(transforms, 0, ...);
    InjectHooks(this, pDriverContext);               // hook GetGenericInterface on the raw ctx ptr
    server.Run();                                    // start IPC thread
    shmem.Create(OPENVR_SPACECALIBRATOR_SHMEM_NAME);
    return vr::VRInitError_None;
}
```
- `VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext)` is the OpenVR macro that
  calls `VRDriverContext()->SetDriverContext(pDriverContext)`; afterwards
  the `VRDriverLog()`, `VRProperties()`, `VRDriverInput()`,
  `VRServerDriverHost()` accessors work.
- The **raw** `pDriverContext` is passed into `InjectHooks` (not the
  post-init wrapper) so the hook lands on the same pointer SteamVR handed
  in.
- Cleanup is symmetric: `server.Stop()` → `shmem.Close()` → `DisableHooks()`
  → `VR_CLEANUP_SERVER_DRIVER_CONTEXT()`. Hooks are removed **before** the
  context is cleaned up — the detours reference the provider singleton,
  which is destroyed once `Cleanup` returns.
- SpaceCalibrator never uses `VRDriverLog`/`VRProperties`/`VRDriverInput`
  directly; it intercepts `IVRServerDriverHost` via the `GetGenericInterface`
  detour instead.

## 4. Pose composition

`ServerTrackedDeviceProvider.cpp:81-95`:
```cpp
IsoTransform toIsoWorldTransform(const vr::DriverPose_t& pose) {
    Eigen::Quaterniond rot(pose.qWorldFromDriverRotation...);
    Eigen::Vector3d trans(pose.vecWorldFromDriverTranslation...);
    return IsoTransform(rot, trans);
}
IsoTransform toIsoPose(const vr::DriverPose_t& pose) {
    return toIsoWorldTransform(pose) * IsoTransform(pose.qRotation, pose.vecPosition);
}
```
The **device world pose** as SpaceCalibrator computes it is
`WorldFromDriver ∘ (vecPosition, qRotation)`.

`DriverFromHead` (`qDriverFromHeadRotation`, `vecDriverFromHeadTranslation`)
is **not used** by SpaceCalibrator — it leaves it untouched. The transform
applied to a pose rewrites only `qWorldFromDriverRotation` and
`vecWorldFromDriverTranslation` (`ServerTrackedDeviceProvider.cpp:188-195`):
the final `WorldFromDriver' = calibrationTransform * WorldFromDriver`.
`vecPosition`, `qRotation`, `vecVelocity`, `vecAcceleration`,
`vecAngularVelocity`, `vecAngularAcceleration`, and `poseTimeOffset` are
never modified.

`poseTimeOffset`, velocity, and acceleration are not recomputed after the
position/rotation are rewritten. The reported velocities remain in the
original frame after a transform is applied.

`HandleDevicePoseUpdated` (lines 241-281) details:
- `openVRID > 0` guard on the debug offset — the HMD (index 0) is never
  offset by the debug transform. Indices are 0-based, 0 is always the HMD.
- `shmem.SetPose(openVRID, pose)` logs every pose to the shared-memory
  ring buffer **before** transform application — the client app sees raw
  device poses for calibration sampling, not the corrected ones.
- `tf.quash` overrides everything and parks the device 9001 m above origin
  (a "hide this device" affordance). Forwarding still happens (`return true`).
- When `tf.enabled`, the device position is first scaled
  (`vecPosition *= tf.scale`) — scale is applied to the driver-local
  position, before the world-from-driver recomposition.

`interpolateAround` (`IsometryTransform.h:42-49`): the blend lerps the
calibration transform toward a target **around the device's current world
position** rather than through the origin:
```cpp
auto initialPos = (*this) * localPoint;
Eigen::Vector3d finalPos = initialPos * (1 - lerp) + (target * localPoint) * lerp;
auto newRotation = rotation.slerp(lerp, target.rotation);
Eigen::Vector3d newTranslation = finalPos - Eigen::Isometry3d(newRotation) * localPoint;
```
where `localPoint = deviceWorldPose.translation`. The device stays roughly
in place during the blend instead of swinging through the origin.

The legacy `BlendTransform` uses a 3-band speed scheduler
(`GetTransformDeltaSize`: TINY/SMALL/LARGE thresholds on translation² and
rotation angular distance), with a sticky "ratchet up but cool down only
to TINY" rule (line 125-126: `max(prior_delta, max(trans_level, rot_level))`).
The newer `BlendTransformExponential` uses a single time-constant
`1 - exp(-dt/tau)` lerp, gated by `tf.useExponential`.

## 5. Property reads

The driver does zero property reads. Grep for `ReadPropertyBatch`,
`TrackedDeviceToPropertyContainer`, `GetPropertyContainer`, `IVRProperties`,
`Prop_`, or `GetStringTrackedDeviceProperty` in `OpenVR-SpaceCalibratorDriver\`
returns no matches. It operates purely on the `unWhichDevice` integer that
`TrackedDevicePoseUpdated` hands it; the client app tells it which device
IDs to apply transforms to via the IPC `SetDeviceTransform` message
carrying `openVRID`.

The **client app** (`OpenVR-SpaceCalibrator\VRState.cpp`) does property
reads using the client-side `IVRSystem` API, not the driver-side
`IVRProperties` API:
```cpp
for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
    auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
    if (deviceClass == vr::TrackedDeviceClass_Invalid) continue;
    if (deviceClass != vr::TrackedDeviceClass_TrackingReference) {
        vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, ...);
        // ... Prop_ModelNumber_String, Prop_SerialNumber_String ...
        device.controllerRole = (vr::ETrackedControllerRole)vr::VRSystem()->GetInt32TrackedDeviceProperty(id, vr::Prop_ControllerRoleHint_Int32, &err);
    }
}
```
`TrackedDeviceClass_TrackingReference` (lighthouses / Oculus sensors) is
skipped. The HMD's tracking system is forced to the front of the
`trackingSystems` list (lines 28-32). Device identity is the triple
`(trackingSystem, model, serial)`.

## 6. Boolean component / button handling

Not implemented. Grep for `CreateBooleanComponent`, `UpdateBooleanComponent`,
`IVRDriverInput`, `GetComponentState`, `GetBoneCount`, `GetBoneTransforms`
in the driver directory returns no matches. SpaceCalibrator never creates
input components, never forwards button state, never maps handles to
devices. Real device input flows through the original drivers untouched.

## 7. RunFrame usage

Empty. `ServerTrackedDeviceProvider.h:29`:
```cpp
virtual void RunFrame() { }
```
No deferred work, no metadata refresh, no container-to-device resolution.
All work happens either inside the `TrackedDevicePoseUpdated` detour
(which runs on the calling driver's thread inside vrserver) or on the IPC
server thread when a `SetDeviceTransform` request arrives. There is no
per-frame polling, no state machine, no periodic resync — the transform
applied to each pose is whatever was last set via IPC, blended toward the
target each time a pose comes through.

## 8. Threading model

Threads in play:
1. **vrserver main thread** — calls `Init`, `RunFrame`, `Cleanup`.
2. **Per-device driver threads inside vrserver** — each real driver calls
   `TrackedDevicePoseUpdated` on its own thread. The SpaceCalibrator detour
   runs on **whichever thread the original driver called from**.
   `HandleDevicePoseUpdated` is reentrant; multiple devices can be updated
   concurrently.
3. **IPC server thread** — `IPCServer::RunThread` (`IPCServer.cpp:75`),
   started by `IPCServer::Run()` in `Init`. Uses overlapped named pipes
   with completion routines (`ReadFileEx`/`WriteFileEx` +
   `WaitForSingleObjectEx(TRUE)` for alertable wait). All IPC requests
   dispatch to `HandleRequest` on this thread.
4. **MinHook** — thread-safe for `MH_CreateHook`/`MH_EnableHook`/`MH_RemoveHook`
   via its own spinlock. Hooks are installed once during `Init` and removed
   once during `Cleanup`.

There is **no mutex** protecting `transforms[]` in `HandleDevicePoseUpdated`.
Two drivers updating devices N and M simultaneously both read/write
`transforms[N]` / `transforms[M]` (different elements, fine), but a
`SetDeviceTransform` IPC call writing `transforms[N].targetTransform`
concurrently with a pose update reading it is unsynchronized. It does not
crash on x64 because `bool`/`double`/`Quaterniond` reads/writes are
atomic-ish and a torn read produces a single bad frame.

The detour (`DetourTrackedDevicePoseUpdated`) does non-trivial Eigen math
(quaternion slerp, slerp+lerp in `interpolateAround`) per pose per device
but no I/O, no locks, no syscalls beyond `QueryPerformanceCounter`. The
IPC server uses overlapped I/O so it never blocks the IPC thread on a
single client. `SetNamedPipeHandleState` puts the client end in
`PIPE_READMODE_MESSAGE`. The IPC `HandleRequest` is synchronous (struct
copies into `transforms[]`).

## 9. Logging

The driver does **not** route logs into `vrserver.txt` via `IVRDriverLog`.
It uses its own `FILE*` (`Logging.{h,cpp}`):
```cpp
extern FILE *LogFile;
void OpenLogFile();
#define LOG(fmt, ...) do { \
    tm logNow = TimeForLog(); \
    fprintf(LogFile, "[%02d:%02d:%02d] " fmt "\n", logNow.tm_hour, logNow.tm_min, logNow.tm_sec, __VA_ARGS__); \
    LogFlush(); \
} while (0)
#define TRACE(...) {}
```
`OpenLogFile()` is called from `DllMain` on `DLL_PROCESS_ATTACH` — before
`Init` runs — so early-load logs survive:
```cpp
void OpenLogFile() {
    LogFile = fopen("space_calibrator_driver.log", "a");
    if (LogFile == nullptr) LogFile = stderr;
}
```
The file is opened append-mode relative to the process's working directory,
which for a vrserver-loaded driver is the SteamVR runtime's `bin\win64`
directory (the NSIS uninstaller hard-codes
`$vrRuntimePath2\drivers\01spacecalibrator\bin\win64\space_calibrator_driver.log`
as the cleanup path). Every `LOG` call flushes so a crash doesn't lose
the tail. `TRACE` is compiled to `{}` by default.

## 10. SteamVR install path discovery and driver registration

No `vrpathreg.exe` usage anywhere. Driver registration relies entirely on
the convention that `<SteamVR-runtime>\drivers\<name>\` is auto-discovered
by SteamVR at startup.

Install path discovery is done by the **client app** via the OpenVR client
API. `OpenVR-SpaceCalibrator.cpp:433-450` (`HandleCommandLine` with
`-openvrpath`):
```cpp
vr::VR_Init(&vrErr, vr::VRApplication_Utility);
char cruntimePath[MAX_PATH] = { 0 };
unsigned int pathLen;
vr::VR_GetRuntimePath(cruntimePath, MAX_PATH, &pathLen);
printf("%s", cruntimePath);
```
`VR_GetRuntimePath` is the OpenVR client SDK function that reads from
`openvrpaths.vrpath` (the SDK does the JSON parse internally). The NSIS
installer invokes the client exe with `-openvrpath` and captures stdout
to learn where to copy the driver files (`installer.nsi:108-111`).

Manifest install / removal (`OpenVR-SpaceCalibrator.cpp:451-512`):
`vr::VRApplications()->AddApplicationManifest(manifestPath)` /
`RemoveApplicationManifest` + `SetApplicationAutoLaunch(OPENVR_APPLICATION_KEY, true)`.
The manifest (`OpenVR-SpaceCalibrator\manifest.vrmanifest`) declares the
dashboard overlay:
```json
{
    "source" : "builtin",
    "applications": [{
        "app_key": "pushrax.SpaceCalibrator",
        "launch_type": "binary",
        "binary_path_windows": "OpenVR-SpaceCalibrator.exe",
        "is_dashboard_overlay": true,
        "strings": { "en_us": { "name": "Space Calibrator", ... } }
    }]
}
```
This registers the client app as a SteamVR dashboard overlay, not the
driver.

`activateMultipleDrivers` (`OpenVR-SpaceCalibrator.cpp:150-180`): the
installer runs the client with `-activatemultipledrivers`, which calls
`vr::VRSettings()->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_ActivateMultipleDrivers_Bool, true)`.
This SteamVR setting allows multiple tracking-system drivers (e.g. Oculus
+ Lighthouse) to coexist — without it, SteamVR refuses to load a second
driver that claims the same device class as an already-loaded one.

## 11. Building / CMake

Pure VS / MSBuild, no Conan/vcpkg. Solution: `OpenVR-SpaceCalibrator.sln`
at the repo root. Driver project:
`OpenVR-SpaceCalibratorDriver\OpenVR-SpaceCalibratorDriver.vcxproj`.

Key build facts (from the `.vcxproj`):
- `ConfigurationType=DynamicLibrary`, `PlatformToolset=v143` (VS 2022),
  x64 only (Debug + Release).
- `<TargetName>driver_01spacecalibrator</TargetName>` — output is
  `driver_01spacecalibrator.dll`, matching the manifest name
  `01spacecalibrator`.
- Preprocessor: `OPENVRSPACECALIBRATORDRIVER_EXPORTS` (enables `dllexport`
  on `HmdDriverFactory`).
- Include dirs: `..\lib;..\lib\openvr;..\lib\MinHook\include`.
- Library dirs: `..\lib\MinHook\lib` (only MinHook's static lib).
- **No `openvr_api.lib` linked.** `AdditionalDependencies` is just the
  standard Windows libs (kernel32, user32, etc.). The driver uses
  `openvr_driver.h` as a header-only interface — SteamVR's `vrserver.exe`
  provides the implementations at runtime via the vtables it hands to
  `Init`.
- MinHook integrated as a `ProjectReference` to
  `..\lib\minhook\build\VC16\libMinHook.vcxproj`. MinHook is a git
  submodule (`.gitmodules`: `lib/minhook -> github.com/bdunderscore/minhook`;
  the `bdunderscore` fork is a maintained branch, upstream is
  `TsudaKageyu/minhook`).
- Eigen is header-only, vendored at `lib\Eigen`.
- `openvr_driver.h` is vendored at `lib\openvr\openvr_driver.h`.

The **client app** (`OpenVR-SpaceCalibrator\OpenVR-SpaceCalibrator.vcxproj`)
is a Win32 `Application` (GUI) that links `openvr_api.lib`
(`lib\openvr\lib\win64\openvr_api.lib`) and ships `openvr_api.dll`
alongside the exe — the client uses the client-side OpenVR API
(`VR_Init`, `VRSystem`, `VROverlay`, `VRSettings`, `VRApplications`),
which requires the import lib and the runtime DLL. The driver DLL does not.

## 12. IPC protocol and shared memory

`Protocol.h` is shared between the client and driver projects (repo root,
included via relative path).

**Version handshake** (`IPCClient.cpp:43-53`): the first IPC message is a
`RequestHandshake`; the client verifies
`response.protocol.version == protocol::Version` (currently `5`). On
mismatch the client throws "Incorrect driver version installed, try
reinstalling."

**Shared-memory ring buffer** (`Protocol.h:212-374`, `DriverPoseShmem`):
every pose is logged to a 64K-entry SPSC ring
(`std::atomic<uint64_t> index`, powers-of-two masking,
`memory_order_acquire`/`release` fences) so the client app can sample
raw device poses for calibration fitting without IPC round-trips.
`AugmentedPose` carries `LARGE_INTEGER sample_time` (from
`QueryPerformanceCounter`) alongside the `DriverPose_t` and `deviceId`.
The reader resyncs if it falls more than half the buffer behind
(`BUFFERED_SAMPLES / 2`).

## 13. Additional implementation details

- The watchdog provider is implemented but does nothing. SteamVR
  instantiates every driver in watchdog mode at startup (a separate
  process, `vrwatchdog.exe`, that monitors whether VR hardware is connected
  to wake the system). The minimal `IVRWatchdogProvider` stub
  (`VRWatchdogProvider.h`) calls `VR_INIT_WATCHDOG_DRIVER_CONTEXT` in
  `Init` and `VR_CLEANUP_WATCHDOG_DRIVER_CONTEXT()` in `Cleanup`.
- The legacy `000spacecalibrator` beta is explicitly cleaned up by the
  NSIS installer (`installer.nsi:114-124`).
