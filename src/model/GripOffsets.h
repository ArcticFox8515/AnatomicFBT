#pragma once

#include "Pose.h"
#include "TrackedDevice.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <vector>

// Per-controller grip-pose offset: a rigid transform from the device's raw
// tracked pose to its grip-pose frame — the OpenVR render-model "grip"
// component (fallback "handgrip" on controllers that lack one). Composed
// onto the raw device pose each frame so the controller pose lands at the
// grip point, which is where the user's hand bone sits. See
// doc/ik-improvements-plan.md (controller pose = tip, not palm).
//
// Keyed by `TrackedDevice::id` (the vrserver device index, which agrees with
// the client `TrackedDeviceIndex_t` and the link id). Identity by default —
// no entry for a device means no shift, so non-controllers and unrecognised
// controllers pass through untouched.
//
// The `.tcrec` roster stores one `GripOffset` per device (identity for
// non-controllers) so replay reproduces the live shift without SteamVR; see
// Recording.h. `mergeGripOffsets` keeps a cached entry whose id is absent
// from a fresh query, so a controller powered off at the second calibration
// does not lose its offset.
struct GripOffset
{
    int deviceId = -1;
    Pose deviceToGrip;  // identity by default
};

// Composes each controller's raw pose with its grip offset:
// `shifted = compose(rawPose, deviceToGrip)`. Non-controllers and devices
// without an entry are copied through untouched. Roster-size vector, so a
// linear scan per device is fine (<10 devices).
std::vector<TrackedDevice> applyGripOffsets(const std::vector<TrackedDevice>& devices,
                                            const std::vector<GripOffset>& offsets);

// Merges a freshly queried set into a cached set by device id: fresh entries
// overwrite cached ones with the same id; cached entries whose id is absent
// from the fresh set are kept (a controller powered off at the second
// calibration keeps its offset). Order follows `fresh` then any cached-only
// ids in their original cached order, so the result is stable for the
// recorder's roster.
std::vector<GripOffset> mergeGripOffsets(const std::vector<GripOffset>& cached,
                                         const std::vector<GripOffset>& fresh);

// OpenVR `HmdMatrix34_t` (row-major 3x4, translation in the last column) ->
// Pose. Header-only so the model-layer unit test can exercise it without
// pulling openvr.h; the caller (src/vr/OpenVrInput) passes the
// `mTrackingToComponentLocal` field of a `RenderModel_ComponentState_t`.
// `quat_cast` on the transposed upper-left 3x3 yields the rotation; the
// translation is `m[r][3]`. Same result as `driver::poseFromRowMajor34`
// (src/driver/PoseMath.h) — both implement the largest-component quaternion
// extraction — but in glm so it stays in the model layer.
inline Pose poseFromHmdMatrix34(const float m[3][4])
{
    // glm::mat3 is column-major; OpenVR's m is row-major (m[row][col]), so
    // transpose on fill: column i of the glm matrix is row i of m.
    const glm::mat3 basis(m[0][0], m[1][0], m[2][0],  // column 0
                          m[0][1], m[1][1], m[2][1],  // column 1
                          m[0][2], m[1][2], m[2][2]); // column 2
    return {glm::vec3(m[0][3], m[1][3], m[2][3]), glm::quat_cast(basis)};
}
