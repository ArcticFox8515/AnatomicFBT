#pragma once

// Conversions between `link::WireVec3` / `link::WireQuat` and the math types
// used on either side of the pipe. The link layer stays std-only (no glm, no
// openvr); these templates are duck-typed — they only require the target type
// to expose `.x` / `.y` / `.z` (and `.w` for quaternions) and to be
// brace-constructible in `(x, y, z)` / `(w, x, y, z)` order. Both `glm::vec3`
// / `glm::quat` (model layer) and `driver::V3` / `driver::Q` (driver layer,
// double-precision) satisfy that, so one header serves both.
//
// The templates are never instantiated inside `LinkLib` (it does not call
// them), so the constraint that LinkLib links no glm is preserved; they
// instantiate only in the consuming translation units, which already have
// their math headers.

#include "Protocol.h"

namespace link
{
// Any vec3-like type with .x/.y/.z -> WireVec3. Casts to float so
// double-precision sources (driver::V3) truncate cleanly.
template <typename V>
WireVec3 toWireVec3(const V& v)
{
    return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

// Any quat-like type with .x/.y/.z/.w -> WireQuat (xyzw on the wire).
template <typename Q>
WireQuat toWireQuat(const Q& q)
{
    return {static_cast<float>(q.x), static_cast<float>(q.y),
            static_cast<float>(q.z), static_cast<float>(q.w)};
}

// WireVec3 -> any vec3-like type constructible from {x, y, z}. Both
// glm::vec3(float x,y,z) and driver::V3(double x,y,z) accept this.
template <typename V>
V fromWireVec3(const WireVec3& v)
{
    return {v.x, v.y, v.z};
}

// WireQuat (xyzw) -> any quat-like type constructible from {w, x, y, z}.
// Both glm::quat(w,x,y,z) and driver::Q{w,x,y,z} use that order.
template <typename Q>
Q fromWireQuat(const WireQuat& q)
{
    return {q.w, q.x, q.y, q.z};
}
} // namespace link
