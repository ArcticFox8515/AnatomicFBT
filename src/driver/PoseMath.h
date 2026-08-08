#pragma once

// The DriverPose_t -> world pose composition, kept openvr-free and header-only so it
// can be unit-tested without SteamVR (tests/DriverTest.cpp).
//
// Doubles throughout: DriverPose_t is double-precision and this math is the one
// thing in the driver that must be provably exact.

#include <cmath>
#include <cstdio>
#include <string>

namespace driver
{
struct V3
{
    double x = 0.0, y = 0.0, z = 0.0;
};

struct Q
{
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
};

struct RigidPose
{
    V3 pos;
    Q rot;
};

inline V3 cross(const V3& a, const V3& b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline V3 add(const V3& a, const V3& b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Q mul(const Q& a, const Q& b)
{
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

inline V3 rotate(const Q& q, const V3& v)
{
    const V3 u{q.x, q.y, q.z};
    const V3 t = cross(u, v);
    const V3 c = cross(u, t);
    return {v.x + 2.0 * (q.w * t.x + c.x), v.y + 2.0 * (q.w * t.y + c.y),
            v.z + 2.0 * (q.w * t.z + c.z)};
}

// (a o b): apply b first, then a.
inline RigidPose compose(const RigidPose& a, const RigidPose& b)
{
    return {add(a.pos, rotate(a.rot, b.pos)), mul(a.rot, b.rot)};
}

// Row-major 3x4 rigid transform (OpenVR's HmdMatrix34_t layout: m[row][column],
// translation in the last column) -> pose. Same result as glm::quat_cast on the
// transposed matrix, i.e. the conversion the app uses, without the glm dependency:
// the largest-component branch keeps it numerically stable for every rotation.
inline RigidPose poseFromRowMajor34(const float m[3][4])
{
    const double xx = m[0][0], yy = m[1][1], zz = m[2][2];
    const double candidates[4] = {xx + yy + zz, xx - yy - zz, yy - xx - zz, zz - xx - yy};

    int biggest = 0;
    for (int i = 1; i < 4; ++i)
        if (candidates[i] > candidates[biggest])
            biggest = i;

    const double value = std::sqrt(candidates[biggest] + 1.0) * 0.5;
    const double scale = 0.25 / value;

    // Off-diagonal sums/differences, named after the matrix entries they come from.
    const double zy = m[2][1], yz = m[1][2];
    const double xz = m[0][2], zx = m[2][0];
    const double yx = m[1][0], xy = m[0][1];

    Q rot;
    switch (biggest)
    {
    case 0: rot = {value, (zy - yz) * scale, (xz - zx) * scale, (yx - xy) * scale}; break;
    case 1: rot = {(zy - yz) * scale, value, (yx + xy) * scale, (xz + zx) * scale}; break;
    case 2: rot = {(xz - zx) * scale, (yx + xy) * scale, value, (zy + yz) * scale}; break;
    default: rot = {(yx - xy) * scale, (xz + zx) * scale, (zy + yz) * scale, value}; break;
    }

    return {{m[0][3], m[1][3], m[2][3]}, rot};
}

inline std::string formatPose(const RigidPose& p)
{
    char buffer[192] = {};
    std::snprintf(buffer, sizeof(buffer),
                  "pos=(%9.4f %9.4f %9.4f) quat=(%8.5f %8.5f %8.5f %8.5f)", p.pos.x, p.pos.y,
                  p.pos.z, p.rot.w, p.rot.x, p.rot.y, p.rot.z);
    return buffer;
}
} // namespace driver
