#pragma once

#include <float.h>
#include <math.h>

#include "core/common.h"

// Rotations and rigid transforms. Trigonometry uses libm, which Nix pins, so results are
// identical on every machine running the same build.

// A unit quaternion: v = axis * sin(angle / 2), s = cos(angle / 2).
struct Quat {
    v3 v;
    f32 s;
};

// A rigid transform: rotate by q, then move by p. Also a frame's pose in its parent.
struct Pose {
    v3 p;
    Quat q;
};

// A 3x3 matrix, by columns.
struct Mat3 {
    v3 cx;
    v3 cy;
    v3 cz;
};

static const Quat identity_quat = {{0.0f, 0.0f, 0.0f}, 1.0f};
static const Pose identity_pose = {{0.0f, 0.0f, 0.0f}, {{0.0f, 0.0f, 0.0f}, 1.0f}};

inline f32 dot(v3 a, v3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline v3 cross(v3 a, v3 b)
{
    return v3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline f32 get_length(v3 v) { return sqrtf(dot(v, v)); }

// The unit vector along v, or zero for a vector too short to have a direction.
inline v3 normalize(v3 v)
{
    f32 length_squared = dot(v, v);
    if (length_squared > 1000.0f * FLT_MIN) {
        return (1.0f / sqrtf(length_squared)) * v;
    }
    return v3{0.0f, 0.0f, 0.0f};
}

inline v3 lerp(v3 a, v3 b, f32 alpha) { return (1.0f - alpha) * a + alpha * b; }

inline v3 rotate_vector(Quat q, v3 v)
{
    v3 t = cross(q.v, v) + q.s * v;
    return v + 2.0f * cross(q.v, t);
}

inline v3 inverse_rotate_vector(Quat q, v3 v)
{
    v3 t = cross(q.v, v) - q.s * v;
    return v + 2.0f * cross(q.v, t);
}

inline Quat multiply_quats(Quat a, Quat b)
{
    return Quat{cross(a.v, b.v) + a.s * b.v + b.s * a.v, a.s * b.s - dot(a.v, b.v)};
}

// inverse(a) * b: b relative to a.
inline Quat inverse_multiply_quats(Quat a, Quat b)
{
    return Quat{cross(b.v, a.v) + a.s * b.v - b.s * a.v, a.s * b.s + dot(a.v, b.v)};
}

inline Quat normalize_quat(Quat q)
{
    f32 length_squared = dot(q.v, q.v) + q.s * q.s;
    if (length_squared > 1000.0f * FLT_MIN) {
        f32 s = 1.0f / sqrtf(length_squared);
        return Quat{s * q.v, s * q.s};
    }
    return identity_quat;
}

// Blends two rotations by alpha (0..1) the short way round, normalised.
inline Quat nlerp_quats(Quat a, Quat b, f32 alpha)
{
    if (dot(a.v, b.v) + a.s * b.s < 0.0f) {
        a = Quat{-a.v, -a.s};
    }
    return normalize_quat(Quat{lerp(a.v, b.v, alpha), (1.0f - alpha) * a.s + alpha * b.s});
}

inline Quat make_quat_from_axis_angle(v3 unit_axis, f32 radians)
{
    f32 s = sinf(0.5f * radians);
    return Quat{{unit_axis.x * s, unit_axis.y * s, unit_axis.z * s}, cosf(0.5f * radians)};
}

// The shortest rotation taking unit vector from to unit vector to.
inline Quat make_quat_between(v3 from, v3 to)
{
    v3 middle = lerp(from, to, 0.5f);
    f32 tolerance = 100.0f * FLT_EPSILON;
    Quat q;
    if (dot(middle, middle) > tolerance * tolerance) {
        q = Quat{cross(from, middle), dot(from, middle)};
    } else if (absolute(from.x) > 0.5f) {
        q = Quat{{from.y, -from.x, 0.0f}, 0.0f}; // opposite: half a turn about a perpendicular
    } else {
        q = Quat{{0.0f, from.z, -from.y}, 0.0f};
    }
    return normalize_quat(q);
}

// The angle of q about z, in -pi..pi, ignoring any tilt of the axis.
inline f32 get_twist_angle(Quat q)
{
    return q.s < 0.0f ? 2.0f * atan2f(-q.v.z, -q.s) : 2.0f * atan2f(q.v.z, q.s);
}

// a * b: b's frame given in a's, taken to a's parent.
inline Pose multiply_poses(Pose a, Pose b)
{
    return Pose{rotate_vector(a.q, b.p) + a.p, multiply_quats(a.q, b.q)};
}

// inverse(a) * b: b's frame relative to a's, both given in the same parent.
inline Pose inverse_multiply_poses(Pose a, Pose b)
{
    return Pose{inverse_rotate_vector(a.q, b.p - a.p), inverse_multiply_quats(a.q, b.q)};
}

inline v3 transform_point(Pose t, v3 v) { return rotate_vector(t.q, v) + t.p; }

inline v3 inverse_transform_point(Pose t, v3 v) { return inverse_rotate_vector(t.q, v - t.p); }

inline Mat3 make_matrix_from_quat(Quat q)
{
    f32 xx = q.v.x * q.v.x, yy = q.v.y * q.v.y, zz = q.v.z * q.v.z;
    f32 xy = q.v.x * q.v.y, xz = q.v.x * q.v.z, yz = q.v.y * q.v.z;
    f32 xw = q.v.x * q.s, yw = q.v.y * q.s, zw = q.v.z * q.s;
    return Mat3{
        {1.0f - 2.0f * (yy + zz), 2.0f * (xy + zw), 2.0f * (xz - yw)},
        {2.0f * (xy - zw), 1.0f - 2.0f * (xx + zz), 2.0f * (yz + xw)},
        {2.0f * (xz + yw), 2.0f * (yz - xw), 1.0f - 2.0f * (xx + yy)},
    };
}

inline v3 multiply_matrix_vector(Mat3 m, v3 v) { return v.x * m.cx + v.y * m.cy + v.z * m.cz; }

inline Mat3 multiply_matrices(Mat3 a, Mat3 b)
{
    return Mat3{multiply_matrix_vector(a, b.cx), multiply_matrix_vector(a, b.cy),
                multiply_matrix_vector(a, b.cz)};
}

inline Mat3 transpose_matrix(Mat3 m)
{
    return Mat3{{m.cx.x, m.cy.x, m.cz.x}, {m.cx.y, m.cy.y, m.cz.y}, {m.cx.z, m.cy.z, m.cz.z}};
}

inline Mat3 add_matrices(Mat3 a, Mat3 b) { return Mat3{a.cx + b.cx, a.cy + b.cy, a.cz + b.cz}; }

inline f32 get_determinant(Mat3 m) { return dot(m.cx, cross(m.cy, m.cz)); }

// The inertia a point mass adds about a point offset from it (the parallel axis theorem).
inline Mat3 get_offset_inertia(f32 mass, v3 offset)
{
    f32 xy = -mass * offset.x * offset.y;
    f32 xz = -mass * offset.x * offset.z;
    f32 yz = -mass * offset.y * offset.z;
    return Mat3{
        {mass * (offset.y * offset.y + offset.z * offset.z), xy, xz},
        {xy, mass * (offset.x * offset.x + offset.z * offset.z), yz},
        {xz, yz, mass * (offset.x * offset.x + offset.y * offset.y)},
    };
}
