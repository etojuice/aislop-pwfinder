// vec.h - float-only 3D vector math.
//
// Everything in the collision/trace path is single-precision `float`, exactly
// as the GoldSrc engine does it. The pixelwalk bug is a 32-bit-float rounding
// artifact (DIST_EPSILON boundary wobble); using double would mask it. Do NOT
// change vec_t to double.
#pragma once
#include <cmath>

namespace pw {

typedef float vec_t;
typedef vec_t vec3_t[3];

static inline vec_t DotProduct(const vec3_t a, const vec3_t b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static inline void VectorCopy(const vec3_t in, vec3_t out) {
    out[0] = in[0]; out[1] = in[1]; out[2] = in[2];
}

static inline void VectorSubtract(const vec3_t a, const vec3_t b, vec3_t out) {
    out[0] = a[0] - b[0]; out[1] = a[1] - b[1]; out[2] = a[2] - b[2];
}

static inline void VectorAdd(const vec3_t a, const vec3_t b, vec3_t out) {
    out[0] = a[0] + b[0]; out[1] = a[1] + b[1]; out[2] = a[2] + b[2];
}

static inline void VectorScale(const vec3_t in, vec_t s, vec3_t out) {
    out[0] = in[0] * s; out[1] = in[1] * s; out[2] = in[2] * s;
}

// out = a + scale*b  (Quake's VectorMA)
static inline void VectorMA(const vec3_t a, vec_t scale, const vec3_t b, vec3_t out) {
    out[0] = a[0] + scale * b[0];
    out[1] = a[1] + scale * b[1];
    out[2] = a[2] + scale * b[2];
}

static inline void CrossProduct(const vec3_t a, const vec3_t b, vec3_t out) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static inline vec_t VectorLength(const vec3_t v) {
    return (vec_t)std::sqrt(DotProduct(v, v));
}
static inline vec_t Length(const vec3_t v) { return VectorLength(v); }

static inline void VectorClear(vec3_t v) { v[0] = v[1] = v[2] = 0.0f; }

// Normalize in place, returning the original length (Quake mathlib.cpp).
static inline vec_t VectorNormalize(vec3_t v) {
    vec_t length = (vec_t)std::sqrt(DotProduct(v, v));
    if (length != 0.0f) {
        vec_t ilength = 1.0f / length;
        v[0] *= ilength; v[1] *= ilength; v[2] *= ilength;
    }
    return length;
}

// forward/right/up from Euler angles (pitch,yaw,roll in degrees) - mathlib.cpp.
static inline void AngleVectors(const vec3_t angles, vec3_t forward, vec3_t right, vec3_t up) {
    const float d2r = 3.14159265358979323846f * 2.0f / 360.0f;
    float a, sr, sp, sy, cr, cp, cy;
    a = angles[1] * d2r; sy = std::sin(a); cy = std::cos(a);   // YAW
    a = angles[0] * d2r; sp = std::sin(a); cp = std::cos(a);   // PITCH
    a = angles[2] * d2r; sr = std::sin(a); cr = std::cos(a);   // ROLL
    if (forward) { forward[0] = cp * cy; forward[1] = cp * sy; forward[2] = -sp; }
    if (right)   { right[0] = -sr*sp*cy + cr*sy; right[1] = -sr*sp*sy - cr*cy; right[2] = -sr*cp; }
    if (up)      { up[0] = cr*sp*cy + sr*sy; up[1] = cr*sp*sy - sr*cy; up[2] = cr*cp; }
}
} // namespace pw
