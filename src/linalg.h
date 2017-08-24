#ifndef LINALG_H
#define LINALG_H

#include <math.h>
#include <float.h>

typedef struct {
    union {
        float v[3];
        struct {
            float x, y, z;
        };
    };
} Vec3;

typedef struct {
    union {
        float v[4];
        struct {
            float w, x, y, z;
        };
    };
} Quat;

typedef struct {
    union {
        float v[4*4];
        struct {
            float xx, xy, xz, xw;
            float yx, yy, yz, yw;
            float zx, zy, zz, zw;
            float wx, wy, wz, ww;
        };
    };
} Mat4;

static inline Vec3 vec3(float x, float y, float z) {
    return (Vec3){{{x, y, z}}};
}

static inline Quat quat(float w, float x, float y, float z) {
    return (Quat){{{w, x, y, z}}};
}

static inline Mat4 mat4(float xx, float xy, float xz, float xw,
                        float yx, float yy, float yz, float yw,
                        float zx, float zy, float zz, float zw,
                        float wx, float wy, float wz, float ww) {
    return (Mat4){{{
        xx, xy, xz, xw,
        yx, yy, yz, yw,
        zx, zy, zz, zw,
        wx, wy, wz, ww,
    }}};
}

static inline float vec_dot(Vec3 v1, Vec3 v2) {
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

static inline Vec3 vec_scale(Vec3 v, float s) {
    return vec3(v.x*s, v.y*s, v.z*s);
}

static inline Vec3 vec_add(Vec3 a, Vec3 b) {
    return vec3(a.x+b.x, a.y+b.y, a.z+b.z);
}

static inline float vec_len_sq(Vec3 v) {
    return v.x*v.x + v.y*v.y + v.z*v.z;
}

static inline float vec_len(Vec3 v) {
    return sqrtf(vec_len_sq(v));
}

static inline Vec3 vec_proj(Vec3 to, Vec3 from) {
    return vec_scale(to, vec_dot(to, from) / vec_len_sq(to));
}

static inline Vec3 vec_norm(Vec3 v) {
    return vec_scale(v, 1.0f / vec_len(v));
}

static inline Vec3 vec_neg(Vec3 v) {
    return vec_scale(v, -1);
}

static inline Vec3 vec_to_circular(Vec3 v) {
    float sum = fabsf(v.x) + fabsf(v.y) + fabsf(v.z);
    float len = vec_len(v);
    if (len < FLT_EPSILON) {
        return v;
    }
    return vec_scale(v, len/sum);
}

static inline Quat quat_neg(Quat q) {
    return quat(
        q.w, -q.x, -q.y, -q.z
    );
}

static inline Quat quat_from_rot(Vec3 rot) {
    float a = vec_len(rot);
    float c = cosf(a/2);
    float s = sinf(a/2);
    if (a < FLT_EPSILON) {
        return quat(1, 0, 0, 0);
    }
    return quat(
        c, s*rot.x/a, s*rot.y/a, s*rot.z/a
    );
}

static inline Quat quat_mul(Quat a, Quat b) {
    return quat(
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    );
}

static inline Mat4 quat_to_mat(Quat q) {
    return mat4(
        1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y-q.w*q.z),   2*(q.w*q.y+q.x*q.z),   0,
        2*(q.x*q.y+q.w*q.z),   1-2*(q.x*q.x+q.z*q.z), 2*(q.y*q.z - q.w*q.x), 0,
        2*(q.x*q.z-q.w*q.y),   2*(q.w*q.x+q.y*q.z),   1-2*(q.x*q.x+q.y*q.y), 0,
        0,                     0,                     0,                     1
    );
}

static inline Mat4 mat_identity(void) {
    return mat4(
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    );
}

static inline Mat4 mat_from_pos(Vec3 pos) {
    return mat4(
        1, 0, 0, pos.x,
        0, 1, 0, pos.y,
        0, 0, 1, pos.z,
        0, 0, 0, 1
    );
}

static inline Mat4 mat_from_scale(Vec3 scale) {
    return mat4(
        scale.x, 0, 0, 0,
        0, scale.y, 0, 0,
        0, 0, scale.z, 0,
        0, 0, 0, 1
    );
}

static inline Mat4 mat_from_persp(float fov, float ratio, float n, float f) {
    float a = 1.0f / tanf(fov / 2);
    return mat4(
        ratio * a, 0, 0, 0,
        0, a, 0, 0,
        0, 0, -(f + n) / (f - n), 2 * f * n / (n - f),
        0, 0, -1, 0
    );
}

static inline Vec3 mat_vec_mul(Mat4 l, Vec3 r) {
    return vec3(
        l.xx*r.x + l.xy*r.y + l.xz*r.z,
        l.yx*r.x + l.yy*r.y + l.yz*r.z,
        l.zx*r.x + l.zy*r.y + l.zz*r.z
    );
}

static inline Mat4 mat_mul(Mat4 l, Mat4 r) {
    return mat4(
        l.xx*r.xx + l.xy*r.yx + l.xz*r.zx + l.xw*r.wx,
        l.xx*r.xy + l.xy*r.yy + l.xz*r.zy + l.xw*r.wy,
        l.xx*r.xz + l.xy*r.yz + l.xz*r.zz + l.xw*r.wz,
        l.xx*r.xw + l.xy*r.yw + l.xz*r.zw + l.xw*r.ww,
        l.yx*r.xx + l.yy*r.yx + l.yz*r.zx + l.yw*r.wx,
        l.yx*r.xy + l.yy*r.yy + l.yz*r.zy + l.yw*r.wy,
        l.yx*r.xz + l.yy*r.yz + l.yz*r.zz + l.yw*r.wz,
        l.yx*r.xw + l.yy*r.yw + l.yz*r.zw + l.yw*r.ww,
        l.zx*r.xx + l.zy*r.yx + l.zz*r.zx + l.zw*r.wx,
        l.zx*r.xy + l.zy*r.yy + l.zz*r.zy + l.zw*r.wy,
        l.zx*r.xz + l.zy*r.yz + l.zz*r.zz + l.zw*r.wz,
        l.zx*r.xw + l.zy*r.yw + l.zz*r.zw + l.zw*r.ww,
        l.wx*r.xx + l.wy*r.yx + l.wz*r.zx + l.ww*r.wx,
        l.wx*r.xy + l.wy*r.yy + l.wz*r.zy + l.ww*r.wy,
        l.wx*r.xz + l.wy*r.yz + l.wz*r.zz + l.ww*r.wz,
        l.wx*r.xw + l.wy*r.yw + l.wz*r.zw + l.ww*r.ww
    );
}

#endif // LINALG_H
