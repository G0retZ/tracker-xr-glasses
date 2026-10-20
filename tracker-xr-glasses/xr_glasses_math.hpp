/* Copyright (c) 2026, Sergei 'G0retZ' Mitrofanov

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#pragma once

// Small vector/quaternion toolkit plus the gyro+accelerometer complementary
// filter used for glasses that stream raw IMU data. The filter design (gravity
// alignment on start, stationary gyro-bias learning, accelerometer-based tilt
// correction, first-order quaternion integration) follows the one in the
// RayNeo-Air-Series-3DoF-Scaffold (Apache-2.0) as tuned in DroidDeck's
// RayNeo/XREAL adapters (GPL-3.0; parameters only, no code reused).

#include <cmath>

namespace xr_glasses {

struct vec3
{
    float x = 0, y = 0, z = 0;

    float length() const { return std::sqrt(x * x + y * y + z * z); }
    vec3 normalized() const
    {
        const float n = length();
        return n < 1e-6f ? vec3{} : vec3{ x / n, y / n, z / n };
    }
    vec3 cross(const vec3& o) const { return { y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x }; }
    vec3 operator+(const vec3& o) const { return { x + o.x, y + o.y, z + o.z }; }
    vec3 operator-(const vec3& o) const { return { x - o.x, y - o.y, z - o.z }; }
    vec3 operator*(float s) const { return { x * s, y * s, z * s }; }
    vec3& operator+=(const vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    vec3& operator-=(const vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    bool finite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
};

struct quat
{
    float w = 1, x = 0, y = 0, z = 0;

    quat operator*(const quat& o) const
    {
        return { w * o.w - x * o.x - y * o.y - z * o.z,
                 w * o.x + x * o.w + y * o.z - z * o.y,
                 w * o.y - x * o.z + y * o.w + z * o.x,
                 w * o.z + x * o.y - y * o.x + z * o.w };
    }
    quat operator*(float s) const { return { w * s, x * s, y * s, z * s }; }
    quat conjugate() const { return { w, -x, -y, -z }; }
    float norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }
    quat normalized() const
    {
        const float n = norm();
        return n < 1e-6f ? quat{} : quat{ w / n, x / n, y / n, z / n };
    }
    vec3 rotate(const vec3& v) const
    {
        const quat r = *this * quat{ 0, v.x, v.y, v.z } * conjugate();
        return { r.x, r.y, r.z };
    }
    // First-order integration of a body angular rate (rad/s) over dt seconds.
    quat integrate_body_rate(const vec3& omega, float dt) const
    {
        const quat qdot = (*this * quat{ 0, omega.x, omega.y, omega.z }) * 0.5f;
        return { w + qdot.w * dt, x + qdot.x * dt, y + qdot.y * dt, z + qdot.z * dt };
    }
    static quat from_axis_angle(const vec3& axis, float angle)
    {
        const vec3 a = axis.normalized();
        const float s = std::sin(angle * 0.5f), c = std::cos(angle * 0.5f);
        return quat{ c, a.x * s, a.y * s, a.z * s }.normalized();
    }
    bool finite() const { return std::isfinite(w) && std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
};

// Convert a body-to-world quaternion expressed in a right/up/back basis
// (OpenGL style: +X right, +Y up, +Z back) to the plugin's NWU basis
// (+X forward, +Y left, +Z up).
inline quat rub_to_nwu(const quat& q)
{
    return { q.w, -q.z, -q.x, q.y };
}

// ZYX Euler angles (degrees) of a body-to-world quaternion in the NWU basis:
// out[0] roll (+ = tilt right), out[1] pitch (+ = look down), out[2] yaw
// (+ = turn left).
inline void nwu_to_euler_deg(const quat& q, double out[3])
{
    constexpr double r2d = 180.0 / 3.14159265358979323846;
    const double w = q.w, x = q.x, y = q.y, z = q.z;
    out[0] = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * r2d;
    double sp = 2.0 * (w * y - z * x);
    sp = sp > 1 ? 1 : (sp < -1 ? -1 : sp);
    out[1] = std::asin(sp) * r2d;
    out[2] = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * r2d;
}

// Gyro + accelerometer complementary filter. Inputs are in a right/up/back
// body basis (gravity along -Y when level); the output quaternion maps body
// to world in that same basis.
class imu_filter
{
public:
    struct params
    {
        float accel_gain = 1.5f;          // gravity-correction strength
        float gravity_tolerance = 1.25f;  // |accel| - g allowed for "stationary"
        float stationary_gyro = 0.18f;    // rad/s below which gyro bias is learned
        float bias_rate = 0.5f;           // bias learning rate (1/s)
        float min_dt = 0.0001f;           // ignore samples closer than this
        float max_dt = 0.05f;             // treat larger gaps as a restart
        float default_dt = 0.01f;         // used when a gap is implausible instead of resetting
        bool reset_on_gap = true;         // XREAL: re-align on a gap; RayNeo: clamp dt instead
    };

    imu_filter() = default;
    explicit imu_filter(const params& cfg_) : cfg(cfg_) {}

    void reset() { initialized = false; bias = {}; }

    // accel in m/s^2, gyro in rad/s, both already in the right/up/back basis.
    // Returns false until gravity has been seen once.
    bool update(const vec3& accel_in, const vec3& gyro_in, float dt, quat& out)
    {
        const vec3 accel = accel_in;
        vec3 gyro = gyro_in;
        const float norm = accel.length();
        const bool gravity_ok = std::fabs(norm - 9.81f) < cfg.gravity_tolerance;

        if (cfg.reset_on_gap && dt > 0.25f)
            initialized = false;

        if (!initialized)
        {
            if (!gravity_ok)
                return false;
            align_to_gravity(accel);
            initialized = true;
            out = q;
            return true;
        }

        if (!cfg.reset_on_gap)
        {
            if (dt < 0.001f || dt > 0.1f)
                dt = cfg.default_dt;
        }
        else if (dt < cfg.min_dt || dt > cfg.max_dt)
        {
            out = q;
            return true;
        }

        if (gravity_ok && gyro.length() < cfg.stationary_gyro)
        {
            const float alpha = std::fmin(1.0f, dt * cfg.bias_rate);
            bias = bias * (1.0f - alpha) + gyro * alpha;
        }
        gyro -= bias;

        if (gravity_ok || (!cfg.reset_on_gap && norm > 1e-3f))
        {
            const vec3 predicted_up = q.conjugate().rotate(world_up()).normalized();
            gyro += accel.normalized().cross(predicted_up) * cfg.accel_gain;
        }

        q = q.integrate_body_rate(gyro, dt).normalized();
        out = q;
        return true;
    }

private:
    static vec3 world_up() { return { 0, 1, 0 }; }

    void align_to_gravity(const vec3& accel)
    {
        const vec3 up = accel.normalized();
        const vec3 axis = up.cross(world_up());
        q = up.y < -0.9999f ? quat::from_axis_angle({ 1, 0, 0 }, 3.14159265f)
                            : quat{ 1.0f + up.y, axis.x, axis.y, axis.z }.normalized();
    }

    params cfg;
    quat q;
    vec3 bias;
    bool initialized = false;
};

} // namespace xr_glasses
