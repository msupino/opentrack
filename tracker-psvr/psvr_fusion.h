/*
 * Quaternion IMU orientation filter for the PSVR tracker.
 *
 * Replaces the per-axis Euler complementary filter, which had two
 * problems:
 *   - It blended pitch/roll 2% toward the accelerometer on EVERY sample
 *     (~2 kHz), so pitch/roll were effectively raw accelerometer angles:
 *     any linear head acceleration leaked straight into them.
 *   - Adding body-frame gyro rates directly to yaw/pitch/roll is only
 *     correct while the head is level; turning while looking down
 *     produced cross-axis error.
 *
 * This filter integrates the gyro as a quaternion and pulls the "up"
 * axis gently toward measured gravity, only while the accelerometer
 * reads ~1 g and the head isn't turning fast. Approach adapted from
 * i33mr/psvr-mac (PSVRKit/OrientationTracker.swift).
 *
 * Body frame (shared by gyro and accel, per OpenHMD drv_psvr, Monado
 * drivers/psvr and psvr-mac): x right, y up, z toward the wearer's face
 * (forward is -z). From the raw HID sample triples:
 *     gyro  = ( g[1], g[0], -g[2] )
 *     accel = ( a[1], a[0], -a[2] )
 * World frame: y up, the forward heading at reset is -z.
 *
 * Header-only and free of Qt/IOKit so tests/fusion_test.cpp can build
 * it standalone.
 */
#pragma once

#include <algorithm>
#include <cmath>

namespace psvr_fusion {

constexpr double kPi = 3.14159265358979323846;

struct Vec3 {
    double x{0}, y{0}, z{0};
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator*(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(Vec3 a) { return std::sqrt(dot(a, a)); }

struct Quat {
    double w{1}, x{0}, y{0}, z{0};

    static Quat axis_angle(Vec3 unit_axis, double angle)
    {
        const double h = 0.5 * angle, s = std::sin(h);
        return {std::cos(h), unit_axis.x * s, unit_axis.y * s, unit_axis.z * s};
    }

    Quat operator*(const Quat& b) const
    {
        return {w * b.w - x * b.x - y * b.y - z * b.z,
                w * b.x + x * b.w + y * b.z - z * b.y,
                w * b.y - x * b.z + y * b.w + z * b.x,
                w * b.z + x * b.y - y * b.x + z * b.w};
    }

    Quat normalized() const
    {
        const double n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n < 1e-12) return {};
        return {w / n, x / n, y / n, z / n};
    }

    // Rotate v by this quaternion (q v q*).
    Vec3 rotate(Vec3 v) const
    {
        const Vec3 u{x, y, z};
        const Vec3 t = cross(u, v) * 2.0;
        return v + t * w + cross(u, t);
    }
};

// Shortest rotation taking unit vector `from` onto unit vector `to`.
inline Quat from_to(Vec3 from, Vec3 to)
{
    const double c = dot(from, to);
    if (c < -0.999999) {
        // Opposite vectors: rotate 180 degrees about any perpendicular axis.
        Vec3 axis = cross({1, 0, 0}, from);
        if (norm(axis) < 1e-6) axis = cross({0, 0, 1}, from);
        return Quat::axis_angle(axis * (1.0 / norm(axis)), kPi);
    }
    const Vec3 a = cross(from, to);
    return Quat{1.0 + c, a.x, a.y, a.z}.normalized();
}

// Head angles in degrees, in the tracker's opentrack conventions:
//   yaw   positive = head turned right
//   pitch positive = looking up
//   roll  positive = right ear toward the right shoulder
struct Angles {
    double yaw{0}, pitch{0}, roll{0};
};

class Orientation {
public:
    // Fraction of the tilt error removed per second.
    double gravity_gain = 0.5;
    // Gravity correction only runs while | |accel| - 1 g | is below this
    // and the head turns slower than kMaxCorrectRate.
    static constexpr double kAccelTolerance = 0.1;   // g
    static constexpr double kMaxCorrectRate = 1.0;   // rad/s

    // Start from the resting attitude: rotate so measured gravity points
    // up, then zero the heading. `accel_body` is in g, body frame.
    void reset_from_gravity(Vec3 accel_body)
    {
        const double n = norm(accel_body);
        q_ = (n > 0.5) ? from_to(accel_body * (1.0 / n), {0, 1, 0}) : Quat{};
        // Remove whatever heading from_to happened to introduce.
        const double h = heading_rad();
        q_ = (Quat::axis_angle({0, 1, 0}, -h) * q_).normalized();
    }

    // gyro_body in rad/s (bias already removed), accel_body in g, dt in s.
    void update(Vec3 gyro_body, Vec3 accel_body, double dt)
    {
        const double rate = norm(gyro_body);
        const double angle = rate * dt;
        if (angle > 1e-12)
            q_ = (q_ * Quat::axis_angle(gyro_body * (1.0 / rate), angle)).normalized();

        const double a = norm(accel_body);
        if (std::fabs(a - 1.0) < kAccelTolerance && rate < kMaxCorrectRate) {
            const Vec3 measured_up = q_.rotate(accel_body * (1.0 / a));
            const Vec3 axis = cross(measured_up, {0, 1, 0});
            const double s = norm(axis);
            if (s > 1e-9) {
                const double err = std::atan2(s, measured_up.y);
                const double step = err * std::min(gravity_gain * dt, 1.0);
                q_ = (Quat::axis_angle(axis * (1.0 / s), step) * q_).normalized();
            }
        }
    }

    Angles angles() const
    {
        const Vec3 fwd   = q_.rotate({0, 0, -1});
        const Vec3 right = q_.rotate({1, 0, 0});
        const Vec3 up    = q_.rotate({0, 1, 0});
        constexpr double r2d = 180.0 / kPi;
        Angles out;
        // heading_rad() is positive for a turn to the left (counter-
        // clockwise seen from above); opentrack's yaw is the other way.
        out.yaw   = -heading_rad() * r2d;
        out.pitch = std::atan2(fwd.y, std::sqrt(fwd.x * fwd.x + fwd.z * fwd.z)) * r2d;
        out.roll  = std::atan2(-right.y, up.y) * r2d;
        return out;
    }

    const Quat& quat() const { return q_; }

private:
    double heading_rad() const
    {
        const Vec3 fwd = q_.rotate({0, 0, -1});
        return std::atan2(-fwd.x, -fwd.z);
    }

    Quat q_{};
};

// Raw HID sample triples (gyro in deg/s after bias removal, accel in g)
// to the shared body frame above. Gyro output is rad/s.
inline Vec3 gyro_body_from_raw(const double g_dps[3])
{
    constexpr double d2r = kPi / 180.0;
    return {g_dps[1] * d2r, g_dps[0] * d2r, -g_dps[2] * d2r};
}

inline Vec3 accel_body_from_raw(const double a_g[3])
{
    return {a_g[1], a_g[0], -a_g[2]};
}

} // namespace psvr_fusion
