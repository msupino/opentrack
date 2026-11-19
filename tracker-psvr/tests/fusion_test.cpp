// Synthetic test for psvr_fusion::Orientation. Not wired into the cmake
// build (otr_module's source glob is non-recursive). Build + run:
//
//   clang++ -std=c++20 -O1 -I .. fusion_test.cpp -o /tmp/ft && /tmp/ft
//
// Drives the filter with gyro/accel generated from a known orientation
// trajectory and checks the output angles.
#include "psvr_fusion.h"

#include <cmath>
#include <cstdio>
#include <functional>

using namespace psvr_fusion;

namespace {
constexpr double d2r = kPi / 180.0;
constexpr double kDt = 0.0005;   // 2 kHz, like the headset
int failures = 0;

void check(const char* name, double got, double want, double tol)
{
    const bool ok = std::fabs(got - want) <= tol;
    std::printf("%-44s got %8.3f want %8.3f  %s\n", name, got, want, ok ? "ok" : "FAIL");
    if (!ok) ++failures;
}

Quat conj(const Quat& q) { return {q.w, -q.x, -q.y, -q.z}; }

// Specific force a resting/moving IMU measures, in body frame:
// R^T * (up + linear_accel_world).
Vec3 body_accel(const Quat& q_true, Vec3 lin_world = {})
{
    return conj(q_true).rotate(Vec3{0, 1, 0} + lin_world);
}

// Body-frame angular velocity taking q0 to q1 over dt.
Vec3 body_rate(const Quat& q0, const Quat& q1, double dt)
{
    Quat d = (conj(q0) * q1).normalized();
    if (d.w < 0) d = {-d.w, -d.x, -d.y, -d.z};
    const double s = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (s < 1e-12) return {};
    const double angle = 2.0 * std::atan2(s, d.w);
    return Vec3{d.x, d.y, d.z} * (angle / s / dt);
}

// Run the filter along trajectory(t) for `seconds`; lin(t) adds linear
// acceleration (world frame, in g).
Orientation run(std::function<Quat(double)> traj, double seconds,
                std::function<Vec3(double)> lin = nullptr)
{
    Orientation f;
    f.reset_from_gravity(body_accel(traj(0)));
    const int n = int(seconds / kDt);
    for (int i = 0; i < n; ++i) {
        const double t0 = i * kDt, t1 = t0 + kDt;
        const Quat q1 = traj(t1);
        f.update(body_rate(traj(t0), q1, kDt),
                 body_accel(q1, lin ? lin(t1) : Vec3{}), kDt);
    }
    return f;
}

Quat yaw_q(double deg)   { return Quat::axis_angle({0, 1, 0}, -deg * d2r); } // right turn
Quat pitch_q(double deg) { return Quat::axis_angle({1, 0, 0},  deg * d2r); } // look up
Quat roll_q(double deg)  { return Quat::axis_angle({0, 0, -1}, deg * d2r); } // right ear down

// The old filter's accelerometer-only formulas (raw axis indices).
double old_acc_pitch(const double a[3]) { return std::atan2(a[2], std::sqrt(a[1]*a[1] + a[0]*a[0])) / d2r; }
double old_acc_roll(const double a[3])  { return -std::atan2(a[1], std::sqrt(a[2]*a[2] + a[0]*a[0])) / d2r; }
} // namespace

int main()
{
    // 1. Single-axis motions produce the documented signs.
    {
        auto f = run([](double t) { return yaw_q(90.0 * std::min(t, 1.0)); }, 1.5);
        check("yaw right 90", f.angles().yaw, 90, 0.5);
        check("  pitch stays 0", f.angles().pitch, 0, 0.5);
        check("  roll stays 0", f.angles().roll, 0, 0.5);
    }
    {
        auto f = run([](double t) { return pitch_q(30.0 * std::min(t, 1.0)); }, 1.5);
        check("pitch up 30", f.angles().pitch, 30, 0.5);
        check("  yaw stays 0", f.angles().yaw, 0, 0.5);
    }
    {
        auto f = run([](double t) { return roll_q(25.0 * std::min(t, 1.0)); }, 1.5);
        check("roll right 25", f.angles().roll, 25, 0.5);
        check("  yaw stays 0", f.angles().yaw, 0, 0.5);
    }

    // 2. Cross-axis: look down 40, then turn right 90 in the world frame.
    //    The Euler filter mixed body rates into the wrong angles here.
    {
        auto traj = [](double t) {
            const double p = -40.0 * std::min(t, 1.0);
            const double y = 90.0 * std::clamp(t - 1.0, 0.0, 1.0);
            return yaw_q(y) * pitch_q(p);
        };
        auto f = run(traj, 2.5);
        check("down 40 then right 90: yaw", f.angles().yaw, 90, 0.5);
        check("down 40 then right 90: pitch", f.angles().pitch, -40, 0.5);
        check("down 40 then right 90: roll", f.angles().roll, 0, 0.5);
    }

    // 3. Linear acceleration: head still and level, 0.3 g sideways push
    //    for 0.5 s. The old filter showed ~17 deg of false roll.
    {
        auto f = run([](double) { return Quat{}; }, 0.5,
                     [](double) { return Vec3{0.3, 0, 0}; });
        check("0.3 g sideways push: roll error < 5", std::fabs(f.angles().roll), 0, 5.0);
    }

    // 4. Drift correction: start with a 10 deg pitch error, head level.
    {
        Orientation f;
        f.reset_from_gravity(body_accel(pitch_q(10)));
        for (int i = 0; i < int(10.0 / kDt); ++i)
            f.update({}, body_accel(Quat{}), kDt);
        check("10 deg tilt error after 10 s", f.angles().pitch, 0, 0.1);
    }

    // 5. Static tilts agree in sign and size with the old accel formulas,
    //    so users see the same pitch/roll directions as before.
    for (auto [p, r] : {std::pair{20.0, 0.0}, {-15.0, 0.0}, {0.0, 20.0}, {0.0, -10.0}}) {
        const Vec3 fb = body_accel(pitch_q(p) * roll_q(r));
        // Body frame back to raw indices: fb = (a1, a0, -a2).
        const double a[3] = {fb.y, fb.x, -fb.z};
        Orientation f;
        f.reset_from_gravity(accel_body_from_raw(a));
        char name[64];
        std::snprintf(name, sizeof name, "static p=%g r=%g: pitch vs old", p, r);
        check(name, f.angles().pitch, old_acc_pitch(a), 0.5);
        std::snprintf(name, sizeof name, "static p=%g r=%g: roll vs old", p, r);
        check(name, f.angles().roll, old_acc_roll(a), 0.5);
    }

    // 6. Yaw sign matches the old filter: old yaw rate = -g[0] (deg/s).
    {
        const double g[3] = {-90.0, 0, 0};   // old filter: +90 deg/s yaw
        Orientation f;
        f.reset_from_gravity({0, 1, 0});
        for (int i = 0; i < int(1.0 / kDt); ++i)
            f.update(gyro_body_from_raw(g), {0, 1, 0}, kDt);
        check("raw g[0]=-90 dps for 1 s: yaw", f.angles().yaw, 90, 0.5);
    }

    std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
                failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
