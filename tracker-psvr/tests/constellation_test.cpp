// Synthetic ground-truth test for the psvr_constellation solver. Not
// wired into the cmake build (otr_module's source glob is non-
// recursive, so this subdirectory is invisible to it). Build + run:
//
//   clang++ -std=c++20 -O1 -I .. $(pkg-config --cflags --libs opencv4) \
//       ../psvr_constellation.cpp constellation_test.cpp -o /tmp/ct && /tmp/ct
//
// Scenario: PSVR at known pose in front of a 640x400 (OV580-like)
// camera, HFOV 85, blob centroids perturbed by 0.7 px gaussian noise
// plus spurious room-light blobs. The IMU's yaw reference is offset
// 20 deg from the camera axis (user calibrated facing away from the
// camera) - the solver must disambiguate the planar mirror twin at
// first lock (the front/temple LEDs are symmetric under
// diag(-1,-1,1), so the twin reprojects identically; only the IMU can
// break the tie), calibrate the extrinsic away, and then track LIVE
// head rotation without leaking it into XYZ.
//
// Pre-fix behavior (frozen first-lock rotation): the rotation sweep
// either rejects every frame (HIGH_RMS) or reports a bogus XYZ
// excursion growing with head yaw. Post-fix: XYZ stays put under pure
// rotation and follows pure translation, ~1-2 cm worst error at the
// noise level above.
#include "psvr_constellation.h"
#include <opencv2/calib3d.hpp>
#include <cstdio>
#include <cmath>
#include <random>
#include <vector>

using namespace psvr_constellation;

// Mirror of the solver's kLEDModel (cm) and normals - keep in sync.
static const cv::Point3d LED[9] = {
    { 0.00,  0.00,   0.00}, { 7.25,  4.05,  3.75}, { 9.05, 0.00,  9.65},
    { 7.25, -4.05,   3.75}, {-7.25,  4.05,  3.75}, {-9.05, 0.00,  9.65},
    {-7.25, -4.05,   3.75}, { 5.65, -1.07, 27.53}, {-5.65, -1.07, 27.53},
};

// Same convention as the solver's head_to_camera_rotation().
static cv::Matx33d head_to_cam(double yaw, double pitch, double roll) {
    const double cy = std::cos(yaw),   sy = std::sin(yaw);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cr = std::cos(roll),  sr = std::sin(roll);
    const cv::Matx33d Ry( cy, 0, sy,  0, 1, 0,  -sy, 0, cy);
    const cv::Matx33d Rx(1, 0, 0,  0, cp, -sp,  0, sp, cp);
    const cv::Matx33d Rz(cr, -sr, 0,  sr, cr, 0,  0, 0, 1);
    const cv::Matx33d F(-1, 0, 0,  0, -1, 0,  0, 0, 1);
    return F * (Ry * Rx * Rz);
}

static cv::Matx33d intrinsics(int w, int h, double hfov_deg) {
    const double fx = 0.5 * w / std::tan(0.5 * hfov_deg * CV_PI / 180.0);
    return cv::Matx33d(fx, 0, 0.5 * w, 0, fx, 0.5 * h, 0, 0, 1);
}

// Project the model at (R_true, t_true); keep facing-camera, in-frame
// LEDs. Adds `n_spurious` fake room-light blobs. Shuffles order.
static std::vector<cv::Point2d> make_blobs(const cv::Matx33d& R,
                                           const cv::Vec3d& t,
                                           const cv::Matx33d& K,
                                           int w, int h, int n_spurious,
                                           std::mt19937& rng) {
    const cv::Vec3d C_head = -(R.t() * t);
    const cv::Vec3d head_centre(0, 0, 12);
    std::vector<cv::Point2d> out;
    for (int i = 0; i < 9; ++i) {
        const cv::Vec3d P(LED[i].x, LED[i].y, LED[i].z);
        cv::Vec3d n = P - head_centre;
        n /= std::sqrt(n.dot(n));
        if (n.dot(C_head - P) <= 0) continue;      // facing away
        const cv::Vec3d Q = R * P + t;
        if (Q(2) <= 1.0) continue;
        const double u = K(0,0) * Q(0) / Q(2) + K(0,2);
        const double v = K(1,1) * Q(1) / Q(2) + K(1,2);
        if (u < 0 || u >= w || v < 0 || v >= h) continue;
        std::normal_distribution<double> px_noise(0.0, 0.7);
        out.emplace_back(u + px_noise(rng), v + px_noise(rng));
    }
    std::uniform_real_distribution<double> ux(10.0, w - 10.0),
                                           uy(10.0, h - 10.0);
    for (int i = 0; i < n_spurious; ++i) out.emplace_back(ux(rng), uy(rng));
    std::shuffle(out.begin(), out.end(), rng);
    return out;
}

int main() {
    const int    W = 640, H = 400;
    const double HFOV = 85.0;
    const double D2R = CV_PI / 180.0;
    const double AZ_OFFSET = 20.0 * D2R;   // IMU yaw reference offset
    const cv::Matx33d K = intrinsics(W, H, HFOV);
    std::mt19937 rng(1234);

    SolverState st;
    int fails = 0;
    auto check = [&](bool ok, const char* what) {
        std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
        if (!ok) ++fails;
    };

    // ---- step 1: cold-start lock at fixed pose --------------------
    cv::Vec3d t_true(2.0, -4.0, 60.0);
    double imu_yaw = 0.0, imu_pitch = 0.0, imu_roll = 0.0;
    auto R_true = [&]() {
        return head_to_cam(imu_yaw + AZ_OFFSET, imu_pitch, imu_roll);
    };

    Result r{};
    int locked_at = -1;
    for (int f = 0; f < 8; ++f) {
        auto blobs = make_blobs(R_true(), t_true, K, W, H, 1, rng);
        r = st.solve(blobs, W, H, imu_yaw, imu_pitch, imu_roll, HFOV);
        if (r.ok && locked_at < 0) locked_at = f;
    }
    check(locked_at >= 0, "cold-start lock acquired (20 deg IMU azimuth offset)");
    const double e1 = std::hypot(r.x_cm - t_true(0),
                     std::hypot(r.y_cm - t_true(1), r.z_cm - t_true(2)));
    std::printf("      lock frame=%d  reason=%s  matched=%d  rms=%.2f  "
                "pos=(%.2f %.2f %.2f)  err=%.2f cm\n",
                locked_at, r.reject_reason, r.n_matched,
                r.reprojection_rms, r.x_cm, r.y_cm, r.z_cm, e1);
    check(r.ok && e1 < 2.0, "locked position within 2 cm of truth");

    // ---- step 2: pure head ROTATION, no translation ---------------
    // Old frozen-R code mis-fit this as translation or rejected all
    // frames. New code must keep publishing with XYZ unchanged.
    int ok_frames = 0; double worst = 0.0;
    for (int i = 1; i <= 15; ++i) {
        imu_yaw = i * 1.0 * D2R;          // sweep to 15 deg, 1 deg/frame
        auto blobs = make_blobs(R_true(), t_true, K, W, H, 1, rng);
        r = st.solve(blobs, W, H, imu_yaw, imu_pitch, imu_roll, HFOV);
        const double e = std::hypot(r.x_cm - t_true(0),
                    std::hypot(r.y_cm - t_true(1), r.z_cm - t_true(2)));
        std::printf("      [rot %2d deg] ok=%d reason=%-22s m=%d rms=%5.2f "
                    "pos=(%6.2f %6.2f %6.2f) err=%.2f\n",
                    i, r.ok, r.reject_reason, r.n_matched,
                    r.reprojection_rms, r.x_cm, r.y_cm, r.z_cm, e);
        if (r.ok) {
            ++ok_frames;
            worst = std::max(worst, e);
        }
    }
    std::printf("      rotation sweep: ok=%d/15  worst XYZ err=%.2f cm  "
                "last reason=%s\n", ok_frames, worst, r.reject_reason);
    check(ok_frames >= 13, "solver keeps accepting during head rotation");
    check(worst < 2.0, "XYZ stays put under pure rotation (< 2 cm leak)");

    // ---- step 3: pitch + roll too ---------------------------------
    ok_frames = 0; worst = 0.0;
    for (int i = 1; i <= 10; ++i) {
        imu_pitch = i * 0.8 * D2R;
        imu_roll  = i * 0.5 * D2R;
        auto blobs = make_blobs(R_true(), t_true, K, W, H, 1, rng);
        r = st.solve(blobs, W, H, imu_yaw, imu_pitch, imu_roll, HFOV);
        if (r.ok) {
            ++ok_frames;
            const double e = std::hypot(r.x_cm - t_true(0),
                        std::hypot(r.y_cm - t_true(1), r.z_cm - t_true(2)));
            worst = std::max(worst, e);
        }
    }
    std::printf("      pitch/roll sweep: ok=%d/10  worst err=%.2f cm\n",
                ok_frames, worst);
    check(ok_frames >= 8 && worst < 2.0, "pitch/roll don't leak into XYZ");

    // ---- step 4: pure TRANSLATION follows -------------------------
    ok_frames = 0; double worst_t = 0.0;
    for (int i = 1; i <= 10; ++i) {
        t_true(0) += 0.8;  t_true(2) -= 0.5;     // lean right + forward
        auto blobs = make_blobs(R_true(), t_true, K, W, H, 1, rng);
        r = st.solve(blobs, W, H, imu_yaw, imu_pitch, imu_roll, HFOV);
        if (r.ok) {
            ++ok_frames;
            const double e = std::hypot(r.x_cm - t_true(0),
                        std::hypot(r.y_cm - t_true(1), r.z_cm - t_true(2)));
            worst_t = std::max(worst_t, e);
        }
    }
    std::printf("      translation sweep: ok=%d/10  worst err=%.2f cm\n",
                ok_frames, worst_t);
    check(ok_frames >= 9 && worst_t < 2.0, "translation tracked accurately");

    // ---- step 5: rotation + translation combined ------------------
    ok_frames = 0; worst = 0.0;
    for (int i = 1; i <= 10; ++i) {
        imu_yaw  -= 1.5 * D2R;
        t_true(1) += 0.5;
        auto blobs = make_blobs(R_true(), t_true, K, W, H, 2, rng);
        r = st.solve(blobs, W, H, imu_yaw, imu_pitch, imu_roll, HFOV);
        if (r.ok) {
            ++ok_frames;
            const double e = std::hypot(r.x_cm - t_true(0),
                        std::hypot(r.y_cm - t_true(1), r.z_cm - t_true(2)));
            worst = std::max(worst, e);
        }
    }
    std::printf("      combined sweep: ok=%d/10  worst err=%.2f cm\n",
                ok_frames, worst);
    check(ok_frames >= 8 && worst < 2.5, "combined rotation+translation tracked");

    // ---- step 6: partial occlusion — only 3 LEDs visible while locked --
    // Regression guard for the inlier-floor fix. With rotation IMU-locked
    // the translation-only solve needs just 3 matched LEDs; the old
    // kMinInliers=4 floor rejected these frames as NO_AP3P_FIT (the exact
    // field symptom: "blobs=4-5, matched=3"). Establish a clean lock at a
    // steady pose first (so the prior is good, as in the field), then drop
    // to exactly 3 blobs with only small motion and require continued
    // accurate tracking.
    for (int i = 0; i < 4; ++i) {              // re-lock at current pose
        auto blobs = make_blobs(R_true(), t_true, K, W, H, 0, rng);
        r = st.solve(blobs, W, H, imu_yaw, imu_pitch, imu_roll, HFOV);
    }
    ok_frames = 0; worst = 0.0;
    int three_blob_frames = 0;
    for (int i = 0; i < 10; ++i) {
        t_true(0) += 0.15;                     // gentle drift, prior stays good
        auto blobs = make_blobs(R_true(), t_true, K, W, H, 0, rng);
        if (blobs.size() < 3) continue;
        // Keep the 3 brightest-equivalent (just the first 3 after the
        // solver-agnostic shuffle) to simulate occlusion of the rest.
        blobs.resize(3);
        ++three_blob_frames;
        r = st.solve(blobs, W, H, imu_yaw, imu_pitch, imu_roll, HFOV);
        if (r.ok) {
            ++ok_frames;
            const double e = std::hypot(r.x_cm - t_true(0),
                        std::hypot(r.y_cm - t_true(1), r.z_cm - t_true(2)));
            worst = std::max(worst, e);
        }
    }
    std::printf("      3-LED occlusion: ok=%d/%d  worst err=%.2f cm  "
                "last reason=%s\n", ok_frames, three_blob_frames, worst,
                r.reject_reason);
    check(three_blob_frames >= 8 && ok_frames >= three_blob_frames - 2 &&
          worst < 3.0,
          "keeps locked on only 3 visible LEDs (inlier-floor fix)");

    std::printf("\n%s (%d failure%s)\n", fails ? "TEST FAILED" : "ALL PASS",
                fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
