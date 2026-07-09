/* PSVR LED constellation model + PnP solver.
 *
 * Pipeline per frame (called from psvr_camera.mm's AVFoundation capture
 * delegate at ~30 Hz):
 *
 *   1. Look up last accepted (rvec, tvec) as the extrinsic guess, or
 *      fall back to an "arms-length in front of camera" default if we
 *      don't have a fresh one (first frame, long gap, or previous
 *      rejection chain).
 *   2. Build camera intrinsics from image size and a default 70° HFOV
 *      pinhole model (no distortion). A Zhang-style calibration would
 *      get us sub-pixel accuracy but is out of scope - the downstream
 *      opentrack curves smooth anything left over.
 *   3. Project the 9 canonical LED positions through the last accepted
 *      camera pose when available, or through a rough IMU/default
 *      rotation at cold start, to get their expected image-space
 *      locations. LEDs that land behind the camera or outside the
 *      frame get dropped (they can't be matched).
 *   4. Establish blob<->LED correspondences: when a fresh prior exists,
 *      greedy nearest-neighbor within a (resolution-scaled) pixel gate
 *      around the prior-pose projections; otherwise a permutation
 *      search sampling random 4-blob/4-LED AP3P hypotheses, scored by
 *      reprojected inlier count. The AP3P rotations are hypothesis
 *      filters only and are discarded.
 *   5. FINAL solve: rotation is HELD to the live IMU rotation composed
 *      with the extrinsic calibrated at lock time (see step 7); only
 *      translation is fit, via a greedy-match/translation-refit ICP
 *      (solve_translation_fixed_rotation). Holding rotation removes
 *      the near-planar PnP two-fold twin that free-rotation solvePnP
 *      flips between. The user-visible yaw/pitch/roll still come from
 *      the PSVR IMU in psvr.cpp.
 *   6. Gates: reprojection RMS (resolution-scaled), Z sanity range,
 *      and a TIME-SCALED translation jump gate (max cm/s since the
 *      last accepted frame, with a per-frame floor) - guards against
 *      a blob burst from a lamp bouncing the matcher to a local-
 *      minimum pose without freezing the tracker after dropouts.
 *   7. On accept, cache (rvec, tvec, timestamp) AND the IMU rotation
 *      of this frame. On later frames the solve rotation is
 *      R_cached * R_imu_at_accept^T * R_imu_now, i.e. the optically
 *      locked extrinsic with the LIVE IMU delta applied, so head
 *      rotation moves the projected LEDs instead of being mis-fit as
 *      translation. Return x_cm/y_cm/z_cm verbatim in the OpenCV
 *      camera frame (+X right, +Y down, +Z into scene), in cm - the
 *      same convention tracker-aruco and tracker-pt use.
 *
 * Coordinate frame notes
 * ----------------------
 * Head frame (as used in kLEDModel below): +X = user's right, +Y = up,
 * +Z = back of head (right-handed; cross(+X, +Y) = +Z). Forward-where-
 * face-points is -Z. The model coordinates differ from PSMoveService's
 * original by a Z-sign flip - see the comment on kLEDModel - because
 * cv::solvePnP needs a right-handed model frame. The per-LED outward-
 * normal table (kLEDNormals) is derived from (P - head_centre) with
 * head_centre = (0, 0, 12) cm, i.e. ~12 cm behind the visor centre
 * (roughly between the ears), matching PSVRTracker's convention.
 *
 * When the user faces the camera with zero IMU rotation, the head-to-
 * camera flip is 180 deg around Z: user right (+X_head) becomes camera
 * left (-X_cam, because the camera sees the mirror image of the user),
 * user up (+Y_head) becomes camera-image up which in OpenCV is -Y_cam,
 * and +Z_head (back of head, away from camera) maps to +Z_cam (into
 * scene, also away from camera). So R_flip = diag(-1, -1, 1) which is
 * a proper rotation (det = +1).
 *
 * Why no constellation brute-force search
 * ---------------------------------------
 * The canonical problem - 9 near-identical blobs, several self-
 * occluded at any angle - normally requires an expensive search. We
 * dodge that mostly by carrying the previous accepted camera pose
 * forward as a prior (or using a "user-at-arm-length" default for cold
 * start). That prior is usually good enough that the sampled
 * correspondence search converges, and when it doesn't the jump/RMS
 * gates reject the frame so we don't latch a bad pose.
 */
#include "psvr_constellation.h"

#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mutex>

namespace psvr_constellation {

// Canonical 9-LED positions on the PSVR (units: cm), derived from
// PSMoveService/MorpheusHMD (@HipsterSloth, source:
// https://github.com/psmoveservice/PSMoveService/blob/master/src/psmoveservice/MorpheusHMD/MorpheusHMD.cpp#L625 ).
// The comment there notes they were "eyeballed with a ruler"; accuracy
// is ~5 mm, which is fine for a head-tracking application whose
// position output feeds a curve-smoothing filter downstream. A proper
// Zhang-style calibration would need a jig that captures the LEDs with
// a known-pose checkerboard visible in the same frame — out of scope
// for this plugin.
//
// Coordinate frame: +X = user's right, +Y = up, +Z = back of head
// (right-handed; cross(+X,+Y) = +Z). Forward-where-face-points is
// therefore -Z. Origin at approximate visor center.
//
// IMPORTANT: this is NOT verbatim PSMoveService - their model is
// stored in a left-handed frame (+Z forward) with rear LEDs at z=-24.
// Passing an LH-handed model to cv::solvePnP doesn't work: the
// rotation Jacobian assumes a proper RH rotation (det=+1), so the
// solver either fails outright or finds a "mirror" pose that lands
// the head behind the camera and trips the z-sanity gate. We
// therefore negate every Z coordinate vs. PSMoveService to obtain a
// RH-consistent geometry; the resulting (rvec, tvec) is then a
// proper rigid transform that the rest of the pipeline handles
// unmodified. The `head_to_camera_rotation` flip below (180° about
// Z) is also correct under this convention.
//
// LED index mapping:
//   0  center front (visor between eyebrows)
//   1  upper right front
//   2  right temple (side of visor)
//   3  lower right front
//   4  upper left front
//   5  left temple
//   6  lower left front
//   7  right rear (strap, near base of skull)
//   8  left rear  (strap, near base of skull)
// Numbers from PSVRTracker (HipsterSloth) MorpheusHMD.cpp's
// getTrackingShape(). Previously rounded approximations (off by ~10%
// on the front LEDs and ~13% on the rear strap) were good enough for
// blob detection but degraded PnP RMS.
static const std::array<cv::Point3d, NUM_LEDS> kLEDModel = {{
    { 0.00,  0.00,   0.00},
    { 7.25,  4.05,   3.75},
    { 9.05,  0.00,   9.65},
    { 7.25, -4.05,   3.75},
    {-7.25,  4.05,   3.75},
    {-9.05,  0.00,   9.65},
    {-7.25, -4.05,   3.75},
    { 5.65, -1.07,  27.53},
    {-5.65, -1.07,  27.53},
}};

// Per-LED outward normals. PSVRTracker's "cheezy" approximation:
// take a head geometric centre 12 cm behind the visor centre
// (roughly between the ears) and normalize (P_LED - head_centre).
// Empirically validated by the upstream tracker. LED 0 sits at the
// visor centre and yields (0, 0, -12) -> (0, 0, -1) after normalize,
// i.e. "facing forward", which is the right outward direction for
// the visor LED.
static const std::array<cv::Vec3d, NUM_LEDS> kLEDNormals = []{
    const cv::Vec3d head_centre(0.0, 0.0, 12.0);
    std::array<cv::Vec3d, NUM_LEDS> n{};
    for (size_t i = 0; i < NUM_LEDS; ++i) {
        cv::Vec3d v(kLEDModel[i].x - head_centre[0],
                    kLEDModel[i].y - head_centre[1],
                    kLEDModel[i].z - head_centre[2]);
        const double mag = std::sqrt(v.dot(v));
        n[i] = (mag > 1e-6) ? cv::Vec3d(v[0]/mag, v[1]/mag, v[2]/mag)
                            : cv::Vec3d(0, 0, -1);
    }
    return n;
}();

// Per-instance solver state; previously TU-globals (g_state_*, g_log_*)
// which entangled hypothetical multi-tracker setups and made test
// isolation impossible. One Impl lives inside each SolverState which
// in turn lives inside one psvr_cam::Worker, so the lifetime tracks
// the tracker session.
struct SolverState::Impl {
    std::mutex state_mu;
    bool       state_valid     = false;
    cv::Vec3d  state_rvec{0, 0, 0};
    cv::Vec3d  state_tvec{0, 0, 60.0};   // kDefaultUserZCm; literal here
                                         //   because constants live below
    // IMU rotation (as head_to_camera_rotation() Rodrigues) sampled on
    // the SAME frame state_rvec was accepted. Composing
    //   R = R_cached * R_imu_at_accept^T * R_imu_now
    // on later frames applies the live IMU rotation delta on top of
    // the optically locked pose, i.e. the lock calibrates the fixed
    // IMU-to-camera extrinsic (including the gyro's arbitrary yaw
    // reference) and the IMU supplies rotation from then on. Without
    // this the rotation stayed frozen at first lock and any real head
    // rotation was mis-fit as translation (or rejected as HIGH_RMS).
    cv::Vec3d  state_imu_rvec{0, 0, 0};
    double     state_epoch_sec = 0.0;
    bool       tentative_valid = false;
    cv::Vec3d  tentative_tvec{0, 0, 60.0};
    int        tentative_hits = 0;
    double     tentative_epoch_sec = 0.0;

    std::mutex log_mu;
    FILE*      log_fp           = nullptr;
    bool       log_attempted    = false;

    // Diagnostic counter: solve() increments unconditionally so the
    // per-LED facing-camera dot-product log line can be gated to
    // ~every 60 frames (about 2 s at 30 fps). Lives in Impl rather
    // than as a function-local static so concurrent SolverStates
    // (hypothetical multi-tracker setup) keep separate counts.
    uint64_t   diag_frame_count = 0;
};

namespace {

// Tuning constants. Collected here so all the numbers that determine
// "does the solver latch or not" are visible at a glance.
//
// kDefaultHFOVDeg is kept as a compile-time fallback used by the
// SolverState::solve() default parameter (see header). Today's
// production call path always passes the value through from
// psvr_settings::camera_hfov_deg via the Qt spinbox, but unit tests
// and any future caller that doesn't have a real HFOV available
// can still call solve() without specifying it. Marking it
// [[maybe_unused]] silences the "unused constant" warning that
// surfaces in builds that only exercise the explicit-HFOV path.
[[maybe_unused]] constexpr double kDefaultHFOVDeg        = 70.0; // typical laptop webcam
// All *_Px constants below are calibrated at 1920-wide frames and are
// scaled by (img_w / 1920) inside solve() before use: reprojection
// error in pixels is proportional to focal length, and the supported
// cameras span a 3x width range (OV580 solves at 640, webcams at up
// to 1920). Unscaled, the gates were 3x too loose on the OV580 path
// (bad fits passed the RMS gate; the prior gate greedily bound LEDs
// to the wrong blob).
constexpr double kPriorMatchDistPx      = 40.0; // locked prior projection to blob
// Inlier RMS threshold is set loose (30 px @ 1920x1080) because the
// LED constellation model is "eyeballed" from PSMoveService (~5 mm
// accuracy per LED) and the 70-deg HFOV intrinsic is approximate -
// both produce ~20-25 px systematic reprojection error even on
// genuinely correct correspondences. A downstream opentrack smoothing
// filter (EWMA / Accela) cleans up the frame-to-frame jitter.
constexpr double kMaxReprojectionRMSPx  = 30.0;
// Inlier gate for the upstream permutation-search step: a candidate
// pose is scored by counting how many of the 9 LEDs project within
// this many pixels of any blob. Was 30 px; tightened to 15 px after
// repeated observation that 30 was permissive enough for a 3-blob
// "spurious cluster of room lights" to score 3 inliers on a wholly
// fictitious pose, win the search, and lock the matcher onto
// nonsense geometry. At 15 px (about 1 LED footprint at our typical
// range) the matcher demands actual spatial coincidence between the
// reprojected LED and a real blob.
constexpr double kPermSearchInlierPx    = 15.0;
// Jump gate, TIME-SCALED. A real head moves at most ~1 m/s
// comfortably; the allowed translation delta since the last ACCEPTED
// frame is kMaxCmPerSec * elapsed, floored at kMaxCmPerFrameFloor for
// back-to-back 30 Hz frames. The floor preserves the original 3 cm/
// frame behavior that rejects the matcher oscillating between two
// near-symmetric LED-to-blob assignments 4-5 cm apart (both with
// reasonable RMS). The time scaling fixes the stuck-tracker failure
// the flat gate created: after a multi-frame dropout (occlusion, blob
// loss) while the user leans, the true pose on reacquire is >3 cm
// from the stale prior, and a flat gate rejected EVERY frame until
// the 2 s staleness reset - the tracker froze for up to 2 s. Scaled,
// a 1 s dropout allows 100 cm, so reacquire is immediate.
constexpr double kMaxCmPerSec           = 100.0;
constexpr double kMaxCmPerFrameFloor    = 3.0;
// The camera-pose rvec is only an internal optical matching prior;
// user-visible yaw/pitch/roll still come from the PSVR IMU. With the
// near-planar five-front-LED view, free PnP can find mirror-rotation
// twins whose translation is close enough to pass the cm jump gate but
// whose projected LED layout flips across the image. Rejecting large
// per-frame camera-rvec changes keeps the matcher on one optical
// branch while still allowing very fast real head motion (20 deg at
// 30 Hz = 600 deg/s).
constexpr double kMaxCameraRotDegPerFrame = 20.0;
// Minimum inliers for the COLD-START correspondence path, where
// rotation is recovered optically by AP3P (which structurally needs 4
// points). Used for the permutation search, its NO_AP3P_FIT gate, and
// the cold-start ICP.
constexpr int    kMinInliers            = 4;
// Minimum inliers for the LOCKED path (have_prior), where rotation is
// FIXED to the IMU-composed R and only the 3-DOF translation is fit.
// AP3P's 4-point requirement does not apply here: 2 LEDs already give
// 4 equations for 3 unknowns. We require 3 (6 equations) so the system
// stays over-determined enough that the reprojection-RMS gate remains
// a meaningful check - with only 2 points the fit is exact and RMS
// can't validate it. This is the direct fix for the observed
// "blobs=4-5, matched=3 -> NO_AP3P_FIT" dropouts: those frames are
// solvable with rotation already known, and the RMS / Z / time-scaled
// jump gates still guard correctness.
constexpr int    kMinInliersLocked      = 3;
// Stricter inlier count required for the FIRST-publish lock when
// there's no fresh prior (cold start, or >1 s since last accepted
// frame). The jump gate can't catch a false first-lock - there's
// nothing to jump from - so the only defense is to demand the
// matcher identify a substantial majority of the 9 LEDs before
// publishing any position. Lowered from 6 to 5 because at most camera
// angles only 5-7 of the 9 LEDs are physically visible (the rear strap
// pair filtered out, plus 0-2 more occluded by helmet pitch/yaw). At
// 6 we silently rejected every match for typical user poses where the
// helmet exposes 5 LEDs. 5 still rules out the "3-blob coincidence"
// failure mode (4 inliers would let it through; we keep 5 as a safety
// margin) without being unreachable in practice.
constexpr int    kStrongLockMinInliers  = 5;
constexpr double kFirstLock4LedMaxRMSPx = 16.0;
constexpr double kFirstLock4LedMaxCm    = 4.0;
constexpr int    kFirstLock4LedHits     = 3;
constexpr double kFirstLockTentativeSec = 0.75;
constexpr double kStalenessResetSec     = 2.0;  // prior expires after
constexpr double kDefaultUserZCm        = 60.0; // cold-start Z guess
// Z sanity bounds. Was [20, 200] cm. Tightened to [30, 120]: a
// head closer than 30 cm to a desktop webcam is physically jammed
// against the screen; farther than 120 cm is across the room from
// any plausible desk/sim-pit setup. The 178 cm false-locks seen in
// the field landed inside the old [20, 200] window and weren't
// rejected; the new ceiling cuts them.
constexpr double kMinAcceptableZCm      = 30.0;
constexpr double kMaxAcceptableZCm      = 120.0;

// Lazy-open the per-instance debug log on first solve() call.
//
// Logging is OFF BY DEFAULT to match the rest of tracker-psvr's
// "no implicit disk writes" policy (see psvr_settings::enable_diag_log,
// also off by default). Two ways to turn it on:
//
//   * Set the PSVR_CONSTELLATION_LOG env var to a writable path before
//     launching opentrack. Useful for ad-hoc debugging / scripting.
//
//   * Tick the "Write diagnostic log" checkbox in the PSVR tracker
//     settings; PSVRTracker::start_tracker setenv()s
//     PSVR_CONSTELLATION_LOG to /tmp/psvr-constellation.log so the
//     IMU diag log and constellation log are gated by a single
//     user-visible toggle.
//
// Holds impl.log_mu only during the open; subsequent calls just
// return impl.log_fp directly.
FILE* debug_log_fp(SolverState::Impl& impl) {
    std::lock_guard<std::mutex> lk(impl.log_mu);
    if (impl.log_attempted) return impl.log_fp;
    impl.log_attempted = true;
    const char* env = std::getenv("PSVR_CONSTELLATION_LOG");
    if (!env || !*env)
        return nullptr;  // off by default unless explicitly opted in
    impl.log_fp = std::fopen(env, "w");
    if (impl.log_fp) {
        std::fprintf(impl.log_fp,
            "# PSVR constellation solver debug. One line per solve() call.\n"
            "# Fields (space-separated):\n"
            "#   t_sec       time since first log line\n"
            "#   ypr_deg     IMU yaw pitch roll in deg\n"
            "#   guess_xyz   prior tvec guess (cm, OpenCV frame)\n"
            "#   n_blobs     detected blob count\n"
            "#   n_visible   LEDs projected on-screen at guess pose\n"
            "#   n_matched   blobs matched to LEDs\n"
            "#   pnp_ok rms  PnP success + reprojection RMS (px)\n"
            "#   tvec        solved tvec (cm, OpenCV frame)\n"
            "#   proj_xy     [led_idx u v]*9 projected positions at guess\n"
            "#   blob_xy     [u v]*n_blobs detected blob positions\n"
            "#   outcome     ACCEPT | REJECT_<reason>\n");
        std::fflush(impl.log_fp);
    }
    return impl.log_fp;
}

double steady_now_sec() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

// Solve camera translation t for a FIXED rotation R, minimizing the
// reprojection error over the given 3D-2D correspondences. Damped
// Gauss-Newton over the 3 translation unknowns (R is held to the IMU
// rotation by the caller). With rotation removed as a free parameter the
// near-planar PSVR front-LED constellation no longer has the two-fold PnP
// twin that free-rotation solvePnP flips between - the source of the
// "XYZ jumps around uncorrelated to head motion" symptom. t is seeded by
// the caller (free-PnP tvec or last prior) and updated in place. Returns
// false if a point falls behind the camera or the normal equations are
// singular, which the caller treats as a diverged translation solve.
bool solve_translation_fixed_rotation(
        const cv::Matx33d& R, const cv::Matx33d& K,
        const std::vector<cv::Point3d>& obj,
        const std::vector<cv::Point2d>& img,
        cv::Vec3d& t) {
    const double fx = K(0, 0), fy = K(1, 1);
    const double cx = K(0, 2), cy = K(1, 2);
    for (int iter = 0; iter < 25; ++iter) {
        cv::Matx33d H = cv::Matx33d::zeros();
        cv::Vec3d   g(0, 0, 0);
        for (size_t i = 0; i < obj.size(); ++i) {
            const cv::Vec3d P(obj[i].x, obj[i].y, obj[i].z);
            const cv::Vec3d Q = R * P + t;
            if (Q(2) <= 1e-3) return false;          // behind camera
            const double iz = 1.0 / Q(2);
            const double rx = (fx * Q(0) * iz + cx) - img[i].x;
            const double ry = (fy * Q(1) * iz + cy) - img[i].y;
            // d(proj)/dt rows (projection depends on t only through Q=RP+t,
            // and dQ/dt = I).
            const cv::Vec3d Jx(fx * iz, 0.0,      -fx * Q(0) * iz * iz);
            const cv::Vec3d Jy(0.0,     fy * iz,  -fy * Q(1) * iz * iz);
            for (int a = 0; a < 3; ++a) {
                g(a) += Jx(a) * rx + Jy(a) * ry;
                for (int b = 0; b < 3; ++b)
                    H(a, b) += Jx(a) * Jx(b) + Jy(a) * Jy(b);
            }
        }
        // Tiny Levenberg damping keeps the 3x3 well-conditioned.
        for (int a = 0; a < 3; ++a) H(a, a) += 1e-6 * H(a, a) + 1e-9;
        const cv::Matx33d Hinv = H.inv(cv::DECOMP_CHOLESKY);
        const cv::Vec3d   dt   = -(Hinv * g);
        if (!std::isfinite(dt(0)) || !std::isfinite(dt(1)) ||
            !std::isfinite(dt(2)))
            return false;
        t += dt;
        if (dt.dot(dt) < 1e-8) break;                // converged (<~0.1mm)
    }
    return true;
}

// Pinhole intrinsics from a horizontal FOV. Square pixels, principal
// point centered. Good enough for a first-pass solver.
cv::Matx33d make_intrinsics(int w, int h, double hfov_deg) {
    const double hfov_rad = hfov_deg * CV_PI / 180.0;
    const double fx = 0.5 * w / std::tan(0.5 * hfov_rad);
    const double fy = fx;
    return cv::Matx33d(fx, 0.0, 0.5 * w,
                       0.0, fy, 0.5 * h,
                       0.0, 0.0, 1.0);
}

// Compose IMU yaw/pitch/roll (head body frame) + the canonical head-to
// -camera flip (180 deg about Z) into a single rotation matrix that
// maps head-frame vectors to OpenCV-camera-frame vectors, before
// applying translation. Rotation order matches psvr.cpp's complementary
// filter output convention (Y then X then Z).
cv::Matx33d head_to_camera_rotation(double yaw, double pitch, double roll) {
    const double cy = std::cos(yaw),   sy = std::sin(yaw);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cr = std::cos(roll),  sr = std::sin(roll);
    const cv::Matx33d Ry( cy, 0.0, sy,
                         0.0, 1.0, 0.0,
                         -sy, 0.0, cy);
    const cv::Matx33d Rx(1.0, 0.0, 0.0,
                         0.0,  cp, -sp,
                         0.0,  sp,  cp);
    const cv::Matx33d Rz( cr, -sr, 0.0,
                          sr,  cr, 0.0,
                         0.0, 0.0, 1.0);
    // User-facing-camera default: +X_head -> -X_cam (user's right is
    // camera's image-left), +Y_head -> -Y_cam (camera image-Y points
    // down in OpenCV, so head-up flips), +Z_head -> +Z_cam (head-back
    // and camera-into-scene both point away from camera). That's
    // diag(-1, -1, 1), a 180 deg rotation about Z (det = +1, proper
    // rotation). Note the model's Z handedness (see kLEDModel) was
    // chosen to make this flip work as a proper rotation - the
    // PSMoveService original used LH coords and would not.
    const cv::Matx33d R_flip(-1.0, 0.0, 0.0,
                              0.0, -1.0, 0.0,
                              0.0, 0.0, 1.0);
    return R_flip * (Ry * Rx * Rz);
}

double rotation_delta_deg(const cv::Vec3d& a_rvec, const cv::Vec3d& b_rvec) {
    cv::Mat A, B;
    cv::Rodrigues(a_rvec, A);
    cv::Rodrigues(b_rvec, B);
    if (A.rows != 3 || A.cols != 3 || B.rows != 3 || B.cols != 3)
        return 180.0;

    const cv::Mat D = A * B.t();
    const double tr =
        D.at<double>(0, 0) + D.at<double>(1, 1) + D.at<double>(2, 2);
    const double c = std::max(-1.0, std::min(1.0, (tr - 1.0) * 0.5));
    return std::acos(c) * 180.0 / CV_PI;
}

} // anonymous namespace

// Helper: write a debug-log line with all the solver-input + outcome
// details. Cheap string-building here is fine; this only runs when the
// log file is open (env var set) which is only intended for debugging.
static void log_frame(FILE* fp,
                      double yaw_rad, double pitch_rad, double roll_rad,
                      const cv::Vec3d& prior_tvec,
                      const std::vector<cv::Point2d>& blobs,
                      const std::array<cv::Point2d, NUM_LEDS>& projected,
                      const std::array<bool, NUM_LEDS>&        visible,
                      int n_matched, bool pnp_ok, double rms,
                      const cv::Vec3d& tvec, const char* outcome)
{
    if (!fp) return;
    static double t0 = 0;
    if (t0 == 0) t0 = steady_now_sec();
    const double t = steady_now_sec() - t0;
    const double r2d = 180.0 / CV_PI;
    int n_visible = 0;
    for (int i = 0; i < NUM_LEDS; ++i) if (visible[i]) ++n_visible;
    std::fprintf(fp,
        "%7.3f ypr=[%+6.1f %+6.1f %+6.1f] guess=[%+5.1f %+5.1f %+5.1f] "
        "n_blobs=%d n_vis=%d n_matched=%d pnp=%d rms=%5.2f "
        "tvec=[%+5.1f %+5.1f %+5.1f] %s proj=[",
        t,
        yaw_rad * r2d, pitch_rad * r2d, roll_rad * r2d,
        prior_tvec(0), prior_tvec(1), prior_tvec(2),
        (int)blobs.size(), n_visible, n_matched, pnp_ok ? 1 : 0, rms,
        tvec(0), tvec(1), tvec(2), outcome);
    for (int i = 0; i < NUM_LEDS; ++i) {
        if (!visible[i]) continue;
        std::fprintf(fp, "%d:(%.0f,%.0f) ", i, projected[i].x, projected[i].y);
    }
    std::fprintf(fp, "] blobs=[");
    for (const auto& b : blobs) std::fprintf(fp, "(%.0f,%.0f) ", b.x, b.y);
    std::fprintf(fp, "]\n");
    std::fflush(fp);
}

SolverState::SolverState() : impl_(std::make_unique<Impl>()) {}

SolverState::~SolverState() {
    if (impl_->log_fp) {
        std::fclose(impl_->log_fp);
        impl_->log_fp = nullptr;
    }
}

Result SolverState::solve(const std::vector<cv::Point2d>& blobs,
                          int img_w, int img_h,
                          double yaw_rad, double pitch_rad, double roll_rad,
                          double hfov_deg)
{
    Impl& s = *impl_;
    Result r;
    r.n_blobs_total = (int)blobs.size();
    FILE* dbg = debug_log_fp(s);
    const uint64_t this_frame = s.diag_frame_count++;

    // Snapshot prior state under the mutex, then release before the
    // OpenCV work. The state is only a handful of doubles so the copy
    // is cheap; we never hold the mutex across solvePnP.
    //
    // Translation freshness gating: only reuse the last-accepted tvec
    // if it's fresh (< kStalenessResetSec). Otherwise the user may
    // have moved while we weren't tracking, and the cached value is
    // worse than the cold-start "user at arm's length, centered"
    // assumption further down.
    //
    // Rotation: LIVE IMU rotation composed with the extrinsic
    // calibrated at lock time.
    //
    // The raw IMU yaw is referenced to wherever the gyro integration
    // started, not to the camera, and the body-frame convention of
    // head_to_camera_rotation() is only approximate - so the IMU
    // rotation alone can't be locked against directly (that's what an
    // earlier revision tried; it fell into HIGH_RMS constantly). The
    // previous workaround reused the CACHED camera rvec verbatim,
    // which froze rotation at the first-lock orientation: any real
    // head rotation then swept the LEDs across the image with R held
    // fixed, and the translation-only solve explained the sweep as a
    // bogus XYZ excursion (small rotations) or blew the RMS gate and
    // froze XYZ (large ones).
    //
    // Fix: on every accept we store BOTH the accepted camera rotation
    // and the IMU rotation of that same frame (state_imu_rvec). On
    // later frames,
    //     R = R_cached * R_imu_at_accept^T * R_imu_now
    // i.e. the frozen extrinsic error (yaw reference, convention
    // mismatch) is calibrated away by the lock, while the IMU's
    // rotation DELTA since the lock tracks live head rotation. When
    // the IMU is not streaming (ypr constant), the composition
    // degenerates to R_cached exactly - the legacy behavior. Without
    // a camera prior, fall back to the IMU/default rotation as a
    // cold-start guess.
    //
    // NOTE: hoisted above the kMinInliers early-return so the per-LED
    // facing-camera dot-product diagnostic below has a prior_tvec to
    // work with even on frames where the extractor delivered too few
    // blobs to run PnP. That gating fail is exactly when we most want
    // visibility into "would the matcher have seen anything anyway?".
    cv::Vec3d prior_rvec_cached;
    cv::Vec3d prior_imu_rvec_cached;
    cv::Vec3d prior_tvec;
    bool      have_prior;
    double    prior_epoch_sec = 0.0;
    {
        std::lock_guard<std::mutex> lk(s.state_mu);
        const bool fresh = s.state_valid &&
                           (steady_now_sec() - s.state_epoch_sec) < kStalenessResetSec;
        have_prior = fresh;
        prior_rvec_cached = s.state_rvec;
        prior_imu_rvec_cached = s.state_imu_rvec;
        prior_tvec = have_prior ? s.state_tvec : cv::Vec3d(0, 0, kDefaultUserZCm);
        prior_epoch_sec = s.state_epoch_sec;
    }

    // Inlier floor for this frame: the relaxed locked floor when rotation
    // is IMU-fixed (translation-only solve), the strict AP3P floor at
    // cold start. Threaded through the blob-count gate, prior-match
    // acceptance, correspondence-result gate, and the ICP.
    const int min_inliers = have_prior ? kMinInliersLocked : kMinInliers;

    const cv::Matx33d R_imu_now =
        head_to_camera_rotation(yaw_rad, pitch_rad, roll_rad);
    cv::Vec3d imu_rvec_now;
    {
        cv::Mat tmp(3, 3, CV_64F);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                tmp.at<double>(i, j) = R_imu_now(i, j);
        cv::Rodrigues(tmp, imu_rvec_now);
    }

    cv::Matx33d R = R_imu_now;
    if (have_prior) {
        cv::Mat R_cached_m, R_imu_acc_m;
        cv::Rodrigues(prior_rvec_cached, R_cached_m);
        cv::Rodrigues(prior_imu_rvec_cached, R_imu_acc_m);
        if (R_cached_m.rows == 3 && R_cached_m.cols == 3 &&
            R_imu_acc_m.rows == 3 && R_imu_acc_m.cols == 3) {
            cv::Matx33d R_cached, R_imu_acc;
            for (int r_i = 0; r_i < 3; ++r_i)
                for (int c_i = 0; c_i < 3; ++c_i) {
                    R_cached(r_i, c_i)  = R_cached_m.at<double>(r_i, c_i);
                    R_imu_acc(r_i, c_i) = R_imu_acc_m.at<double>(r_i, c_i);
                }
            R = R_cached * R_imu_acc.t() * R_imu_now;
        }
    }

    // Per-LED facing-camera dot-product diagnostic, every 60 solves
    // (~2 s at 30 fps). Computes the same quantity the visibility
    // filter further down rejects on: dot(normal_i, C_head - P_i).
    // Positive means the LED faces the camera at the current pose
    // prior; negative means it points away (helmet self-occluded).
    //
    // Behavior unchanged - this only prints. Runs BEFORE the
    // blobs<kMinInliers early-return because that's the symptom we
    // want to debug: when vis=0 is being reported in the periodic
    // [psvr-cam] log, this line tells us whether vis=0 is "all LEDs
    // legitimately self-occluded by the current IMU pose" or "we
    // bailed out before reaching the visibility filter at all".
    //
    // prior_tvec is whatever the solver itself would use this frame
    // - either the cached last-accepted translation (have_prior=
    // true), or the cold-start fallback (0, 0, kDefaultUserZCm). The
    // cold-start centroid back-projection refinement happens further
    // down and never runs when blobs<kMinInliers, so the diagnostic
    // legitimately reflects what the solver was looking at.
    if ((this_frame % 60) == 0) {
        const cv::Vec3d   C_dbg = -(R.t() * prior_tvec);
        char buf[320];
        int  off = std::snprintf(buf, sizeof buf,
            "[psvr-cam] vis-detail: prior=%s tvec=(%+.1f,%+.1f,%+.1f) "
            "C_head=(%+.1f,%+.1f,%+.1f) dots=",
            have_prior ? "fresh" : "cold",
            prior_tvec(0), prior_tvec(1), prior_tvec(2),
            C_dbg(0), C_dbg(1), C_dbg(2));
        for (int i = 0; i < NUM_LEDS && off < (int)sizeof buf; ++i) {
            const cv::Vec3d to_cam = C_dbg - cv::Vec3d(
                kLEDModel[i].x, kLEDModel[i].y, kLEDModel[i].z);
            const double d = kLEDNormals[i].dot(to_cam);
            off += std::snprintf(buf + off, sizeof buf - off,
                                 " [%d]%+.1f", i, d);
        }
        std::fprintf(stderr, "%s\n", buf);
    }

    if ((int)blobs.size() < min_inliers) {
        r.reject_reason = "TOO_FEW_BLOBS";
        if (dbg) {
            std::array<cv::Point2d, NUM_LEDS> empty_proj{};
            std::array<bool, NUM_LEDS>        empty_vis{};
            log_frame(dbg, yaw_rad, pitch_rad, roll_rad, cv::Vec3d(0, 0, 0),
                      blobs, empty_proj, empty_vis,
                      0, false, 0, cv::Vec3d(0, 0, 0), "REJECT_TOO_FEW_BLOBS");
        }
        return r;
    }

    // Camera intrinsics from the user-configured HFOV. Was a TU-
    // constant kDefaultHFOVDeg=70 here; now plumbed from the dialog's
    // Camera HFOV spinbox via psvr_cam::Worker::set_hfov_deg so the
    // solver matches whatever webcam is actually mounted. solve()'s
    // default-parameter value (70 deg) preserves the legacy behavior
    // for callers that don't supply the HFOV (tests, etc.).
    const cv::Matx33d K = make_intrinsics(img_w, img_h, hfov_deg);

    // Resolution scaling for all pixel-space gates (see the comment at
    // the constants block). Calibrated at 1920-wide; clamped so a
    // pathological tiny/huge frame can't collapse or balloon the gates.
    const double px_scale = std::clamp(img_w / 1920.0, 1.0 / 3.0, 1.5);
    const double prior_match_px    = kPriorMatchDistPx     * px_scale;
    const double perm_inlier_px    = kPermSearchInlierPx   * px_scale;
    const double max_rms_px        = kMaxReprojectionRMSPx * px_scale;
    const double firstlock_rms_px  = kFirstLock4LedMaxRMSPx * px_scale;

    // Time-scaled translation jump allowance since the last ACCEPTED
    // frame. Rejected frames never bump state_epoch_sec, so after an
    // N-second reject/dropout burst the gate opens to N * 100 cm and
    // reacquire isn't fenced off by the stale prior. Floored so back-
    // to-back 30 Hz frames keep the original 3 cm behavior.
    const double dt_since_accept = have_prior
        ? std::max(0.0, steady_now_sec() - prior_epoch_sec) : 0.0;
    const double jump_allow_cm =
        std::max(kMaxCmPerFrameFloor, kMaxCmPerSec * dt_since_accept);

    // Build the IMU rotation prior as a Rodrigues vector. R already
    // expresses the head-to-camera transform that the visibility
    // filter and projectPoints use directly, so prior_rvec just makes
    // that same rotation available to the AP3P/RANSAC seed paths in
    // the conventional (rvec, tvec) form. cv::Rodrigues round-trips
    // any proper rotation; at yaw=pitch=roll=0 it produces (0, 0, pi)
    // which is the head-to-camera 180-deg flip about Z encoded in
    // R_flip (see head_to_camera_rotation comments). Solver behavior
    // at the test case yaw=pitch=roll=0 is unchanged: AP3P inside the
    // permutation-RANSAC loop runs with useExtrinsicGuess=false (it
    // overwrites trial_rvec from scratch), and the downstream
    // solvePnPRansac is seeded with best_rvec from the AP3P winner,
    // not prior_rvec. Logging and any future direct-projection-seed
    // path will see the correct value.
    cv::Mat R_mat(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R_mat.at<double>(i, j) = R(i, j);
    cv::Vec3d prior_rvec;
    cv::Rodrigues(R_mat, prior_rvec);

    // Cold-start improvement: if we don't have a fresh prior, derive
    // an initial (x, y) translation from the blob centroid instead of
    // assuming the user sits dead center in front of the camera. The
    // MBP lid camera, desktop webcams mounted off-axis, and anyone
    // not sitting precisely in front of the sensor would otherwise
    // put real LEDs hundreds of pixels from where the solver projects
    // them, and the 80 px match gate rejects every candidate. Back-
    // projecting the centroid to kDefaultUserZCm depth gives a
    // starting (x, y) within the match gate's radius of reality.
    if (!have_prior) {
        double cu = 0.0, cv_ = 0.0;
        for (const auto& b : blobs) { cu += b.x; cv_ += b.y; }
        cu /= blobs.size();
        cv_ /= blobs.size();
        const double fx = K(0, 0), fy = K(1, 1);
        const double cx = K(0, 2), cy = K(1, 2);
        prior_tvec(0) = (cu - cx) * kDefaultUserZCm / fx;
        prior_tvec(1) = (cv_ - cy) * kDefaultUserZCm / fy;
        prior_tvec(2) = kDefaultUserZCm;
    }

    // Camera position in HEAD frame: C = -R^T * t. Used for the
    // facing-camera visibility filter below. Doing the dot product
    // in head frame (rather than camera frame) avoids transforming a
    // separate "outward normal" vector per LED: for a roughly
    // spherical helmet the LED's outward normal is approximately its
    // position vector from the head origin, so "LED faces camera"
    // reduces to C_head . P_head > P_head . P_head.
    const cv::Matx33d R_T = R.t();
    const cv::Vec3d   C_head = -(R_T * prior_tvec);

    // Project each LED model point into the image under (R, prior_tvec).
    // LEDs are dropped from the visibility set if any of:
    //   * They're behind the camera (Pcam.z <= 1cm).
    //   * They're off-screen (u/v outside image bounds).
    //   * Their outward-facing direction points away from the camera
    //     (the helmet itself is occluding them). The visor center
    //     LED has no defined outward direction so we skip the check
    //     for it; it's effectively always visible from the front
    //     hemisphere anyway.
    //
    // Without this filter, the rear strap LEDs (indices 7-8) projected
    // somewhere in the image whenever they were in-frame geometrically,
    // and the random-sample inner loop below frequently picked them as
    // correspondence candidates for blobs that physically came from
    // front-of-visor LEDs. The resulting AP3P hypothesis would either
    // be impossible-Z (rejected) or low-quality (drowned in the
    // inlier-count race by other random samples), so the correct
    // identification rarely won. With the filter, the candidate LED
    // set is typically 4-6 forward-facing LEDs that ARE in fact what
    // the camera is seeing, which gives permutation-RANSAC a vastly
    // better-conditioned problem to solve.
    //
    // Cold-start case (no fresh IMU prior): prior_tvec is set above
    // by back-projecting the blob centroid to kDefaultUserZCm, so
    // C_head lands near (small, small, -60) and the front-facing
    // LEDs pass while the rear ones don't.
    constexpr int kMinVisibleForPnp = 4;
    std::array<cv::Point2d, NUM_LEDS> projected;
    std::array<bool, NUM_LEDS>        visible{};
    for (int i = 0; i < NUM_LEDS; ++i) {
        const cv::Vec3d model_pt(kLEDModel[i].x, kLEDModel[i].y, kLEDModel[i].z);
        const cv::Vec3d P = R * model_pt + prior_tvec;
        r.matched_blob_idx[i] = -1;
        if (P(2) <= 1.0) { visible[i] = false; r.visible[i] = false; continue; }

        // Facing-camera check using PROPER per-LED outward normals
        // (kLEDNormals). The previous heuristic (cdotp > r2) used
        // P_head as a proxy for the normal, which only works for a
        // sphere whose origin is the geometric centre - our model
        // has its origin at the visor centre (LED 0), 12 cm in
        // front of the head centre, so position vectors of front
        // LEDs incorrectly pointed "back-of-head" and the heuristic
        // dropped them. With per-LED normals derived from
        // (point - head_centre), the test is correct for every LED
        // (front, side, and rear strap).
        const cv::Vec3d to_camera = C_head - cv::Vec3d(
            kLEDModel[i].x, kLEDModel[i].y, kLEDModel[i].z);
        if (kLEDNormals[i].dot(to_camera) <= 0.0) {
            visible[i] = false;
            r.visible[i] = false;
            continue;
        }

        const double u = K(0, 0) * P(0) / P(2) + K(0, 2);
        const double v = K(1, 1) * P(1) / P(2) + K(1, 2);
        if (u < 0 || u >= img_w || v < 0 || v >= img_h) {
            visible[i] = false;
            r.visible[i] = false;
            continue;
        }
        projected[i]   = cv::Point2d(u, v);
        visible[i]     = true;
        r.projected[i] = projected[i];
        r.visible[i]   = true;
    }

    auto visible_count = [&visible]() {
        int n = 0;
        for (bool v : visible)
            if (v) ++n;
        return n;
    };

    if (visible_count() < kMinVisibleForPnp) {
        // The free camera-pose rvec can land on the planar-PnP twin that
        // reprojects well but makes the LED normals look self-occluded.
        // If that leaves too few candidates to solve XYZ, relax only the
        // normal-facing test for this frame. The later RANSAC, RMS, Z,
        // and jump gates still decide whether the correspondence is safe
        // to publish.
        for (int i = 0; i < NUM_LEDS; ++i) {
            const cv::Vec3d model_pt(kLEDModel[i].x, kLEDModel[i].y, kLEDModel[i].z);
            const cv::Vec3d P = R * model_pt + prior_tvec;
            if (P(2) <= 1.0) { visible[i] = false; r.visible[i] = false; continue; }

            const double u = K(0, 0) * P(0) / P(2) + K(0, 2);
            const double v = K(1, 1) * P(1) / P(2) + K(1, 2);
            if (u < 0 || u >= img_w || v < 0 || v >= img_h) {
                visible[i] = false;
                r.visible[i] = false;
                continue;
            }
            projected[i] = cv::Point2d(u, v);
            visible[i] = true;
            r.projected[i] = projected[i];
            r.visible[i] = true;
        }
    }

    // Permutation-RANSAC correspondence search. With a handful of real
    // LED blobs out of a noisy set, greedy NN (which we used
    // previously) often paired a real blob with the wrong LED index
    // because the prior-pose projection was off. solvePnPRansac on
    // those bad pairs then "fit" through the noise and produced
    // high-RMS or impossible-Z solutions.
    //
    // Instead, sample random (k-blob, k-LED, permutation) triples,
    // solve a P3P / AP3P for each, reproject the full LED model, and
    // score by the number of blobs within a tight pixel gate. Keep
    // the best. O(kSamples * cost(AP3P)) ~= 300 * 80us = 24ms/frame
    // max, well under our 33ms budget. The kSamples cap keeps worst
    // case bounded regardless of blob count.
    // k is the AP3P sample size for the cold-start permutation search
    // (structurally needs 4). The blob/visible-count REJECT thresholds,
    // however, use min_inliers: on the locked path (min_inliers=3) a
    // 3-blob / 3-visible frame is solvable by the prior-match + fixed-
    // rotation ICP without ever running AP3P, so it must not be rejected
    // here. The permutation branch below is separately guarded to only
    // run when there are >= k blobs/visible LEDs.
    const int k = kMinVisibleForPnp;  // min for P3P + disambiguation
    if ((int)blobs.size() < min_inliers) {
        r.reject_reason = "TOO_FEW_BLOBS";
        log_frame(dbg, yaw_rad, pitch_rad, roll_rad, prior_tvec,
                  blobs, projected, visible,
                  0, false, 0, cv::Vec3d(0, 0, 0),
                  "REJECT_TOO_FEW_BLOBS");
        return r;
    }

    // Count how many LEDs are currently visible (projected in-frame);
    // only sample from those since off-screen LEDs can't match a blob.
    std::vector<int> visible_leds;
    for (int i = 0; i < NUM_LEDS; ++i)
        if (visible[i]) visible_leds.push_back(i);
    r.n_visible = (int)visible_leds.size();
    if ((int)visible_leds.size() < min_inliers) {
        r.reject_reason = "TOO_FEW_VISIBLE";
        log_frame(dbg, yaw_rad, pitch_rad, roll_rad, prior_tvec,
                  blobs, projected, visible,
                  0, false, 0, cv::Vec3d(0, 0, 0),
                  "REJECT_TOO_FEW_VISIBLE");
        return r;
    }

    // Use thread-local RNG seeded with frame content so different
    // frames get different samples but runs are reproducible given
    // same input (useful for debugging).
    static thread_local cv::RNG rng(0xBEEF);
    const int kSamples = 300;
    const double inlier_px2 = perm_inlier_px * perm_inlier_px;
    const std::vector<double> no_distortion;

    struct MatchCandidate {
        double d2;
        int    led;
        int    blob;
    };

    std::vector<int> best_led_indices;
    std::vector<int> best_blob_indices;
    int              best_inliers = 0;
    double           best_match_error = 1e300;
    double           best_prior_d2 = 1e300;
    cv::Vec3d        best_rvec, best_tvec;
    bool             best_from_prior = false;

    std::vector<int> led_pick(k), blob_pick(k);
    std::vector<cv::Point3d> obj_sample(k);
    std::vector<cv::Point2d> img_sample(k);

    if (have_prior) {
        const double prior_px2 = prior_match_px * prior_match_px;
        std::vector<MatchCandidate> prior_matches;
        prior_matches.reserve(visible_leds.size() * blobs.size());
        for (int li : visible_leds) {
            for (size_t bj = 0; bj < blobs.size(); ++bj) {
                const double dx = blobs[bj].x - projected[li].x;
                const double dy = blobs[bj].y - projected[li].y;
                const double d2 = dx * dx + dy * dy;
                if (d2 < prior_px2)
                    prior_matches.push_back({d2, li, (int)bj});
            }
        }
        std::sort(prior_matches.begin(), prior_matches.end(),
                  [](const MatchCandidate& a, const MatchCandidate& b) {
                      return a.d2 < b.d2;
                  });

        std::array<bool, NUM_LEDS> led_claimed{};
        std::vector<bool> blob_claimed(blobs.size(), false);
        double match_error = 0.0;
        for (const MatchCandidate& mc : prior_matches) {
            if (led_claimed[mc.led] || blob_claimed[mc.blob])
                continue;
            led_claimed[mc.led] = true;
            blob_claimed[mc.blob] = true;
            best_led_indices.push_back(mc.led);
            best_blob_indices.push_back(mc.blob);
            match_error += mc.d2;
        }
        best_inliers = (int)best_led_indices.size();
        if (best_inliers > 0) {
            best_match_error = match_error;
            best_prior_d2 = 0.0;
            best_rvec = prior_rvec;
            best_tvec = prior_tvec;
            best_from_prior = best_inliers >= min_inliers;
        }
    }

    // Cold-start permutation search: only runnable with >= k blobs AND
    // >= k visible LEDs (AP3P samples k distinct of each). A locked
    // frame with 3 blobs skips this - it already has best_from_prior
    // from the prior-match above, or is rejected below as NO_AP3P_FIT.
    if (!best_from_prior &&
        (int)blobs.size() >= k && (int)visible_leds.size() >= k) {
        for (int iter = 0; iter < kSamples; ++iter) {
            // Random k-subset of visible LEDs (sample without replacement).
            for (int i = 0; i < k; ++i) {
                int idx;
                bool clash;
                do {
                    idx = rng.uniform(0, (int)visible_leds.size());
                    clash = false;
                    for (int p = 0; p < i; ++p)
                        if (led_pick[p] == idx) { clash = true; break; }
                } while (clash);
                led_pick[i] = idx;
            }
            // Random k-subset of blobs (sample without replacement).
            for (int i = 0; i < k; ++i) {
                int idx;
                bool clash;
                do {
                    idx = rng.uniform(0, (int)blobs.size());
                    clash = false;
                    for (int p = 0; p < i; ++p)
                        if (blob_pick[p] == idx) { clash = true; break; }
                } while (clash);
                blob_pick[i] = idx;
            }
            // Build the sample correspondences.
            for (int i = 0; i < k; ++i) {
                const int li = visible_leds[led_pick[i]];
                obj_sample[i] = kLEDModel[li];
                img_sample[i] = blobs[blob_pick[i]];
            }
            // Solve with AP3P (fast, needs exactly 4 for disambiguation).
            cv::Vec3d trial_rvec = prior_rvec;
            cv::Vec3d trial_tvec = prior_tvec;
            if (!cv::solvePnP(obj_sample, img_sample, cv::Mat(K),
                              no_distortion, trial_rvec, trial_tvec,
                              /*useExtrinsicGuess=*/false,
                              cv::SOLVEPNP_AP3P))
                continue;
            // Quick z-sanity reject before the expensive reproject.
            if (trial_tvec(2) < kMinAcceptableZCm ||
                trial_tvec(2) > kMaxAcceptableZCm) continue;
            // Prior-consistency reject: when we have a fresh last-accepted
            // pose, the solver shouldn't consider hypotheses that place
            // the head farther than the TIME-SCALED jump allowance.
            // Otherwise it alternates between two geometrically
            // consistent LED-to-blob assignments whose 3D poses differ
            // by 10-20 cm, producing the "XYZ jumping all over" symptom.
            // Dropping these hypotheses here (not just at the final jump
            // gate) lets the inlier-count race be won by the correct
            // cluster. The rotation check compares against prior_rvec -
            // the LIVE IMU-composed rotation for this frame - not the
            // stale cached rvec, so real head rotation during a dropout
            // doesn't fence off valid hypotheses.
            if (have_prior) {
                const cv::Vec3d d = trial_tvec - prior_tvec;
                if (d.dot(d) > jump_allow_cm * jump_allow_cm) continue;
                if (rotation_delta_deg(trial_rvec, prior_rvec) >
                    kMaxCameraRotDegPerFrame) {
                    continue;
                }
            }
            // Reproject the full 9-LED model and count blob inliers.
            std::array<cv::Point3d, NUM_LEDS> all_leds;
            for (int i = 0; i < NUM_LEDS; ++i) all_leds[i] = kLEDModel[i];
            std::vector<cv::Point2d> proj_all;
            cv::projectPoints(std::vector<cv::Point3d>(all_leds.begin(),
                                                       all_leds.end()),
                              trial_rvec, trial_tvec, cv::Mat(K),
                              no_distortion, proj_all);
            std::vector<MatchCandidate> match_candidates;
            match_candidates.reserve(NUM_LEDS * blobs.size());
            for (int i = 0; i < NUM_LEDS; ++i) {
                for (size_t j = 0; j < blobs.size(); ++j) {
                    const double dx = blobs[j].x - proj_all[i].x;
                    const double dy = blobs[j].y - proj_all[i].y;
                    const double d2 = dx * dx + dy * dy;
                    if (d2 < inlier_px2)
                        match_candidates.push_back({d2, i, (int)j});
                }
            }
            std::sort(match_candidates.begin(), match_candidates.end(),
                      [](const MatchCandidate& a, const MatchCandidate& b) {
                          return a.d2 < b.d2;
                      });

            int inliers_this = 0;
            double match_error = 0.0;
            std::vector<int> leds_used, blobs_used;
            std::array<bool, NUM_LEDS> led_claimed{};
            std::vector<bool> blob_claimed(blobs.size(), false);
            for (const MatchCandidate& mc : match_candidates) {
                if (led_claimed[mc.led] || blob_claimed[mc.blob])
                    continue;
                led_claimed[mc.led] = true;
                blob_claimed[mc.blob] = true;
                ++inliers_this;
                match_error += mc.d2;
                leds_used.push_back(mc.led);
                blobs_used.push_back(mc.blob);
            }
            const cv::Vec3d prior_delta = trial_tvec - prior_tvec;
            const double prior_d2 = prior_delta.dot(prior_delta);
            if (inliers_this > best_inliers ||
                (inliers_this == best_inliers &&
                 (match_error < best_match_error ||
                  (match_error == best_match_error && prior_d2 < best_prior_d2)))) {
                best_inliers = inliers_this;
                best_match_error = match_error;
                best_prior_d2 = prior_d2;
                best_rvec = trial_rvec;
                best_tvec = trial_tvec;
                best_from_prior = false;
                best_led_indices  = leds_used;
                best_blob_indices = blobs_used;
            }
        }
    }

    // Produce the 2D-3D correspondence set from the best hypothesis
    // for downstream solvePnPRansac refinement.
    std::vector<cv::Point3d> obj_pts;
    std::vector<cv::Point2d> img_pts;
    obj_pts.reserve(best_inliers);
    img_pts.reserve(best_inliers);
    for (int kk = 0; kk < best_inliers; ++kk) {
        const int li = best_led_indices[kk];
        const int bj = best_blob_indices[kk];
        obj_pts.push_back(kLEDModel[li]);
        img_pts.push_back(blobs[bj]);
        r.matched_blob_idx[li] = bj;
    }

    r.n_matched = best_inliers;
    if (r.n_matched < min_inliers) {
        r.reject_reason = "NO_AP3P_FIT";
        log_frame(dbg, yaw_rad, pitch_rad, roll_rad, prior_tvec,
                  blobs, projected, visible,
                  r.n_matched, false, 0, cv::Vec3d(0, 0, 0),
                  "REJECT_NO_AP3P_FIT");
        return r;
    }

    // Stash the ORIGINAL prior (from last-accepted frame) before we
    // overwrite prior_rvec/tvec with the permutation-RANSAC winner -
    // the jump gate downstream must compare the refined tvec against
    // the last-accepted frame, NOT against this frame's hypothesis,
    // or else any jump-gate violation gets masked by seeding bias.
    const cv::Vec3d jump_ref_tvec = prior_tvec;
    const bool      jump_ref_valid = have_prior;

    // Seed the final solve with the best hypothesis's pose so it
    // starts close to the right answer rather than the stale prior.
    prior_rvec = best_rvec;
    prior_tvec = best_tvec;

    // ----------------------------------------------------------------
    // Final camera pose: rotation-locked ICP (translation-only fit).
    //
    // Rotation is NEVER solved freely here. Locked frames hold R to
    // the composition R_cached * R_imu_at_accept^T * R_imu_now (live
    // IMU delta on the lock-time extrinsic, computed above). Cold-
    // start frames anchor R OPTICALLY from the permutation winner's
    // rotation below. Holding rotation fixed removes the near-planar
    // PnP two-fold ambiguity that threw XYZ around with no relation
    // to head motion.
    //
    // We alternate, ICP-style:
    //   1. project the facing-camera LEDs at (R, t),
    //   2. greedy-match each to its nearest blob within the gate,
    //   3. re-solve translation (3 DOF) over those matches.
    // Under a fixed rotation the translation that fits a given match
    // set is unique, so the result is seed-independent. The user-
    // visible yaw/pitch/roll stay IMU-driven in psvr.cpp.
    cv::Vec3d rvec, tvec;
    double    rms           = 0.0;
    int       final_inliers = 0;

    if (!have_prior) {
        // Cold start: anchor rotation from the permutation winner's
        // optically solved rvec, NOT the raw IMU convention. The IMU
        // yaw is referenced to wherever gyro integration started - the
        // user may have calibrated facing 30+ deg away from the camera
        // - so the IMU-conventional R can be arbitrarily wrong in
        // azimuth; holding it would either never pass the RMS gate or
        // lock distorted geometry. Anchoring optically captures that
        // offset into the cached extrinsic: once locked, later frames
        // compose the LIVE IMU delta on top of it, so the offset is
        // calibrated away for the rest of the lock.
        //
        // Planar-twin disambiguation: the LEDs visible from the front
        // (5 visor + 2 temples) are symmetric under S = diag(-1,-1,1)
        // in the model frame, so AP3P returns either the true pose or
        // its twin R_opt*S with mirrored correspondences - the two
        // reproject IDENTICALLY and no optical gate can tell them
        // apart from a single frontal frame (verified: the twin fits
        // exact synthetic data at 0 px RMS). The IMU breaks the tie:
        // its pitch/roll are gravity-anchored and the user's yaw zero
        // is in practice within ~90 deg of the camera axis, while the
        // twin is always ~180 deg out. Pick whichever candidate lies
        // closer to the IMU-conventional rotation.
        cv::Mat R_opt_m;
        cv::Rodrigues(best_rvec, R_opt_m);
        if (R_opt_m.rows == 3 && R_opt_m.cols == 3) {
            cv::Matx33d R_opt;
            for (int r_i = 0; r_i < 3; ++r_i)
                for (int c_i = 0; c_i < 3; ++c_i)
                    R_opt(r_i, c_i) = R_opt_m.at<double>(r_i, c_i);
            const cv::Matx33d S(-1, 0, 0,  0, -1, 0,  0, 0, 1);
            const cv::Matx33d R_twin = R_opt * S;
            auto angle_to_imu = [&](const cv::Matx33d& A) {
                const cv::Matx33d D = A * R_imu_now.t();
                const double tr = D(0, 0) + D(1, 1) + D(2, 2);
                return std::acos(std::clamp((tr - 1.0) * 0.5, -1.0, 1.0));
            };
            R = (angle_to_imu(R_opt) <= angle_to_imu(R_twin)) ? R_opt
                                                              : R_twin;
        }
    }
    cv::Rodrigues(R, rvec);   // published rotation == R used for the fit

    // Seed policy: primary seed is the permutation-search winner's
    // translation (prior_tvec == best_tvec after the stash above) -
    // it already passed the inlier race and the Z/jump hypothesis
    // gates, so it's the best estimate available this frame. The old
    // all-blob centroid back-projection is kept only as a FALLBACK
    // retry: with spurious non-LED blobs in frame (room lights), the
    // centroid is dragged toward them, the seed projection lands >
    // gate distance from the real blobs, and the ICP starved below
    // kMinInliers - dropping frames RANSAC had already solved.
    cv::Vec3d t_centroid;
    {
        double cu = 0.0, cvv = 0.0;
        for (const auto& b : blobs) { cu += b.x; cvv += b.y; }
        cu /= blobs.size();
        cvv /= blobs.size();
        const double z0 = have_prior ? prior_tvec(2) : kDefaultUserZCm;
        t_centroid(0) = (cu  - K(0, 2)) * z0 / K(0, 0);
        t_centroid(1) = (cvv - K(1, 2)) * z0 / K(1, 1);
        t_centroid(2) = z0;
    }

    const double icp_px2 = prior_match_px * prior_match_px;
    std::vector<int> matched_leds, matched_blobs;
    cv::Vec3d t_icp;

    // One ICP run from a given translation seed. LED candidacy is
    // re-derived EVERY iteration from the CURRENT (R, t) - behind-
    // camera and facing-away LEDs are skipped per-iteration rather
    // than using the `visible_leds` set frozen at the (possibly
    // stale/cold-start) prior pose. The frozen set could permanently
    // exclude LEDs that are on-screen at the true depth but projected
    // off-frame at the initial guess, starving the first lock of
    // inliers it physically had.
    auto run_icp = [&](cv::Vec3d seed) -> bool {
        t_icp = seed;
        bool ok = false;
        for (int iter = 0; iter < 6; ++iter) {
            struct MC { double d2; int led; int blob; };
            std::vector<MC> cand;
            const cv::Vec3d C_now = -(R.t() * t_icp);
            for (int li = 0; li < NUM_LEDS; ++li) {
                const cv::Vec3d P(kLEDModel[li].x, kLEDModel[li].y,
                                  kLEDModel[li].z);
                const cv::Vec3d to_cam = C_now - P;
                if (kLEDNormals[li].dot(to_cam) <= 0.0) continue;
                const cv::Vec3d Q = R * P + t_icp;
                if (Q(2) <= 1.0) continue;
                const double u = K(0, 0) * Q(0) / Q(2) + K(0, 2);
                const double v = K(1, 1) * Q(1) / Q(2) + K(1, 2);
                for (size_t bj = 0; bj < blobs.size(); ++bj) {
                    const double dx = blobs[bj].x - u, dy = blobs[bj].y - v;
                    const double d2 = dx * dx + dy * dy;
                    if (d2 < icp_px2) cand.push_back({d2, li, (int)bj});
                }
            }
            std::sort(cand.begin(), cand.end(),
                      [](const MC& a, const MC& b) { return a.d2 < b.d2; });
            std::array<bool, NUM_LEDS> led_claimed{};
            std::vector<bool> blob_claimed(blobs.size(), false);
            matched_leds.clear();
            matched_blobs.clear();
            for (const MC& mc : cand) {
                if (led_claimed[mc.led] || blob_claimed[mc.blob]) continue;
                led_claimed[mc.led]   = true;
                blob_claimed[mc.blob] = true;
                matched_leds.push_back(mc.led);
                matched_blobs.push_back(mc.blob);
            }
            if ((int)matched_leds.size() < min_inliers) break;
            std::vector<cv::Point3d> obj_in;
            std::vector<cv::Point2d> img_in;
            obj_in.reserve(matched_leds.size());
            img_in.reserve(matched_leds.size());
            for (size_t m = 0; m < matched_leds.size(); ++m) {
                obj_in.push_back(kLEDModel[matched_leds[m]]);
                img_in.push_back(blobs[matched_blobs[m]]);
            }
            const cv::Vec3d t_prev = t_icp;
            if (!solve_translation_fixed_rotation(R, K, obj_in, img_in,
                                                  t_icp))
                return false;
            ok = true;
            const cv::Vec3d d = t_icp - t_prev;
            if (d.dot(d) < 1e-4) break;   // converged (<0.1 mm)
        }
        return ok && (int)matched_leds.size() >= min_inliers;
    };

    bool icp_ok = run_icp(prior_tvec);
    if (!icp_ok)
        icp_ok = run_icp(t_centroid);

    if (!icp_ok) {
        // Starved of matches vs. numerically diverged translation solve;
        // matched_leds reflects the last (fallback) attempt.
        const bool starved = (int)matched_leds.size() < min_inliers;
        r.reject_reason = starved ? "NO_AP3P_FIT" : "T_SOLVE_DIVERGED";
        log_frame(dbg, yaw_rad, pitch_rad, roll_rad, prior_tvec,
                  blobs, projected, visible,
                  (int)matched_leds.size(), false, 0, t_icp,
                  starved ? "REJECT_NO_AP3P_FIT" : "REJECT_T_SOLVE_DIVERGED");
        return r;
    }

    tvec          = t_icp;
    final_inliers = (int)matched_leds.size();
    r.n_matched   = final_inliers;
    for (int i = 0; i < NUM_LEDS; ++i) r.matched_blob_idx[i] = -1;
    for (size_t m = 0; m < matched_leds.size(); ++m)
        r.matched_blob_idx[matched_leds[m]] = matched_blobs[m];

    // RMS over the matched set under the IMU-locked pose.
    double sum_sq = 0.0;
    for (size_t m = 0; m < matched_leds.size(); ++m) {
        const cv::Vec3d P(kLEDModel[matched_leds[m]].x,
                          kLEDModel[matched_leds[m]].y,
                          kLEDModel[matched_leds[m]].z);
        const cv::Vec3d Q = R * P + tvec;
        const double iz = 1.0 / Q(2);
        const double dx = K(0, 0) * Q(0) * iz + K(0, 2) - blobs[matched_blobs[m]].x;
        const double dy = K(1, 1) * Q(1) * iz + K(1, 2) - blobs[matched_blobs[m]].y;
        sum_sq += dx * dx + dy * dy;
    }
    rms = std::sqrt(sum_sq / (double)matched_leds.size());

    r.reprojection_rms = rms;
    if (rms > max_rms_px) {
        r.reject_reason = "HIGH_RMS";
        log_frame(dbg, yaw_rad, pitch_rad, roll_rad, prior_tvec,
                  blobs, projected, visible,
                  r.n_matched, true, rms, tvec, "REJECT_HIGH_RMS");
        return r;
    }

    // No final rotation-jump gate: the published rvec IS the live
    // IMU-composed rotation (never optically solved), so its frame-to-
    // frame delta measures real head rotation, not solver error.
    // Gating it froze the tracker whenever the user turned their head
    // during a reject burst. The per-hypothesis rotation check inside
    // the permutation search still guards the free AP3P rotations.

    // Z sanity check: a user <20 cm or >2 m from the camera is almost
    // certainly a spurious geometric fit. Those distances land a head
    // against the screen or across the room, far outside any desk-
    // flight-sim use case.
    if (tvec(2) < kMinAcceptableZCm || tvec(2) > kMaxAcceptableZCm) {
        r.reject_reason = "Z_OUT_OF_RANGE";
        log_frame(dbg, yaw_rad, pitch_rad, roll_rad, prior_tvec,
                  blobs, projected, visible,
                  r.n_matched, true, rms, tvec, "REJECT_Z_OUT_OF_RANGE");
        return r;
    }

    // Translation jump gate, TIME-SCALED (see jump_allow_cm above). A
    // real head can't move faster than ~1 m/s; anything more since the
    // last ACCEPT means the matcher latched on to a different
    // configuration (spurious lamp blob, near-symmetric pose
    // ambiguity). Reject without updating the cached prior, so the
    // next frame gets another shot with the last-good prior - and
    // because the allowance grows with elapsed time, a reject burst
    // opens the gate instead of fencing off reacquire forever.
    // Use the pre-RANSAC reference (last-accepted frame's pose) for
    // the jump gate, not the mid-frame permutation hypothesis seed.
    if (jump_ref_valid) {
        const cv::Vec3d d = tvec - jump_ref_tvec;
        const double jump = std::sqrt(d.dot(d));
        if (jump > jump_allow_cm) {
            r.reject_reason = "JUMP";
            log_frame(dbg, yaw_rad, pitch_rad, roll_rad, jump_ref_tvec,
                      blobs, projected, visible,
                      r.n_matched, true, rms, tvec, "REJECT_JUMP");
            return r;
        }
    }

    // Strong-lock-on-first-publish gate. When we DON'T have a fresh
    // prior, the jump gate above couldn't run; the only remaining
    // defense against latching a one-off false geometric fit is to
    // demand a higher inlier count before publishing the very first
    // position. Once the lock is established (have_prior becomes
    // true on the next frame), the standard kMinInliers floor
    // applies and the jump gate takes over consistency duty.
    //
    // Without this gate, the matcher would occasionally publish a
    // single-frame nonsense position (the observed (20, 63, 178) cm
    // false-lock landed here) and the consuming pose pipeline
    // smoothed it through to opentrack's output, producing the
    // visible "green dots flash, X/Y/Z jumps" symptom.
    if (!jump_ref_valid && final_inliers < kStrongLockMinInliers) {
        if (final_inliers >= kMinInliers && rms <= firstlock_rms_px) {
            const double now_sec = steady_now_sec();
            bool promote = false;
            int hits = 1;
            {
                std::lock_guard<std::mutex> lk(s.state_mu);
                const bool fresh =
                    s.tentative_valid &&
                    (now_sec - s.tentative_epoch_sec) <= kFirstLockTentativeSec;
                if (fresh) {
                    const cv::Vec3d d = tvec - s.tentative_tvec;
                    if (d.dot(d) <= kFirstLock4LedMaxCm * kFirstLock4LedMaxCm) {
                        s.tentative_hits++;
                        s.tentative_tvec = tvec;
                    } else {
                        s.tentative_hits = 1;
                        s.tentative_tvec = tvec;
                    }
                } else {
                    s.tentative_valid = true;
                    s.tentative_hits = 1;
                    s.tentative_tvec = tvec;
                }
                s.tentative_epoch_sec = now_sec;
                hits = s.tentative_hits;
                promote = hits >= kFirstLock4LedHits;
            }
            if (!promote) {
                r.reject_reason = "TENTATIVE_FIRST_LOCK";
                log_frame(dbg, yaw_rad, pitch_rad, roll_rad, prior_tvec,
                          blobs, projected, visible,
                          r.n_matched, true, rms, tvec,
                          "REJECT_TENTATIVE_FIRST_LOCK");
                return r;
            }
            if (dbg) {
                std::fprintf(dbg,
                    "# promoting 4-LED tentative first lock after %d close frames\n",
                    hits);
                std::fflush(dbg);
            }
        } else {
            {
                std::lock_guard<std::mutex> lk(s.state_mu);
                s.tentative_valid = false;
                s.tentative_hits = 0;
            }
            r.reject_reason = "WEAK_FIRST_LOCK";
            log_frame(dbg, yaw_rad, pitch_rad, roll_rad, prior_tvec,
                      blobs, projected, visible,
                      r.n_matched, true, rms, tvec, "REJECT_WEAK_FIRST_LOCK");
            return r;
        }
    }

    // Accept. Cache as prior for the next frame and pass tvec through
    // verbatim in cm. This is the same convention tracker-aruco and
    // tracker-pt use (data[TX/TY/TZ] = tvec.x/y/z in cm), so a user
    // who's configured curves for those trackers gets identical
    // behavior here. OpenCV camera frame: +X right (camera-image),
    // +Y down, +Z into scene (away from camera). Previously we
    // flipped Y and Z into an OpenGL-style frame; that was
    // inconsistent with every other opentrack camera tracker and
    // forced users to invert axes in opentrack's curve UI on top
    // of the standard centering operation, which masked the
    // "doesn't move" symptom as "moves the wrong way".
    {
        std::lock_guard<std::mutex> lk(s.state_mu);
        s.state_rvec      = rvec;
        s.state_tvec      = tvec;
        // Pair the accepted camera rotation with THIS frame's IMU
        // rotation so later frames can apply the live IMU delta (see
        // state_imu_rvec in Impl).
        s.state_imu_rvec  = imu_rvec_now;
        s.state_epoch_sec = steady_now_sec();
        s.state_valid     = true;
        s.tentative_valid = false;
        s.tentative_hits  = 0;
    }

    r.ok   = true;
    r.x_cm = tvec(0);
    r.y_cm = tvec(1);
    r.z_cm = tvec(2);
    log_frame(dbg, yaw_rad, pitch_rad, roll_rad, prior_tvec,
              blobs, projected, visible,
              r.n_matched, true, rms, tvec, "ACCEPT");
    return r;
}

} // namespace psvr_constellation
