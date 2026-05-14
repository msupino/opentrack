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
 *   3. Project the 9 canonical LED positions through the IMU rotation
 *      prior at the guessed translation to get their expected image-
 *      space locations. LEDs that land behind the camera or outside
 *      the frame get dropped (they can't be matched).
 *   4. Greedy nearest-neighbor assignment from projected LEDs to the
 *      observed blobs, rejecting matches beyond MAX_MATCH_DIST_PX.
 *      Each blob can be used at most once. O(NUM_LEDS * blobs) is fine
 *      for 9 * ~20.
 *   5. If >=4 correspondences, run cv::solvePnP SOLVEPNP_ITERATIVE with
 *      the extrinsic guess turned on when we have a prior, otherwise
 *      without. The "iterative" algorithm minimizes reprojection error
 *      directly so it doubles as the refine step.
 *   6. Compute reprojection RMS; reject if > MAX_REPROJECTION_RMS_PX.
 *      Also reject if the translation jumps more than MAX_CM_PER_FRAME
 *      from the previous accepted result - guards against a blob burst
 *      from a lamp bouncing the matcher to a local-minimum pose.
 *   7. On accept, update cached (rvec, tvec, timestamp) for next
 *      frame, and return x_cm/y_cm/z_cm in the Worker's convention
 *      (+X right, +Y up, -Z forward) which inverts OpenCV's +Y-down
 *      camera frame on Y and Z.
 *
 * Coordinate frame notes
 * ----------------------
 * Head frame (as used in kLEDModel below): +X = user's right, +Y = up,
 * +Z = back of head (right-handed; cross(+X, +Y) = +Z). Forward-where-
 * face-points is -Z. The model coordinates differ from PSMoveService's
 * original by a Z-sign flip - see the comment on kLEDModel - because
 * cv::solvePnP needs a right-handed model frame.
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
 * dodge that entirely because we HAVE a rotation prior from the IMU
 * and a translation prior from the previous frame (or a
 * "user-at-arm-length" default for cold start). Those two priors are
 * usually good enough that greedy NN matching just works, and when it
 * doesn't the jump gate rejects the frame so we don't latch a bad
 * pose. This is the approach PSMoveService/MorpheusHMD uses.
 */
#include "psvr_constellation.h"

#include <opencv2/calib3d.hpp>
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
static const std::array<cv::Point3d, NUM_LEDS> kLEDModel = {{
    { 0.0,  0.0,   0.0},
    { 8.0,  4.5,   2.5},
    { 9.0,  0.0,  10.0},
    { 8.0, -4.5,   2.5},
    {-8.0,  4.5,   2.5},
    {-9.0,  0.0,  10.0},
    {-8.0, -4.5,   2.5},
    { 6.0, -1.0,  24.0},
    {-6.0, -1.0,  24.0},
}};

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
    double     state_epoch_sec = 0.0;

    std::mutex log_mu;
    FILE*      log_fp           = nullptr;
    bool       log_attempted    = false;
};

namespace {

// Tuning constants. Collected here so all the numbers that determine
// "does the solver latch or not" are visible at a glance.
constexpr double kDefaultHFOVDeg        = 70.0; // typical laptop webcam
constexpr double kMaxMatchDistPx        = 250.0;// projected-LED to blob
// Inlier RMS threshold is set loose (30 px @ 1920x1080) because the
// LED constellation model is "eyeballed" from PSMoveService (~5 mm
// accuracy per LED) and the 70-deg HFOV intrinsic is approximate -
// both produce ~20-25 px systematic reprojection error even on
// genuinely correct correspondences. A downstream opentrack smoothing
// filter (EWMA / Accela) cleans up the frame-to-frame jitter.
constexpr double kMaxReprojectionRMSPx  = 30.0;
constexpr double kRansacInlierThreshPx  = 25.0; // per-point error for RANSAC
constexpr int    kRansacIterations      = 200;  // usually converges in <50
// Inlier gate for the upstream permutation-search step: a candidate
// pose is scored by counting how many of the 9 LEDs project within
// this many pixels of any blob. Set wider than kRansacInlierThreshPx
// so the search is permissive (we want to identify the right
// LED-to-blob assignment, then let downstream RANSAC tighten the fit).
constexpr double kPermSearchInlierPx    = 30.0;
// Jump gate: at 30 Hz, a real head can move at most ~1 m/s
// comfortably, which is ~3 cm per frame. We set the gate at 6 cm
// (~1.8 m/s - fast head motion) to reject the common failure mode
// where the permutation matcher oscillates between two geometrically
// consistent LED-to-blob assignments that differ by 10-20 cm in 3D.
// Both interpretations have equally-low reprojection RMS so RANSAC
// can't break the tie; the jump gate does it by locking in whichever
// interpretation won the first frame. A briefly-stale prior (>1 s)
// resets this lock so a real re-enter-frame-from-elsewhere scenario
// isn't permanently rejected.
constexpr double kMaxCmPerFrame         = 6.0;
constexpr int    kMinInliers            = 4;    // solvePnPRansac minimum
constexpr double kStalenessResetSec     = 1.0;  // prior expires after
constexpr double kDefaultUserZCm        = 60.0; // cold-start Z guess
constexpr double kMinAcceptableZCm      = 20.0; // sanity floor
constexpr double kMaxAcceptableZCm      = 200.0;// sanity ceiling

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
                          double yaw_rad, double pitch_rad, double roll_rad)
{
    Impl& s = *impl_;
    Result r;
    r.n_blobs_total = (int)blobs.size();
    FILE* dbg = debug_log_fp(s);
    if ((int)blobs.size() < kMinInliers) {
        if (dbg) {
            std::array<cv::Point2d, NUM_LEDS> empty_proj{};
            std::array<bool, NUM_LEDS>        empty_vis{};
            log_frame(dbg, yaw_rad, pitch_rad, roll_rad, cv::Vec3d(0, 0, 0),
                      blobs, empty_proj, empty_vis,
                      0, false, 0, cv::Vec3d(0, 0, 0), "REJECT_TOO_FEW_BLOBS");
        }
        return r;
    }

    // Snapshot prior state under the mutex, then release before the
    // OpenCV work. The state is only a handful of doubles so the copy
    // is cheap; we never hold the mutex across solvePnP.
    cv::Vec3d prior_rvec, prior_tvec;
    bool      have_prior;
    {
        std::lock_guard<std::mutex> lk(s.state_mu);
        const bool fresh = s.state_valid &&
                           (steady_now_sec() - s.state_epoch_sec) < kStalenessResetSec;
        have_prior = fresh;
        prior_rvec = have_prior ? s.state_rvec : cv::Vec3d(0, 0, 0);
        prior_tvec = have_prior ? s.state_tvec : cv::Vec3d(0, 0, kDefaultUserZCm);
    }

    const cv::Matx33d R = head_to_camera_rotation(yaw_rad, pitch_rad, roll_rad);
    const cv::Matx33d K = make_intrinsics(img_w, img_h, kDefaultHFOVDeg);

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
    std::array<cv::Point2d, NUM_LEDS> projected;
    std::array<bool, NUM_LEDS>        visible{};
    for (int i = 0; i < NUM_LEDS; ++i) {
        const cv::Vec3d model_pt(kLEDModel[i].x, kLEDModel[i].y, kLEDModel[i].z);
        const cv::Vec3d P = R * model_pt + prior_tvec;
        r.matched_blob_idx[i] = -1;
        if (P(2) <= 1.0) { visible[i] = false; r.visible[i] = false; continue; }

        // Facing-camera check. The 1.0 cm^2 floor handles the visor-
        // center LED (|P_head| = 0) without a magic special-case
        // branch: when r2 is tiny we just skip the gate and treat the
        // LED as visible from any reasonable angle.
        const cv::Vec3d Phead(kLEDModel[i].x, kLEDModel[i].y, kLEDModel[i].z);
        const double r2 = Phead.dot(Phead);
        if (r2 > 1.0) {
            const double cdotp = C_head.dot(Phead);
            if (cdotp <= r2) {
                visible[i] = false;
                r.visible[i] = false;
                continue;
            }
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
    const int k = 4;  // min for P3P + disambiguation
    if ((int)blobs.size() < k) {
        log_frame(dbg, yaw_rad, pitch_rad, roll_rad, prior_tvec,
                  blobs, projected, visible,
                  0, false, 0, cv::Vec3d(0, 0, 0),
                  "REJECT_TOO_FEW_MATCHES");
        return r;
    }

    // Count how many LEDs are currently visible (projected in-frame);
    // only sample from those since off-screen LEDs can't match a blob.
    std::vector<int> visible_leds;
    for (int i = 0; i < NUM_LEDS; ++i)
        if (visible[i]) visible_leds.push_back(i);
    if ((int)visible_leds.size() < k) {
        log_frame(dbg, yaw_rad, pitch_rad, roll_rad, prior_tvec,
                  blobs, projected, visible,
                  0, false, 0, cv::Vec3d(0, 0, 0),
                  "REJECT_TOO_FEW_MATCHES");
        return r;
    }

    // Use thread-local RNG seeded with frame content so different
    // frames get different samples but runs are reproducible given
    // same input (useful for debugging).
    static thread_local cv::RNG rng(0xBEEF);
    const int kSamples = 300;
    const double inlier_px2 = kPermSearchInlierPx * kPermSearchInlierPx;
    const std::vector<double> no_distortion;

    std::vector<int> best_led_indices;
    std::vector<int> best_blob_indices;
    int              best_inliers = 0;
    cv::Vec3d        best_rvec, best_tvec;

    std::vector<int> led_pick(k), blob_pick(k);
    std::vector<cv::Point3d> obj_sample(k);
    std::vector<cv::Point2d> img_sample(k);

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
        // the head more than kMaxCmPerFrame away. Otherwise it
        // alternates between two geometrically consistent LED-to-
        // blob assignments whose 3D poses differ by 10-20 cm,
        // producing the "XYZ jumping all over" symptom. Dropping
        // these hypotheses here (not just at the final jump gate)
        // lets the inlier-count race be won by the correct cluster.
        if (have_prior) {
            const cv::Vec3d d = trial_tvec - prior_tvec;
            if (d.dot(d) > kMaxCmPerFrame * kMaxCmPerFrame) continue;
        }
        // Reproject the full 9-LED model and count blob inliers.
        std::array<cv::Point3d, NUM_LEDS> all_leds;
        for (int i = 0; i < NUM_LEDS; ++i) all_leds[i] = kLEDModel[i];
        std::vector<cv::Point2d> proj_all;
        cv::projectPoints(std::vector<cv::Point3d>(all_leds.begin(),
                                                   all_leds.end()),
                          trial_rvec, trial_tvec, cv::Mat(K),
                          no_distortion, proj_all);
        int inliers_this = 0;
        std::vector<int> leds_used, blobs_used;
        for (int i = 0; i < NUM_LEDS; ++i) {
            double best_d2 = inlier_px2;
            int    best_b  = -1;
            for (size_t j = 0; j < blobs.size(); ++j) {
                const double dx = blobs[j].x - proj_all[i].x;
                const double dy = blobs[j].y - proj_all[i].y;
                const double d2 = dx * dx + dy * dy;
                if (d2 < best_d2) { best_d2 = d2; best_b = (int)j; }
            }
            if (best_b >= 0) {
                ++inliers_this;
                leds_used.push_back(i);
                blobs_used.push_back(best_b);
            }
        }
        if (inliers_this > best_inliers) {
            best_inliers = inliers_this;
            best_rvec = trial_rvec;
            best_tvec = trial_tvec;
            best_led_indices  = leds_used;
            best_blob_indices = blobs_used;
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
    if (r.n_matched < kMinInliers) {
        log_frame(dbg, yaw_rad, pitch_rad, roll_rad, prior_tvec,
                  blobs, projected, visible,
                  r.n_matched, false, 0, cv::Vec3d(0, 0, 0),
                  "REJECT_TOO_FEW_MATCHES");
        return r;
    }

    // Stash the ORIGINAL prior (from last-accepted frame) before we
    // overwrite prior_rvec/tvec with the permutation-RANSAC winner -
    // the jump gate downstream must compare the refined tvec against
    // the last-accepted frame, NOT against this frame's hypothesis,
    // or else any jump-gate violation gets masked by seeding bias.
    const cv::Vec3d jump_ref_tvec = prior_tvec;
    const bool      jump_ref_valid = have_prior;

    // Seed the refinement with the best hypothesis's pose so RANSAC
    // starts close to the right answer rather than the prior.
    prior_rvec = best_rvec;
    prior_tvec = best_tvec;

    // Refine via solvePnPRansac. The camera pipeline produces many
    // spurious blobs beyond the 9 real LEDs - room lights, monitor
    // reflections, glasses glare, etc - so we widened the match gate
    // above to be permissive on candidate correspondences and let
    // RANSAC reject the outliers. Prior tvec/rvec is the extrinsic
    // guess seeding the refinement step; inliers come back as a
    // column-vector of indices into obj_pts/img_pts.
    cv::Vec3d rvec = prior_rvec;
    cv::Vec3d tvec = prior_tvec;
    // no_distortion already declared above in the permutation-RANSAC
    // search block; reuse it here rather than re-declaring.
    cv::Mat inliers;
    const bool ok = cv::solvePnPRansac(obj_pts, img_pts, cv::Mat(K),
                                       no_distortion,
                                       rvec, tvec,
                                       /*useExtrinsicGuess=*/true,
                                       kRansacIterations,
                                       (float)kRansacInlierThreshPx,
                                       /*confidence=*/0.99,
                                       inliers,
                                       cv::SOLVEPNP_ITERATIVE);
    if (!ok || inliers.rows < kMinInliers) {
        log_frame(dbg, yaw_rad, pitch_rad, roll_rad, prior_tvec,
                  blobs, projected, visible,
                  ok ? (int)inliers.rows : 0, false, 0, tvec,
                  "REJECT_RANSAC_FEW_INLIERS");
        return r;
    }
    r.n_matched = (int)inliers.rows;

    // Compute RMS over INLIERS only. Outliers (monitor reflections etc)
    // that RANSAC already discarded must not poison the quality metric.
    std::vector<cv::Point2d> reproj;
    cv::projectPoints(obj_pts, rvec, tvec, cv::Mat(K), no_distortion, reproj);
    double sum_sq = 0.0;
    for (int k = 0; k < inliers.rows; ++k) {
        const int i = inliers.at<int>(k, 0);
        const double dx = reproj[i].x - img_pts[i].x;
        const double dy = reproj[i].y - img_pts[i].y;
        sum_sq += dx * dx + dy * dy;
    }
    const double rms = std::sqrt(sum_sq / (double)inliers.rows);
    r.reprojection_rms = rms;
    if (rms > kMaxReprojectionRMSPx) {
        log_frame(dbg, yaw_rad, pitch_rad, roll_rad, prior_tvec,
                  blobs, projected, visible,
                  r.n_matched, true, rms, tvec, "REJECT_HIGH_RMS");
        return r;
    }

    // Z sanity check: a user <20 cm or >2 m from the camera is almost
    // certainly a spurious geometric fit. Those distances land a head
    // against the screen or across the room, far outside any desk-
    // flight-sim use case.
    if (tvec(2) < kMinAcceptableZCm || tvec(2) > kMaxAcceptableZCm) {
        log_frame(dbg, yaw_rad, pitch_rad, roll_rad, prior_tvec,
                  blobs, projected, visible,
                  r.n_matched, true, rms, tvec, "REJECT_Z_OUT_OF_RANGE");
        return r;
    }

    // Frame-to-frame jump gate. A real user can't physically move their
    // head more than kMaxCmPerFrame in 33ms; anything larger means the
    // matcher latched on to a different configuration (spurious lamp
    // blob, near-symmetric pose ambiguity). Reject without updating
    // the cached prior, so the next frame gets another shot with the
    // last-good prior.
    // Use the pre-RANSAC reference (last-accepted frame's pose) for
    // the jump gate, not the mid-frame permutation hypothesis seed.
    if (jump_ref_valid) {
        const cv::Vec3d d = tvec - jump_ref_tvec;
        const double jump = std::sqrt(d.dot(d));
        if (jump > kMaxCmPerFrame) {
            log_frame(dbg, yaw_rad, pitch_rad, roll_rad, jump_ref_tvec,
                      blobs, projected, visible,
                      r.n_matched, true, rms, tvec, "REJECT_JUMP");
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
        s.state_epoch_sec = steady_now_sec();
        s.state_valid     = true;
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
