/* PSVR LED constellation model + PnP solver.
 *
 * Given a list of blue-blob image coordinates (from psvr_camera.mm) and
 * the current IMU rotation as a prior, try to identify which blob is
 * which PSVR LED and solve for the headset's 3D position in the
 * camera's coordinate frame.
 *
 * Separated from psvr_camera.mm so the CV math stays testable (it's
 * plain C++ and doesn't pull in AVFoundation / Foundation). The
 * interface uses cv::Point2d rather than CV_WRAP types so callers can
 * build vectors cheaply.
 */
#pragma once

#include <cstdio>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <vector>

namespace psvr_constellation {

// Per-frame solver output. Position is the head origin in the OpenCV
// camera frame, in cm: +X right (camera-image), +Y down, +Z into
// scene (away from camera). Same convention tracker-aruco and
// tracker-pt use. `ok == false` means the solver didn't find a
// plausible pose this frame; `n_matched` and `n_blobs_total` are
// still filled so callers can log the failure mode (too few blobs,
// identification ambiguous, high reprojection error, etc.).
//
// Debug overlay data (projected / visible / matched_blob_idx) is
// filled even on reject paths so callers can render what the solver
// was seeing when it gave up.
struct Result {
    bool   ok{false};
    double x_cm{0}, y_cm{0}, z_cm{0};
    int    n_matched{0};       // blobs that were assigned to LEDs
    int    n_visible{0};       // LEDs that passed the facing-camera filter
    int    n_blobs_total{0};   // input blob count
    double reprojection_rms{0};// in pixels; only meaningful when ok

    // Short tag identifying the path that produced this Result. One
    // of: "OK", "TOO_FEW_BLOBS", "TOO_FEW_VISIBLE", "NO_AP3P_FIT",
    // "RANSAC_FEW_INLIERS", "HIGH_RMS", "Z_OUT_OF_RANGE", "JUMP",
    // "WEAK_FIRST_LOCK".
    // Lifetime: points to a static string literal owned by the
    // solver; safe to copy or dereference without ownership tracking.
    // Used by the camera worker's periodic [psvr-cam] stderr summary
    // so users can tell at a glance why matched=0 frames are failing.
    const char* reject_reason{"OK"};

    // Projected image-space (u, v) of each of the 9 model LEDs under
    // the prior pose used for matching. Only meaningful where the
    // corresponding `visible[i]` is true.
    cv::Point2d projected[9]{};
    bool        visible[9]{};

    // For each visible LED i, the index into `blobs` it was matched
    // to (or -1 if unmatched). Indices are into the vector the caller
    // passed into solve(). Only meaningful where visible[i] is true.
    int         matched_blob_idx[9]{};
};

// Number of LEDs in the canonical model (all 9: 5 front cluster on the
// visor + 2 temples + 2 rear on the strap). Exposed so callers can
// size debug arrays consistently.
constexpr int NUM_LEDS = 9;

// Per-tracker solver state. Holds the last-accepted (rvec, tvec)
// used as the prior for matching/refinement on the next frame, plus
// the optional debug log handle. Owned by a Worker (one per tracker
// instance), so two simultaneous PSVRTracker instances would each
// have their own state - previously this was TU-globals which would
// have entangled them.
class SolverState {
public:
    SolverState();
    ~SolverState();

    SolverState(const SolverState&)            = delete;
    SolverState& operator=(const SolverState&) = delete;

    // Run the identification + PnP pipeline against this state. The
    // call may update the cached prior on success and rotates the log
    // file on first call. Thread-safety: a single SolverState should
    // be called from one thread at a time (the camera dispatch
    // queue). Internal mutexes guard the prior + log against test or
    // diagnostic accesses from other threads.
    //
    // blobs:         image-space centroids in pixels
    // img_w, img_h:  frame dimensions (for camera-intrinsics default)
    // yaw/pitch/roll_rad: IMU rotation prior in radians.
    // hfov_deg:      camera horizontal field of view, in degrees,
    //                used to build the pinhole intrinsics. Defaults
    //                to 70 deg when omitted, matching the legacy
    //                hard-coded value (so unit tests and any caller
    //                that doesn't supply HFOV behaves as before).
    Result solve(const std::vector<cv::Point2d>& blobs,
                 int img_w, int img_h,
                 double yaw_rad, double pitch_rad, double roll_rad,
                 double hfov_deg = 70.0);

    // Forward-declared and defined in the .cpp; Impl needs to be
    // declared at struct-level visibility (not nested-private) so
    // the file-local debug_log_fp helper in psvr_constellation.cpp
    // can take a reference to it. Holding the layout opaque is
    // already accomplished by unique_ptr<Impl>; consumers outside
    // the .cpp don't get the type definition either way.
    struct Impl;
private:
    std::unique_ptr<Impl> impl_;
};

} // namespace psvr_constellation
