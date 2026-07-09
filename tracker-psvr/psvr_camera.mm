/* PSVR camera-based LED constellation worker (implementation).
 *
 * Pipeline per frame (driven by AVFoundation's capture delegate on a
 * private dispatch queue):
 *
 *   1. Receive CMSampleBuffer, wrap its pixel buffer as a cv::Mat
 *      without copying (IOSurface-backed, planar BGRA).
 *   2. Extract bright blobs: convert to grayscale, threshold above
 *      BRIGHT_THRESH (~200/255), morph-open to kill speckle. Replaces
 *      the previous HSV-blue-gate + white-saturated double-mask;
 *      the LEDs are eye-searing bright (>=240 on a real LED, >200
 *      even with webcam AWB gain crushing), and "bright + roughly
 *      circular" is a strictly stronger signal than "bright + blue"
 *      because the latter depends on the camera's color rendition
 *      (UGREEN/PSCam/FaceTime all hue-shift PSVR blue differently).
 *   3. Find contours, compute centroid + area per contour. Drop tiny
 *      and gigantic ones (noise / lamp reflections).
 *   3b. Sub-pixel refine each blob's centre by mean-shifting on the
 *      grayscale ROI (intensity-weighted COM iteration, ported from
 *      tracker-pt). Gives ~5x lower PnP reprojection error than raw
 *      contour moments because the latter is biased by which side
 *      of the LED rasterised to one extra pixel.
 *   4. Pass the list of blob centroids to the constellation module
 *      along with the latest IMU rotation prior; it does LED-ID and
 *      solvePnP and returns a 6-DOF position if successful.
 *   5. Publish the result via atomic store of three doubles + a
 *      monotonic result_epoch. Consumers read_position() compares
 *      epoch to RESULT_STALE_SEC wall time and returns freshness.
 *
 * Notes on camera selection
 * -------------------------
 * The plugin exposes a "Camera" dropdown in the settings dialog
 * populated from compat/camera-names. The chosen name is pushed to
 * this worker via set_desired_camera_name() before start() runs. At
 * start() time we resolve it against the live AVCaptureDevice list:
 *
 *   1. If a non-empty preference is set, try to match by uniqueID
 *      first (covers future ini files written by a uniqueID-aware
 *      picker), then by localizedName (covers the current picker,
 *      which stores the human-readable name returned by
 *      compat/camera-names).
 *   2. If no preference is set OR no match is found, fall back to
 *      [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo],
 *      i.e. the legacy behavior. That's also what an empty ini
 *      key persists as, so users on an unmigrated profile get the
 *      same camera they had before this change.
 *
 * The enumeration goes through AVCaptureDeviceDiscoverySession because
 * the older devicesWithMediaType: API is deprecated on macOS 11+ and
 * the discovery session pulls in external USB cameras (deskViewCamera,
 * external) that the deprecated API misses on Apple Silicon.
 *
 * Notes on permissions
 * --------------------
 * The first frame triggers macOS's Camera TCC prompt if permission
 * hasn't been granted. If the user denies, AVCaptureSession fires
 * AVCaptureSessionRuntimeErrorNotification and we flip is_running_
 * back to false. The plugin gracefully falls back to IMU-only.
 */
#import "psvr_camera.h"
#import "psvr_constellation.h"
#import "ps4cam_firmware.h"

#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>
#import <CoreMedia/CoreMedia.h>
#import <CoreVideo/CoreVideo.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <mutex>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <algorithm>
#include <cctype>

namespace psvr_cam {

// Single source of truth for per-camera-type HFOV defaults. Matched
// case-insensitively as a substring of the camera's localizedName.
// These numbers live here and ONLY here; the dialog and the worker
// both call this function rather than carrying their own copies.
double recommended_hfov_for_camera(const std::string& localized_name) {
    std::string n = localized_name;
    std::transform(n.begin(), n.end(), n.begin(),
                   [](unsigned char c){ return (char)std::tolower(c); });
    auto has = [&](const char* needle) {
        std::string s(needle);
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c){ return (char)std::tolower(c); });
        return n.find(s) != std::string::npos;
    };
    if (has("ov580"))       return 85.0;  // PS4 Camera (OV580) per-lens HFOV
    if (has("playstation")) return 85.0;  // safety alias for PS Camera naming
    if (has("ugreen"))      return 80.0;  // UGREEN USB webcam
    if (has("facetime"))    return 78.0;  // Apple FaceTime HD
    if (has("macbook"))     return 78.0;  // built-in MacBook (Pro) lid camera
    return 70.0;                          // generic webcam default
}

static bool contains_ci(std::string n, const char* needle)
{
    std::transform(n.begin(), n.end(), n.begin(),
                   [](unsigned char c){ return (char)std::tolower(c); });
    std::string s(needle);
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return (char)std::tolower(c); });
    return n.find(s) != std::string::npos;
}

static bool is_ov580_camera(const std::string& localized_name)
{
    return contains_ci(localized_name, "ov580") ||
           contains_ci(localized_name, "playstation");
}

static bool ov580_eye_dims_for_raw(int rw, int& ew, int& ih)
{
    if (rw == 3448) { ew = 1280; ih = 800; return true; }
    if (rw == 1748) { ew =  640; ih = 400; return true; }
    if (rw ==  898) { ew =  320; ih = 200; return true; }
    return false;
}

static double max_fps_for_format(AVCaptureDeviceFormat* fmt)
{
    double best = 0.0;
    for (AVFrameRateRange* range in fmt.videoSupportedFrameRateRanges)
        best = std::max(best, range.maxFrameRate);
    return best;
}

static bool format_supports_fps(AVCaptureDeviceFormat* fmt, double fps)
{
    for (AVFrameRateRange* range in fmt.videoSupportedFrameRateRanges) {
        if (range.minFrameRate <= fps + 0.01 &&
            range.maxFrameRate + 0.01 >= fps) {
            return true;
        }
    }
    return false;
}

static bool frame_duration_for_fps(AVCaptureDeviceFormat* fmt,
                                   double fps,
                                   CMTime* duration)
{
    AVFrameRateRange* best = nil;
    double best_delta = std::numeric_limits<double>::infinity();
    for (AVFrameRateRange* range in fmt.videoSupportedFrameRateRanges) {
        if (range.minFrameRate <= fps + 0.01 &&
            range.maxFrameRate + 0.01 >= fps) {
            const double delta = std::abs(range.maxFrameRate - fps);
            if (delta < best_delta) {
                best = range;
                best_delta = delta;
            }
        }
    }
    if (!best)
        return false;
    // Use the exact UVC duration advertised by AVFoundation. Some
    // devices print "60.00 fps" but reject a synthesized 1/60 CMTime.
    *duration = best.minFrameDuration;
    return true;
}

// Freshness window for publish/consume. Camera runs at ~30 Hz and the
// solver is expected to accept at frame rate while locked (the old
// ~1 Hz accept cadence was a symptom of the frozen-rotation solve and
// the flat jump gate, both since fixed in psvr_constellation.cpp).
// This window now only bridges genuine reject bursts - occlusion, a
// hand in front of the visor, momentary blob dropout - so opentrack
// XYZ doesn't snap back to zero between locks. Still short enough
// that a real loss of tracking (helmet leaves the frame) falls back
// to IMU-only promptly.
static constexpr double RESULT_STALE_SEC = 2.0;

// Periodic [psvr-cam] stderr summary cadence (frames). At ~30 Hz this
// emits one line per second - low enough to read at a glance without
// flooding the console. Independent of the diag-log toggle so users
// can verify the camera is actually seeing blobs / solving PnP from
// the same console they use to watch USB HID startup, without first
// enabling the verbose constellation log file.
static constexpr int PSVR_CAM_LOG_INTERVAL_FRAMES = 30;

static bool frame_dumps_enabled()
{
    static const bool enabled = [] {
        const char* v = std::getenv("PSVR_CAM_DUMP_FRAMES");
        return v && *v && !(v[0] == '0' && v[1] == '\0');
    }();
    return enabled;
}

static double requested_ov580_fps()
{
    static const double fps = [] {
        const char* v = std::getenv("PSVR_CAM_FPS");
        if (!v || !*v)
            return 0.0;
        char* end = nullptr;
        const double parsed = std::strtod(v, &end);
        return (end != v && parsed >= 1.0) ? parsed : 0.0;
    }();
    return fps;
}

// Minimum / maximum blob area in pixels for the extractor. The PSVR
// LEDs at ~60-100 cm from a 720p camera at ~70deg HFOV render as
// roughly 3-25 px diameter, except the front-center visor LED which
// saturates into a wide bar on the OV580. The lower bound rejects
// single-pixel speckle and the morphological-opening leftovers that
// would otherwise inflate the candidate set into the 20+ range; the
// upper bound rejects bright spread-out areas like a ceiling lamp or
// a window in frame while keeping that center bar. 5 px^2 is small
// enough to keep a 2-3 px LED at ~150 cm range while still dropping
// the salt-and-pepper noise that every webcam produces after grayscale
// thresholding.
// DIAGNOSTIC v2 dropped this to 1.0 to admit even single-pixel LED dots.
// Raised to 20: the re-enabled circularity gate kills ELONGATED window-
// blind slats, but a few small near-square slat fragments still score
// high circularity (measured area 4-18 px, circ 0.75-0.84) and would
// slip through. Genuine LED blobs on the same capture were 115-300 px,
// so a 20 px floor drops those fragments with wide margin while still
// keeping a ~6 px-diameter LED for longer-range tracking. Pair with
// BLOB_MIN_CIRCULARITY below: area kills the small round fragments,
// circularity kills the large elongated slats.
static constexpr double BLOB_MIN_AREA_PX = 20.0;
static constexpr double BLOB_MAX_AREA_PX = 1500.0;

// Minimum 4*PI*A/P^2 circularity. A geometric circle is 1.0; a real
// PSVR LED (with a tiny bit of motion blur / partial saturation tail)
// ranges 0.7-0.95. Highlights from room edges (monitor bezels, table
// edges, glasses frames, the helmet's own metal trim) are elongated
// and score 0.2-0.4. 0.55 was the original tight value; DIAGNOSTIC v2
// disabled the gate (0.0) so reflections off helmet plastic, partial
// rim LEDs at off-axis poses, and pixelated dots all pass.
//
// Re-enabled at 0.45: field testing with a PS4 Camera facing a window
// showed daylight through venetian-blind slats producing a cluster of
// ELONGATED bright blobs in one image corner. With the gate off they
// reached the matcher, which latched onto them as if they were front-
// left LEDs - dragging the solved head position ~35 cm off axis (a
// free-rotation PnP fit that still reprojected at acceptable RMS).
// Measured on a real capture (de-interleaved OV580 left eye): the slats
// score 0.20-0.41 circularity, the genuine LED blobs 0.52-0.65. 0.45
// sits below the lowest measured LED so partially-saturated / motion-
// blurred cores survive, while every slat fragment is rejected. Note the
// OV580's center visor LED can saturate into a wider bar at some poses;
// 0.45 leaves headroom for that without re-admitting the slats.
static constexpr double BLOB_MIN_CIRCULARITY = 0.45;

// Max blobs handed to the constellation solver. The PSVR has 9 LEDs and
// at most 5-7 are visible at once, so 16 leaves generous headroom for a
// few spurious bright spots while bounding the solver's permutation-
// RANSAC combinatorics. Blobs beyond the 16 brightest are dropped
// (see the brightness sort in process_frame). Matches tracker-pt's
// max_blobs cap.
static constexpr int BLOB_POOL_MAX = 16;

// Grayscale brightness threshold for the bright-blob mask is now
// chosen ADAPTIVELY per frame from the gray histogram (see
// adaptive_bright_threshold below). The principle: PSVR LEDs are
// reliably the brightest things in a normally-lit room, so we walk
// down from V=255 until we've captured enough bright pixels for
// ~9 LED blobs (N_TARGET_BRIGHT_PIXELS), then threshold there.
// Adapts to dim rooms (LEDs at V~210), bright rooms (V~250), or
// any auto-exposure shift between frames. The chosen value is
// surfaced in the [psvr-cam] periodic log as `thresh=NNN`.
//
// FLOOR/CEIL clamp the search range so:
//   - Hyper-bright scenes (sun on the wall, max=255 over half the
//     frame) don't pin the threshold to 255 and lose the LEDs.
//   - Hyper-dim scenes (no LEDs visible at all) don't drive the
//     threshold down into ordinary mid-grey territory.
// N_TARGET = 1500 matches roughly 9 LEDs * pi * (7px)^2 = 1385 px,
// rounded up for blur halos.
static constexpr int BRIGHT_THRESH_FLOOR = 180;
static constexpr int BRIGHT_THRESH_CEIL  = 250;
// Dropped from 1500 to 100 because field testing showed a bright
// monitor/window in-frame eats most of the 1500-pixel budget,
// landing the threshold low enough that LED *halos* fall below it
// while their saturated cores barely scrape over - and 2-pixel-core
// LEDs are then indistinguishable from sensor hot-pixel noise that
// also sits just above the threshold. Targeting the top 100 brightest
// pixels lands the threshold near V=250+, which captures only the
// actual LED cores and the very brightest tip of any monitor/lamp
// (typically <100 pixels' worth of saturated-white). Combined with
// MORPH_DILATE below, each LED's 2-px core becomes a 4-px blob the
// area gate accepts.
// Lowered again from 100 to 40 after field-testing showed 100 still
// admitted ~35 noise/sensor-hot pixels per frame on top of the ~5
// real LED cores - the matcher would then find two geometrically
// valid solutions (~5cm and ~45cm Z) and oscillate. 40 lands the
// threshold near V=250+, capturing only the very tip of the LED
// signal and the few brightest noise pixels. With dilate(3x3) below,
// each LED core grows into a clean 4-6 px blob; noise pixels
// become isolated 3x3 blobs that the matcher filters out via
// inlier scoring far more reliably at signal/noise = 5/(5+5) = 50%
// vs the old 5/40 = 12%.
static constexpr int N_TARGET_BRIGHT_PIXELS = 40;

// Blue chroma-key rescue channel: chroma = B - 0.5*(G + R), the
// tracker-pt continuous blue key (point_extractor.cpp filter_single_
// channel, blue case). OR-ed with the luma brightness mask above.
//
// Why a rescue channel at all: the adaptive LUMA threshold has a hard
// failure mode - anything in frame brighter than the LED cores (sunlit
// wall, window, monitor, lamp) contributes >= N_TARGET_BRIGHT_PIXELS
// of near-255 pixels, pins the luma threshold at its ceiling, and the
// dimmer LED cores fall below it, so blobs vanish and the solver never
// locks. PSVR LEDs are strongly BLUE while those interferers are
// white-ish (B ~= G ~= R -> chroma ~ 0), so the chroma channel
// isolates the LEDs regardless of how bright the white background is.
//
// Upgraded from a FIXED cut (B - max(R,G) > 64) to a CONTINUOUS channel
// fed through the same top-N adaptive threshold used for luma, because
// PSVR blue renders at very different chroma levels across cameras
// (UGREEN / FaceTime / OV580) - a single fixed cut is either noisy on
// one camera or misses the LEDs on another; the adaptive walk tracks
// the actual LED chroma level per frame. The floor keeps it above
// webcam AWB chroma noise on white/grey scenes (empirically < ~20) so
// a blue-free scene never drops the threshold into noise; the ceiling
// bounds strongly-blue cameras. LED chroma is typically >= 120
// (B>=200, G/R<=40..80), comfortably inside the band.
//
// The union with the luma mask is what makes this regression-safe:
// bloomed-white LED cores (B=G=R=255 -> chroma 0) are still caught by
// luma, so adding the chroma channel can only ADD detections, never
// remove the ones the brightness path already finds.
static constexpr int CHROMA_THRESH_FLOOR = 40;
static constexpr int CHROMA_THRESH_CEIL  = 200;

// Pick a threshold by walking a single-channel histogram from 255 down
// until the cumulative pixel count reaches `target`, clamped to
// [floor, ceil]. O(W*H) for the histogram + O(256) for the walk;
// ~0.5 ms at 1280x720. No state - next frame's threshold is
// independent, so a momentary occlusion can't pin a bad value.
static int adaptive_topN_threshold(const cv::Mat& chan, int target,
                                   int floor_v, int ceil_v) {
    int hist_size = 256;
    float range[] = {0.f, 256.f};
    const float* hist_range = range;
    cv::Mat hist;
    cv::calcHist(&chan, 1, nullptr, cv::Mat(), hist, 1,
                 &hist_size, &hist_range);
    int cum = 0;
    for (int v = 255; v >= 0; --v) {
        cum += static_cast<int>(hist.at<float>(v));
        if (cum >= target)
            return std::max(floor_v, std::min(ceil_v, v));
    }
    return floor_v;
}

static int adaptive_bright_threshold(const cv::Mat& gray) {
    return adaptive_topN_threshold(gray, N_TARGET_BRIGHT_PIXELS,
                                   BRIGHT_THRESH_FLOOR, BRIGHT_THRESH_CEIL);
}

// Mean-shift kernel-radius multiplier. Matches tracker-pt's
// radius_c constant: the kernel is sized to the LED's footprint
// (~sqrt(area/pi)) and scaled up by this factor so it covers a
// neighbourhood big enough to "pull" the iteration toward the
// brightest part of the LED even when the initial contour-centroid
// estimate is biased by an extra rasterised pixel on one side.
// 1.75 is what tracker-pt found "smaller values mean more changes;
// 1 makes too many changes while 1.5 makes about .1".
static constexpr double MEAN_SHIFT_RADIUS_C = 1.75;
static constexpr int    MEAN_SHIFT_MAX_ITERS = 5;

static double now_sec() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

/* ------------------------------------------------------------------ *
 * Mean-shift sub-pixel centering, ported verbatim from
 * tracker-pt/module/point_extractor.cpp. The original carries this
 * license header (MIT-equivalent) which travels with the code:
 *
 *   Copyright (c) 2012 Patrick Ruoff
 *   Copyright (c) 2015-2017 Stanislaw Halik <sthalik@misaki.pl>
 *
 *   Permission to use, copy, modify, and/or distribute this software for any
 *   purpose with or without fee is hereby granted, provided that the above
 *   copyright notice and this permission notice appear in all copies.
 *
 * Algorithm (paraphrased from the tracker-pt comment): a moving
 * kernel is multiplied with the gray-scale image and the centre of
 * mass of the result is computed. The kernel centre is then set to
 * the previous COM and the process is iterated until the kernel
 * stops moving. Peaks in image intensity "pull" the kernel toward
 * themselves. Eliminates the rasterisation bias of a pure contour-
 * centroid estimate (the centroid can only move in 1-pixel steps as
 * threshold-area boundary pixels are added/removed; mean-shift on
 * the underlying grayscale image moves in sub-pixel steps).
 *
 * Returns the refined centre; if the integrated weight m is too
 * small (e.g. the kernel landed on a uniformly-dark patch), returns
 * the input current_center unchanged so the caller's loop terminates
 * gracefully.
 * ------------------------------------------------------------------ */
static cv::Point2d MeanShiftIteration(const cv::Mat1b& frame_gray,
                                      const cv::Point2d& current_center,
                                      double filter_width)
{
    const double s = 1.0 / filter_width;
    double m = 0.0;
    cv::Point2d com{0.0, 0.0};
    for (int i = 0; i < frame_gray.rows; ++i)
    {
        const uint8_t* __restrict ptr = frame_gray.ptr(i);
        for (int j = 0; j < frame_gray.cols; ++j)
        {
            double val = (double)ptr[j];
            val = val * val;  // square so brighter parts dominate
            const double dx = (j - current_center.x) * s;
            const double dy = (i - current_center.y) * s;
            const double max_ = std::fmax(0.0, 1.0 - dx * dx - dy * dy);
            val *= max_;
            m += val;
            com.x += j * val;
            com.y += i * val;
        }
    }
    if (m > 0.1)
    {
        com.x /= m;
        com.y /= m;
        return com;
    }
    return current_center;
}

} // namespace psvr_cam

// -- Objective-C delegate, lives for the lifetime of a Worker::Impl ---
@interface PSVRCaptureDelegate : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate>
@property (nonatomic, assign) psvr_cam::Worker::Impl* owner;
@end

namespace psvr_cam {

// ---- private state ---------------------------------------------------
struct Worker::Impl {
    // AVFoundation objects retained for the session lifetime.
    AVCaptureSession*          session{nil};
    AVCaptureDeviceInput*      input{nil};
    AVCaptureVideoDataOutput*  output{nil};
    dispatch_queue_t           queue{nullptr};
    PSVRCaptureDelegate*       delegate{nil};
    // NSNotificationCenter token for the AVCaptureSessionRuntimeError
    // observer registered in start() (TCC denial, device unplug,
    // media-services reset). Removed in stop().
    id                         runtime_err_observer{nil};

    std::atomic<bool> running{false};

    // One-shot latch for the width-based OV580 auto-HFOV correction in
    // process_frame (see there). Touched only on the capture queue.
    bool ov580_hfov_checked{false};

    // Atomic rotation prior published by the HID thread.
    std::atomic<double> yaw_rad{0}, pitch_rad{0}, roll_rad{0};

    // Last-known position + freshness timestamp, published via a
    // seqlock so the multi-double tuple is read atomically. Tear-free
    // properties:
    //   Writer: pos_seq.fetch_add(1, acq_rel)  -- becomes ODD,
    //           "in-progress"; mutates the doubles; pos_seq.fetch_add(1)
    //           -- becomes EVEN, "stable".
    //   Reader: spin-loads pos_seq twice with acquire fences and
    //           rejects when either is odd or they differ; otherwise
    //           the doubles read between them belong to the same
    //           writer-publish atomically.
    // Seqlock is wait-free for the writer and contention-free in the
    // common case for the reader. Previous design with separate
    // relaxed atomics could let a reader see x from frame N+1 and y/z
    // from frame N - actual write tearing.
    // The payload fields are std::atomic with RELAXED accesses (the
    // seq bracket + fences provide the ordering): plain doubles under
    // a seqlock are still a formal C++ data race, and the reader also
    // needs an acquire fence BEFORE the validating re-read of pos_seq
    // or the payload loads may be reordered past it. See Boehm, "Can
    // seqlocks get along with programming language memory models?".
    std::atomic<uint64_t> pos_seq{0};
    std::atomic<double>   pos_x_cm{0}, pos_y_cm{0}, pos_z_cm{0};
    std::atomic<double>   result_epoch{0};      // monotonic wall time
    std::atomic<bool>     pnp_ok_latest{false};

    // Diag counters, guarded by a mutex only during bulk copy.
    mutable std::mutex diag_mu;
    Diag diag{};

    // Reusable work buffers; owned by the delegate thread.
    cv::Mat bgra_view;      // zero-copy wrapper of the incoming frame
    cv::Mat bgr_owned;      // owned BGR copy (lives past IOSurface unlock)
    cv::Mat gray;           // single-channel grayscale of bgr_owned;
                            // input to both the brightness threshold and
                            // the mean-shift sub-pixel refinement pass
    cv::Mat mask;

    // Preview buffer: BGR annotated copy of the latest capture, with
    // overlays drawn by process_frame (blob centroids, projected LED
    // positions, match lines, status text). Mutex-guarded so the Qt
    // preview-dialog poller can safely copy it out from the main
    // thread while capture writes it from the camera dispatch queue.
    mutable std::mutex preview_mu;
    cv::Mat preview_bgr;    // CV_8UC3, same resolution as capture

    // Status banner (multi-line, possibly empty) painted on every
    // preview frame. Set from the plugin's GUI thread when calibration
    // state changes. Read under the same mutex by process_frame on the
    // camera dispatch queue. Color is BGR (OpenCV order) so we don't
    // have to swap on the hot path.
    mutable std::mutex banner_mu;
    std::string        banner_text;
    cv::Scalar         banner_color_bgr{255, 255, 255}; // default white

    // Per-instance constellation solver state (replaces what used to
    // be TU-globals in psvr_constellation.cpp). Holds the last-
    // accepted pose for prior-based matching and the optional debug
    // log file. One per Worker = one per tracker session.
    psvr_constellation::SolverState solver_state;

    // User-selected camera (localizedName or uniqueID; matched both
    // ways at start time). Empty = legacy default-device behavior.
    // Written from the Qt UI thread before start(); read only in
    // start() on the camera thread, never afterwards.
    std::string desired_camera_name;

    // Camera horizontal FOV (degrees) used to build the constellation
    // solver's pinhole intrinsics. Default 70 reproduces the legacy
    // hard-coded behavior. Written from the Qt UI thread (start +
    // hot-apply on dialog valueChanged); read from the camera
    // dispatch queue every frame. Atomic so the cross-thread store/
    // load is well-defined without needing a mutex on the hot path.
    std::atomic<double> desired_hfov_deg{70.0};

    // Auto-HFOV mode. When true, start() overrides desired_hfov_deg
    // with recommended_hfov_for_camera() for the resolved device.
    // Written from the Qt UI thread before start(); read in start()
    // on the camera thread. Atomic for a well-defined cross-thread
    // store/load. Default true (most users want HFOV to just work).
    std::atomic<bool> desired_hfov_auto{true};
};

Worker::Worker() : impl_(std::make_unique<Impl>()) {}

Worker::~Worker() { stop(); }

bool Worker::is_running() const { return impl_->running.load(); }

void Worker::set_desired_camera_name(const std::string& s) {
    impl_->desired_camera_name = s;
}

void Worker::set_hfov_deg(double hfov_deg) {
    // Defensive clamp matching the dialog's spinbox range. Anything
    // outside 40..130 is implausible for a head-tracking webcam and
    // would produce nonsensical solvePnP intrinsics.
    if (hfov_deg < 40.0)  hfov_deg = 40.0;
    if (hfov_deg > 130.0) hfov_deg = 130.0;
    impl_->desired_hfov_deg.store(hfov_deg, std::memory_order_relaxed);
}

void Worker::set_hfov_auto(bool enabled) {
    impl_->desired_hfov_auto.store(enabled, std::memory_order_relaxed);
}

void Worker::set_rotation_prior(double yaw, double pitch, double roll) {
    impl_->yaw_rad.store(yaw, std::memory_order_relaxed);
    impl_->pitch_rad.store(pitch, std::memory_order_relaxed);
    impl_->roll_rad.store(roll, std::memory_order_relaxed);
}

bool Worker::get_position(double* x, double* y, double* z) const {
    // Seqlock read: spin until we get two equal even seq values
    // bracketing a clean read of the doubles. Bounded retry count -
    // a writer that finishes a publish in <100 ns can starve us
    // briefly but never indefinitely. If we're contended for many
    // iterations the writer is publishing far faster than expected
    // (impossible at 30 Hz camera rate), so just bail.
    for (int tries = 0; tries < 8; ++tries) {
        const uint64_t s1 = impl_->pos_seq.load(std::memory_order_acquire);
        if (s1 & 1u) continue;          // mid-write
        const double  epoch  = impl_->result_epoch.load(std::memory_order_relaxed);
        const double  x_v    = impl_->pos_x_cm.load(std::memory_order_relaxed);
        const double  y_v    = impl_->pos_y_cm.load(std::memory_order_relaxed);
        const double  z_v    = impl_->pos_z_cm.load(std::memory_order_relaxed);
        // Fence BEFORE the validating re-read: without it the payload
        // loads above may sink past s2 and the tear check is unsound.
        std::atomic_thread_fence(std::memory_order_acquire);
        const uint64_t s2 = impl_->pos_seq.load(std::memory_order_relaxed);
        if (s1 != s2) continue;         // writer ran during our read
        if (epoch == 0.0) return false;
        if (now_sec() - epoch > RESULT_STALE_SEC) return false;
        *x = x_v;
        *y = y_v;
        *z = z_v;
        return true;
    }
    return false;
}

Diag Worker::diag() const {
    std::lock_guard<std::mutex> lk(impl_->diag_mu);
    return impl_->diag;
}

void Worker::set_status_banner(const std::string& text,
                               int r, int g, int b) {
    std::lock_guard<std::mutex> lk(impl_->banner_mu);
    impl_->banner_text       = text;
    impl_->banner_color_bgr  = cv::Scalar(b, g, r);
}

bool Worker::fetch_preview_rgb(std::vector<uint8_t>* out,
                               int* out_w, int* out_h) const {
    // Take a shallow Mat copy under the lock - cv::Mat is refcounted,
    // so this is O(1) (just a pointer + refcount bump). Then drop the
    // lock and do the actual ~3 MB pixel walk outside it. The next
    // process_frame writer can replace impl_->preview_bgr concurrently;
    // its move-assign decrements the old refcount but our `local`
    // still holds a reference, so the underlying pixel data stays
    // alive until we're done reading from it.
    cv::Mat local;
    {
        std::lock_guard<std::mutex> lk(impl_->preview_mu);
        if (impl_->preview_bgr.empty()) return false;
        local = impl_->preview_bgr;  // shallow refcount-bump
    }
    const int w = local.cols;
    const int h = local.rows;
    out->resize((size_t)w * h * 3);
    // BGR -> RGB byte swap. We do it inline rather than via cvtColor
    // because we'd have to allocate a temporary Mat anyway and the
    // explicit loop avoids that.
    for (int y = 0; y < h; ++y) {
        const uint8_t* src = local.ptr<uint8_t>(y);
        uint8_t*       dst = out->data() + (size_t)y * w * 3;
        for (int x = 0; x < w; ++x) {
            dst[3 * x + 0] = src[3 * x + 2]; // R <- B
            dst[3 * x + 1] = src[3 * x + 1]; // G
            dst[3 * x + 2] = src[3 * x + 0]; // B <- R
        }
    }
    *out_w = w;
    *out_h = h;
    return true;
}

// Resolve the user's camera preference to a concrete AVCaptureDevice.
// Matching strategy (uniqueID > localizedName > default) is documented
// in the file header. Returns nil only if there is no usable video
// camera attached at all.
static AVCaptureDevice* pick_camera(const std::string& desired)
{
    AVCaptureDevice* fallback =
        [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];

    if (desired.empty()) {
        if (fallback) {
            std::fprintf(stderr,
                "[psvr-cam] using default video device: %s (%s)\n",
                fallback.localizedName.UTF8String,
                fallback.uniqueID.UTF8String);
        }
        return fallback;
    }

    NSString* want = [NSString stringWithUTF8String:desired.c_str()];

    // AVCaptureDeviceDiscoverySession with a fairly inclusive device-
    // type list. builtInWideAngleCamera covers the FaceTime lid cam,
    // `external` (macOS 14+) covers USB webcams and the PS Camera,
    // deskViewCamera covers Continuity Camera "Desk View" on macOS
    // 13+. Older systems fall through with whatever subset of the
    // list is available; AVCaptureDeviceDiscoverySession ignores
    // unknown device-type identifiers gracefully.
    NSMutableArray<AVCaptureDeviceType>* types = [NSMutableArray array];
    [types addObject:AVCaptureDeviceTypeBuiltInWideAngleCamera];
    if (@available(macOS 14.0, *)) {
        [types addObject:AVCaptureDeviceTypeExternal];
    } else {
        // Pre-macOS 14, external USB cameras (PS Camera, Logitech,
        // etc.) come through as AVCaptureDeviceTypeExternalUnknown.
        // Use the literal NSString to keep building on SDKs that
        // have already removed the symbol from their headers.
        [types addObject:(AVCaptureDeviceType)@"AVCaptureDeviceTypeExternalUnknown"];
    }
    if (@available(macOS 13.0, *)) {
        [types addObject:AVCaptureDeviceTypeDeskViewCamera];
    }

    AVCaptureDeviceDiscoverySession* ds =
        [AVCaptureDeviceDiscoverySession
            discoverySessionWithDeviceTypes:types
                                  mediaType:AVMediaTypeVideo
                                   position:AVCaptureDevicePositionUnspecified];

    AVCaptureDevice* by_uid  = nil;
    AVCaptureDevice* by_name = nil;
    for (AVCaptureDevice* d in ds.devices) {
        if (!by_uid  && [d.uniqueID      isEqualToString:want]) by_uid  = d;
        if (!by_name && [d.localizedName isEqualToString:want]) by_name = d;
    }

    if (by_uid) {
        std::fprintf(stderr,
            "[psvr-cam] matched preferred camera by uniqueID: %s (%s)\n",
            by_uid.localizedName.UTF8String, by_uid.uniqueID.UTF8String);
        return by_uid;
    }
    if (by_name) {
        std::fprintf(stderr,
            "[psvr-cam] matched preferred camera by name: %s (%s)\n",
            by_name.localizedName.UTF8String, by_name.uniqueID.UTF8String);
        return by_name;
    }

    std::fprintf(stderr,
        "[psvr-cam] preferred camera \"%s\" not found among %lu device(s); "
        "falling back to default\n",
        desired.c_str(), (unsigned long)ds.devices.count);
    if (fallback) {
        std::fprintf(stderr,
            "[psvr-cam] using default video device: %s (%s)\n",
            fallback.localizedName.UTF8String,
            fallback.uniqueID.UTF8String);
    }
    return fallback;
}

bool Worker::start() {
    if (impl_->running.load()) return true;

    // If a PS4 Camera is plugged in but still in OV580 Boot Mode, ship
    // the firmware and wait for it to re-enumerate as a UVC device
    // before we ask AVFoundation what cameras are connected. Without
    // this, the device is invisible to pick_camera() below and the
    // user's "PS4 Camera" selection silently falls back to the
    // default camera. Idempotent and cheap (silent no-op) when no
    // PS4 Camera is present, which is the common case.
    (void)ps4cam::ensure_firmware_uploaded();

    @autoreleasepool {
        AVCaptureDevice* dev = pick_camera(impl_->desired_camera_name);
        if (!dev) {
            std::fprintf(stderr, "[psvr-cam] no camera device available\n");
            return false;
        }

        // Auto-HFOV: now that the actual capture device is resolved (the
        // only place its localizedName is known), override the manual
        // HFOV with the per-camera-type recommendation. Reuses the
        // existing clamp inside set_hfov_deg. When auto is off we leave
        // desired_hfov_deg as the dialog's manual spinbox value.
        if (impl_->desired_hfov_auto.load(std::memory_order_relaxed)) {
            const double auto_hfov = recommended_hfov_for_camera(
                dev.localizedName.UTF8String);
            set_hfov_deg(auto_hfov);
            std::fprintf(stderr,
                "[psvr-cam] auto HFOV: %.1f deg for \"%s\"\n",
                auto_hfov, dev.localizedName.UTF8String);
        }

        NSError* err = nil;
        AVCaptureDeviceInput* input =
            [AVCaptureDeviceInput deviceInputWithDevice:dev error:&err];
        if (!input) {
            std::fprintf(stderr,
                "[psvr-cam] camera input init failed: %s\n",
                err ? err.localizedDescription.UTF8String : "unknown");
            return false;
        }

        AVCaptureSession* session = [[AVCaptureSession alloc] init];
        // Don't hard-code AVCaptureSessionPreset1280x720: the PS4 Camera
        // (OV580 stereo cam) has no 1280x720 mode - its native formats
        // are 1748x408, 896x500, 640x400, 320x192. Forcing a non-
        // native preset produced misaligned BGRA frames that look like
        // a diagonal-streak "weave" in the preview. Prefer the camera's
        // own format negotiation: try the standard "High" preset (which
        // means "best resolution this device supports"), and if even
        // that doesn't fit, fall back to whatever the device picks.
        if ([session canSetSessionPreset:AVCaptureSessionPresetHigh])
            session.sessionPreset = AVCaptureSessionPresetHigh;
        else if ([session canSetSessionPreset:AVCaptureSessionPresetMedium])
            session.sessionPreset = AVCaptureSessionPresetMedium;
        // Otherwise leave sessionPreset at its default and let the
        // device's activeFormat decide.

        if (![session canAddInput:input]) {
            std::fprintf(stderr, "[psvr-cam] session refused camera input\n");
            return false;
        }
        [session addInput:input];

        // Picky-camera activeFormat selection. The OV580 (PS4 Camera)
        // reports raw stereo container widths (898/1748/3448); the
        // decoder below extracts the left-eye image from those as
        // 320x200 / 640x400 / 1280x800. Prefer the 640x400 eye mode.
        // A higher OV580 frame rate can be forced for experiments by
        // launching with PSVR_CAM_FPS=60, but the default leaves frame
        // duration negotiation to AVFoundation because some adapters
        // stall after a few frames when hard-pinned.
        //
        // We pick by (a) sensible dimensions for tracking (target
        // 640 wide, prefer <= 1280) and (b) avoiding tile-packed
        // modes (height < 200 is a give-away that the format is
        // sub-lens / non-image).
        {
            const char* dev_name_utf8 = dev.localizedName.UTF8String;
            const std::string dev_name = dev_name_utf8 ? dev_name_utf8 : "";
            const bool ov580 = is_ov580_camera(dev_name);
            const double target_fps = ov580 ? requested_ov580_fps() : 0.0;
            AVCaptureDeviceFormat* best = nil;
            int                    best_score = -1;
            for (AVCaptureDeviceFormat* fmt in dev.formats) {
                CMVideoDimensions dim =
                    CMVideoFormatDescriptionGetDimensions(fmt.formatDescription);
                int score = -1;
                if (ov580) {
                    int eye_w = 0, eye_h = 0;
                    if (!ov580_eye_dims_for_raw(dim.width, eye_w, eye_h))
                        continue;
                    if (target_fps > 0.0 &&
                        !format_supports_fps(fmt, target_fps))
                        continue;
                    const int target_w = 640, target_h = 400;
                    score = 100000
                            - std::abs(eye_w - target_w) * 10
                            - std::abs(eye_h - target_h) * 10
                            + (int)std::round(max_fps_for_format(fmt));
                } else {
                    if (dim.height < 200) continue;  // skip sub-lens / weird
                    if (dim.width > 1280) continue;  // skip wide stereo packs
                    const int target_w = 640, target_h = 400;
                    score = 10000
                            - std::abs(dim.width  - target_w)
                            - std::abs(dim.height - target_h);
                }
                if (score > best_score) {
                    best_score = score;
                    best       = fmt;
                }
            }
            if (best && [dev lockForConfiguration:nil]) {
                dev.activeFormat = best;
                if (target_fps > 0.0) {
                    CMTime frame_duration = kCMTimeInvalid;
                    if (frame_duration_for_fps(best, target_fps,
                                               &frame_duration)) {
                        dev.activeVideoMinFrameDuration = frame_duration;
                        dev.activeVideoMaxFrameDuration = frame_duration;
                    }
                }
                [dev unlockForConfiguration];
            }
        }

        // Log what the device actually settled on so we can debug
        // pixel-format / dimension mismatches without a debugger.
        {
            AVCaptureDeviceFormat* fmt = dev.activeFormat;
            CMVideoDimensions dim = CMVideoFormatDescriptionGetDimensions(
                fmt.formatDescription);
            FourCharCode codec = CMFormatDescriptionGetMediaSubType(
                fmt.formatDescription);
            char c[5] = {
                (char)((codec >> 24) & 0xff), (char)((codec >> 16) & 0xff),
                (char)((codec >> 8) & 0xff),  (char)(codec & 0xff), 0};
            std::fprintf(stderr,
                "[psvr-cam] active format: %dx%d fourcc='%s' target_fps=%.1f\n",
                dim.width, dim.height, c,
                dev.activeVideoMinFrameDuration.timescale > 0
                    ? (double)dev.activeVideoMinFrameDuration.timescale /
                      (double)dev.activeVideoMinFrameDuration.value
                    : 0.0);
        }

        AVCaptureVideoDataOutput* output = [[AVCaptureVideoDataOutput alloc] init];
        output.alwaysDiscardsLateVideoFrames = YES;
        // Ask for packed BGRA so OpenCV can wrap the IOSurface directly
        // without a color-space conversion pass.
        // Pixel format. We used to ask for 32BGRA which let macOS's
        // Core Video subsystem do the YUV->BGRA conversion - but that
        // converter has a SIMD fast path that assumes width is a
        // multiple of 16, and produces diagonal-streak garbage on the
        // PS4 Camera (OV580) whose native modes are 1748x408 and
        // 896x500 (neither a multiple of 16). Asking for the camera's
        // native YUV422 ('yuvs') format and doing the conversion
        // ourselves with cv::cvtColor sidesteps that bug for any
        // odd-width camera. Normal webcams (UGREEN, FaceTime, etc.)
        // also offer yuvs natively, so this is a no-cost change for
        // them.
        output.videoSettings = @{
            (id)kCVPixelBufferPixelFormatTypeKey :
                @(kCVPixelFormatType_422YpCbCr8_yuvs)
        };

        dispatch_queue_t q = dispatch_queue_create(
            "com.opentrack.psvr-camera", DISPATCH_QUEUE_SERIAL);
        PSVRCaptureDelegate* del = [[PSVRCaptureDelegate alloc] init];
        del.owner = impl_.get();
        [output setSampleBufferDelegate:del queue:q];
        if (![session canAddOutput:output]) {
            std::fprintf(stderr, "[psvr-cam] session refused data output\n");
            return false;
        }
        [session addOutput:output];

        // Retain into the Impl (ARC keeps them alive because they're
        // assigned to strong properties).
        impl_->session  = session;
        impl_->input    = input;
        impl_->output   = output;
        impl_->queue    = q;
        impl_->delegate = del;

        // Runtime-error watchdog. TCC permission denial, device unplug
        // mid-session, and media-services resets all surface as
        // AVCaptureSessionRuntimeErrorNotification. Flip `running`
        // false so is_running()/get_position() consumers fall back to
        // IMU-only instead of holding the last position against a dead
        // camera forever. (The permissions note at the top of this
        // file promised this observer; it previously didn't exist.)
        // The block captures the raw Impl*, which outlives the
        // observer: stop() removes the observer before ~Worker
        // destroys the Impl.
        Worker::Impl* impl_raw = impl_.get();
        impl_->runtime_err_observer =
            [[NSNotificationCenter defaultCenter]
                addObserverForName:AVCaptureSessionRuntimeErrorNotification
                            object:session
                             queue:nil
                        usingBlock:^(NSNotification* note) {
                NSError* e = note.userInfo[AVCaptureSessionErrorKey];
                std::fprintf(stderr,
                    "[psvr-cam] capture session runtime error: %s\n",
                    e ? e.localizedDescription.UTF8String : "unknown");
                impl_raw->running.store(false);
            }];

        [session startRunning];
        impl_->running.store(true);
        std::fprintf(stderr, "[psvr-cam] camera started: %s (%s)\n",
            dev.localizedName.UTF8String,
            dev.uniqueID.UTF8String);
        return true;
    }
}

void Worker::stop() {
    // No early-return on !running: the runtime-error observer flips
    // `running` false on a dead session, and the old guard then
    // skipped the actual teardown (session, observer, delegate) -
    // leaking them until process exit. Every step below is nil-guarded
    // and idempotent, so calling stop() twice is harmless.
    @autoreleasepool {
        if (impl_->runtime_err_observer) {
            [[NSNotificationCenter defaultCenter]
                removeObserver:impl_->runtime_err_observer];
            impl_->runtime_err_observer = nil;
        }
        if (impl_->session) {
            [impl_->session stopRunning];
            impl_->session = nil;
        }
        impl_->input    = nil;
        impl_->output   = nil;
        impl_->delegate = nil;
        impl_->queue    = nullptr; // ARC: dispatch queues released here
    }
    impl_->running.store(false);
}

// Called from the capture-delegate dispatch queue only.
static void process_frame(Worker::Impl* s, CVPixelBufferRef buf) {
    CVPixelBufferLockBaseAddress(buf, kCVPixelBufferLock_ReadOnly);
    const int w = (int)CVPixelBufferGetWidth(buf);
    const int h = (int)CVPixelBufferGetHeight(buf);
    const int stride = (int)CVPixelBufferGetBytesPerRow(buf);
    uint8_t* base = (uint8_t*)CVPixelBufferGetBaseAddress(buf);

    // Diagnostic: log the actual pixel format of incoming frames, once.
    // AVFoundation sometimes silently ignores videoSettings and delivers
    // a different format; this log line tells us conclusively what
    // arrived so we don't try to interpret 32BGRA as YUYV (or vice versa).
    {
        static bool announced = false;
        if (!announced) {
            announced = true;
            OSType fmt = CVPixelBufferGetPixelFormatType(buf);
            char c[5] = {
                (char)((fmt >> 24) & 0xff), (char)((fmt >> 16) & 0xff),
                (char)((fmt >> 8) & 0xff),  (char)(fmt & 0xff), 0};
            std::fprintf(stderr,
                "[psvr-cam] incoming frame format: fourcc='%s' (0x%08x), "
                "%dx%d stride=%d (=%d bytes/pixel)\n",
                c, (unsigned)fmt, w, h, stride,
                h > 0 ? stride / std::max(1, w) : 0);
        }
    }

    // Decode the packed-YUYV ('yuvs') plane to BGR.
    //
    // The OV580 (PS4 Camera) does NOT deliver a plain scanline image.
    // Per the ps4eye / PS4EYECam reverse-engineering (ps4eye.cpp's
    // de-interleave, memcpy at +32+64), every row is laid out as:
    //
    //   [32 B header][64 B header][LEFT eye ew*2 B][RIGHT eye ew*2 B][junk]
    //
    // and only `ih` of the reported rows carry image (the rest are
    // metadata). Treating the whole reported WxH as a single YUYV
    // image is what produced the diagonal-streak shear - the 96-byte
    // per-row header offsets each row progressively. We extract just
    // the LEFT eye into a packed buffer and decode that.
    //
    //   reported width   eye width (ew)   image height (ih)
    //        3448             1280              800
    //        1748              640              400
    //         898              320              200
    //
    // (YUYV, not UYVY, confirmed: COLOR_YUV2BGR_UYVY gave magenta/green.)
    int eye_w = 0, eye_h = 0;
    if (ov580_eye_dims_for_raw(w, eye_w, eye_h)) {
        static bool announced = false;
        if (!announced) {
            std::fprintf(stderr,
                "[psvr-cam] OV580 frame %dx%d: de-interleaving left eye "
                "%dx%d (row=%dB packed, skip 96B/row header)\n",
                w, h, eye_w, eye_h, w * 2);
            announced = true;
        }
        // Width-based auto-HFOV correction. start()'s auto-HFOV keys
        // off the device's localizedName; an OV580 that enumerates
        // under an unexpected name falls back to the generic 70 deg -
        // an ~18% focal error that scales the solved Z (and X/Y) off
        // by the same factor. The 898/1748/3448 raw widths reaching
        // this branch are conclusive OV580 evidence, so correct the
        // auto HFOV here once per session.
        if (!s->ov580_hfov_checked) {
            s->ov580_hfov_checked = true;
            if (s->desired_hfov_auto.load(std::memory_order_relaxed)) {
                const double cur =
                    s->desired_hfov_deg.load(std::memory_order_relaxed);
                if (std::abs(cur - 85.0) > 0.5) {
                    s->desired_hfov_deg.store(85.0,
                                              std::memory_order_relaxed);
                    std::fprintf(stderr,
                        "[psvr-cam] OV580 detected by frame width; "
                        "auto HFOV %.1f -> 85.0 deg\n", cur);
                }
            }
        }
        // CRITICAL: the OV580 image data is packed CONTIGUOUSLY at
        // w*2 bytes per row (3496 for the 1748-wide mode), NOT at the
        // stride CVPixelBufferGetBytesPerRow() reports (3520). The
        // 3520 figure is the buffer's allocation row size; the actual
        // pixel data has no per-row padding (the slack is all trailing).
        // Stepping by the reported 3520 drifts 24 bytes (12 px) per row
        // and shears the whole image diagonally - verified empirically
        // by reshaping a raw dump: a 12-px/row shift vanishes at a
        // 3496-byte row pitch. So we step by `row_bytes = w * 2`.
        //
        // Within each contiguous row: [32B+64B header][LEFT eye ew*2 B]
        // [RIGHT eye ew*2 B][junk]. Extract the left eye's YUYV and
        // decode it.
        const int    row_bytes = w * 2;       // 3496 for 1748-wide mode
        const int    kHdrBytes = 32 + 64;     // per-row header
        cv::Mat left(eye_h, eye_w, CV_8UC2);
        for (int y = 0; y < eye_h; ++y) {
            std::memcpy(left.ptr(y),
                        base + (size_t)y * row_bytes + kHdrBytes,
                        (size_t)eye_w * 2);
        }
        cv::cvtColor(left, s->bgr_owned, cv::COLOR_YUV2BGR_YUYV);
    } else {
        // Ordinary webcam delivering plain packed YUYV. Wrap at the
        // TRUE image width and pass the reported stride as the row
        // step - OpenCV walks padded rows correctly via step. The
        // previous stride/2-wide wrap baked any row padding into the
        // image as garbage right-edge columns AND shifted the pinhole
        // principal point (cx became stride/4, not w/2), silently
        // corrupting the PnP intrinsics on cameras that pad rows.
        cv::Mat yuyv(h, w, CV_8UC2, base, (size_t)stride);
        cv::cvtColor(yuyv, s->bgr_owned, cv::COLOR_YUV2BGR_YUYV);
    }

    CVPixelBufferUnlockBaseAddress(buf, kCVPixelBufferLock_ReadOnly);

    // Optional diagnostic dump of the clean de-interleaved frame. Keep
    // this out of the default capture path; disk I/O from the camera
    // callback adds avoidable jitter when testing tracking latency.
    if (frame_dumps_enabled()) {
        static int dump_skip = 0;
        if ((dump_skip++ % 150) == 0) {
            cv::Mat g;
            cv::cvtColor(s->bgr_owned, g, cv::COLOR_BGR2GRAY);
            if (FILE* f = std::fopen("/tmp/psvr-frame.pgm", "wb")) {
                std::fprintf(f, "P5\n%d %d\n255\n", g.cols, g.rows);
                for (int y = 0; y < g.rows; ++y)
                    std::fwrite(g.ptr(y), 1, g.cols, f);
                std::fclose(f);
            }
        }
    }

    // Grayscale brightness gate. Was an HSV-blue + white-saturated
    // double mask; replaced with a single grayscale threshold because
    // (a) the PSVR LEDs are eye-searingly bright (>=240 on a
    // calibrated capture, >=200 even after webcam AWB and exposure
    // compression) so brightness alone separates them from the room,
    // (b) the HSV blue-hue gate was camera-dependent - UGREEN,
    // FaceTime, and the PS Camera each render PSVR blue at a
    // different OpenCV-H angle, and we were chasing per-camera tuning
    // tables for what is fundamentally a brightness signal. The
    // tracker-pt webcam extractor uses the same approach (cv::COLOR_
    // BGR2GRAY -> cv::threshold), see tracker-pt/module/point_
    // extractor.cpp::threshold_image fixed-threshold branch.
    cv::cvtColor(s->bgr_owned, s->gray, cv::COLOR_BGR2GRAY);
    const int bright_thresh = adaptive_bright_threshold(s->gray);
    cv::threshold(s->gray, s->mask, bright_thresh, 255, cv::THRESH_BINARY);

    // Blue chroma-key rescue mask, OR-ed in (see CHROMA_THRESH_*).
    // chroma = B - 0.5*(G+R), adaptively thresholded, unions with luma.
    int chroma_thresh = 0;
    int chroma_px = 0;
    {
        cv::Mat ch[3];
        cv::split(s->bgr_owned, ch);            // BGR order: ch[0] = B
        cv::Mat gr, chroma, blue_mask;
        cv::addWeighted(ch[1], 0.5, ch[2], 0.5, 0.0, gr);  // 0.5*(G+R)
        cv::subtract(ch[0], gr, chroma);        // CV_8U saturates at 0
        chroma_thresh = adaptive_topN_threshold(
            chroma, N_TARGET_BRIGHT_PIXELS,
            CHROMA_THRESH_FLOOR, CHROMA_THRESH_CEIL);
        cv::threshold(chroma, blue_mask, chroma_thresh, 255,
                      cv::THRESH_BINARY);
        chroma_px = cv::countNonZero(blue_mask);
        cv::bitwise_or(s->mask, blue_mask, s->mask);
    }

    // Field-tuned: switched from MORPH_OPEN(2x2) to MORPH_DILATE(3x3).
    // Open (erode-then-dilate) was eating the 1-2 pixel LED cores -
    // a 2x2 erode of a 2x2 blob leaves nothing for dilate to grow back.
    // Plain dilate (no erode) GROWS each LED's saturated core by 1 px
    // on every side, taking a 2x2 core to 4x4, comfortably above the
    // area gate. Isolated noise hot-pixels also grow to 3x3 but the
    // tighter N_TARGET_BRIGHT_PIXELS (100) above means very few of
    // them appear in the first place, so the signal/noise stays good.
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
    cv::dilate(s->mask, s->mask, kernel);

    // Diagnostic: report the actual brightness range the camera is
    // delivering, ONCE per second (~30 frames). Reveals AVFoundation
    // exposure issues - if max never gets above ~220 even with the
    // LEDs in view, the camera is suppressing them and a software-
    // side threshold tweak can't fix it.
    {
        static int diag_skip = 0;
        if (++diag_skip >= 30) {
            diag_skip = 0;
            double mn = 0, mx = 0;
            cv::minMaxLoc(s->gray, &mn, &mx);
            std::fprintf(stderr,
                "[psvr-cam] frame brightness: min=%.0f max=%.0f thresh=%d "
                "| chroma thresh=%d px=%d\n",
                mn, mx, bright_thresh, chroma_thresh, chroma_px);
        }
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(s->mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Diagnostic: raw contour statistics, once per ~30 frames. Tells
    // us at a glance whether the extractor is producing many small
    // contours that the area/circularity gate is filtering down, or
    // one giant contour (room window, monitor reflection, ceiling
    // lamp) that BLOB_MAX_AREA_PX rejects and leaves us with
    // nothing.
    //
    // Symptom-side correspondence: the periodic [psvr-cam] line
    // showing blobs=1-3 with thresh=237 is consistent with both
    // (a) "thousands of bright pixels merged into one giant blob
    // larger than BLOB_MAX_AREA_PX" and (b) "thousands of pixels in
    // many tiny clusters all under BLOB_MIN_AREA_PX". This log
    // distinguishes them: case (a) shows raw_n small and max_area
    // huge with a large bbox; case (b) shows raw_n big and max_area
    // tiny. The fix is different - case (a) needs a watershed split
    // or BLOB_MAX_AREA_PX raise, case (b) needs morph_open tuning
    // or a lower BLOB_MIN_AREA_PX.
    {
        static int diag_skip = 0;
        if (++diag_skip >= 30) {
            diag_skip = 0;
            int    raw_n   = (int)contours.size();
            double min_a   = raw_n ? std::numeric_limits<double>::infinity() : 0.0;
            double max_a   = 0.0;
            cv::Rect big_bbox{0, 0, 0, 0};
            for (const auto& c : contours) {
                const double a = cv::contourArea(c);
                if (a < min_a) min_a = a;
                if (a > max_a) { max_a = a; big_bbox = cv::boundingRect(c); }
            }
            // Print 0 for min_a when there are no contours, not +inf.
            if (raw_n == 0) min_a = 0.0;
            std::fprintf(stderr,
                "[psvr-cam] contour stats: raw=%d min_area=%.0f max_area=%.0f "
                "largest_bbox=(%d,%d,%dx%d)\n",
                raw_n, min_a, max_a,
                big_bbox.x, big_bbox.y, big_bbox.width, big_bbox.height);
        }
    }

    // Per-contour area + circularity gate, then mean-shift sub-pixel
    // refinement of the centroid using the underlying grayscale image
    // as a peak-finder. The contour centroid alone is biased by which
    // direction the binary-mask boundary rasterised; mean-shift on
    // the gray image moves in sub-pixel steps toward the true LED
    // peak. Cost: ~50 us per blob at typical ROI sizes; with <=20
    // blobs that's <=1 ms/frame on top of the ~3 ms threshold/contour
    // path. Net per-frame cost is comparable to or slightly cheaper
    // than the old two-mask HSV path, since BGR2GRAY+threshold is
    // strictly faster than BGR2HSV+inRange+inRange+bitwise_or.
    // Collect candidate blobs with an integrated-brightness score, then
    // keep only the BLOB_POOL_MAX brightest (tracker-pt does the same:
    // point_extractor.cpp sorts by brightness and caps max_blobs=16).
    // Rationale for the PSVR case: the real LEDs are the brightest
    // things in frame, so ranking by integrated intensity pushes sensor
    // hot-pixels and dim reflections to the tail, and the cap bounds the
    // constellation solver's permutation-RANSAC combinatorics regardless
    // of how many spurious blobs a busy scene produces. The cap only
    // drops blobs beyond the 16 brightest, so the typical 5-10-blob
    // PSVR frame is unaffected.
    struct ScoredBlob { cv::Point2d pt; double brightness; };
    std::vector<ScoredBlob> scored;
    scored.reserve(contours.size());
    for (const auto& c : contours) {
        const double area = cv::contourArea(c);
        if (area < BLOB_MIN_AREA_PX || area > BLOB_MAX_AREA_PX) continue;
        const double perim = cv::arcLength(c, /*closed=*/true);
        if (perim <= 0) continue;
        const double circularity = 4.0 * CV_PI * area / (perim * perim);
        if (circularity < BLOB_MIN_CIRCULARITY) continue;
        const cv::Moments mom = cv::moments(c);
        if (mom.m00 <= 0) continue;
        const double cx_global = mom.m10 / mom.m00;
        const double cy_global = mom.m01 / mom.m00;

        // Mean-shift refinement uses an ROI around the contour
        // bounding box, padded slightly so the kernel can "see" the
        // LED's brightness falloff outside the thresholded core.
        // We clip to image bounds (negative origin / past-edge ROIs
        // would crash cv::Mat); if the contour sits on the image
        // edge we fall back to the contour-centroid estimate.
        const cv::Rect bbox = cv::boundingRect(c);
        const int pad = std::max(2, (int)std::ceil(std::sqrt(area) * 0.5));
        cv::Rect roi(bbox.x - pad, bbox.y - pad,
                     bbox.width + 2 * pad, bbox.height + 2 * pad);
        roi &= cv::Rect(0, 0, s->gray.cols, s->gray.rows);
        if (roi.width < 3 || roi.height < 3) {
            scored.push_back({cv::Point2d(cx_global, cy_global), area});
            continue;
        }
        const cv::Mat1b roi_gray = s->gray(roi);
        // Integrated brightness over the ROI. The thresholded scene is
        // dark everywhere but the LEDs, so the ROI sum is dominated by
        // this blob's energy - a good, cheap ranking key. Used only for
        // sort/cap ordering, never for geometry.
        const double brightness = cv::sum(roi_gray)[0];
        // Kernel radius from the LED's footprint (radius = sqrt(A/pi))
        // scaled by tracker-pt's empirical MEAN_SHIFT_RADIUS_C.
        const double radius_px    = std::sqrt(area / CV_PI);
        const double filter_width = radius_px * MEAN_SHIFT_RADIUS_C;
        // Seed mean-shift with the contour centroid in ROI-local
        // coordinates. The iteration runs until the centre stops
        // moving (delta^2 < 1e-3 px) or MEAN_SHIFT_MAX_ITERS rounds.
        cv::Point2d pos(cx_global - roi.x, cy_global - roi.y);
        for (int iter = 0; iter < MEAN_SHIFT_MAX_ITERS; ++iter) {
            const cv::Point2d com_new = MeanShiftIteration(
                roi_gray, pos, filter_width);
            const double ddx = com_new.x - pos.x;
            const double ddy = com_new.y - pos.y;
            pos = com_new;
            if (ddx * ddx + ddy * ddy < 1e-3) break;
        }
        scored.push_back({cv::Point2d(pos.x + roi.x, pos.y + roi.y),
                          brightness});
    }

    // Keep the brightest BLOB_POOL_MAX; emit their centroids in
    // brightness order (brightest first).
    if ((int)scored.size() > BLOB_POOL_MAX) {
        std::nth_element(scored.begin(), scored.begin() + BLOB_POOL_MAX,
                         scored.end(),
                         [](const ScoredBlob& a, const ScoredBlob& b) {
                             return a.brightness > b.brightness;
                         });
        scored.resize(BLOB_POOL_MAX);
    }
    std::sort(scored.begin(), scored.end(),
              [](const ScoredBlob& a, const ScoredBlob& b) {
                  return a.brightness > b.brightness;
              });
    std::vector<cv::Point2d> blobs;
    blobs.reserve(scored.size());
    for (const auto& sb : scored) blobs.push_back(sb.pt);

    // The overlay-drawing block below mutates a BGR copy of the
    // capture in-place. Use the owned bgr_owned we already built (the
    // IOSurface is long since unlocked); clone so the next frame's
    // process_frame can reuse bgr_owned without smearing overlays
    // from this frame across it.
    cv::Mat vis = s->bgr_owned.clone();

    // Hand off to constellation stage. It may return no_solution; in
    // that case we still record the blob count so the user can tell
    // whether the camera even saw LEDs vs. a PnP / ID failure.
    const double yaw      = s->yaw_rad.load(std::memory_order_relaxed);
    const double pitch    = s->pitch_rad.load(std::memory_order_relaxed);
    const double roll     = s->roll_rad.load(std::memory_order_relaxed);
    const double hfov_deg = s->desired_hfov_deg.load(std::memory_order_relaxed);

    // Pass the DE-INTERLEAVED working-image dimensions, not the raw pixel
    // buffer's. The blob centroids are in s->gray's coordinate space, and
    // for the OV580 that gray is the 640x400 left eye carved out of the
    // 1748x408 stereo buffer (w/h above). Feeding the solver 1748x408 put
    // the intrinsics' principal point at (874,204) while blobs sit near
    // x~300, so K was grossly wrong - free-rotation PnP masked it by
    // absorbing the error into a skewed pose (a big contributor to the
    // bogus X offset), but the IMU-rotation-locked solve cannot and just
    // diverged. For non-OV580 cameras s->gray matches the buffer size, so
    // this is correct for every camera.
    const int solve_w = s->gray.cols;
    const int solve_h = s->gray.rows;
    psvr_constellation::Result r =
        s->solver_state.solve(blobs, solve_w, solve_h, yaw, pitch, roll,
                              hfov_deg);

    // Seqlock publish: bracket the multi-double mutation with two
    // increments of pos_seq so any concurrent reader either sees the
    // pre-publish or post-publish state, never a half-written tuple.
    // pos_seq becomes ODD between the increments ("write in
    // progress") and EVEN again afterwards ("stable"). The
    // acq_rel ordering on the bracket pins the data writes between
    // them as far as other threads are concerned.
    if (r.ok) {
        s->pos_seq.fetch_add(1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        s->pos_x_cm.store(r.x_cm, std::memory_order_relaxed);
        s->pos_y_cm.store(r.y_cm, std::memory_order_relaxed);
        s->pos_z_cm.store(r.z_cm, std::memory_order_relaxed);
        s->pnp_ok_latest.store(true, std::memory_order_relaxed);
        s->result_epoch.store(now_sec(), std::memory_order_relaxed);
        s->pos_seq.fetch_add(1, std::memory_order_release);
    } else {
        // Failure path: only the pnp_ok flag changes. Same seqlock
        // bracket so a concurrent reader either sees the previous
        // good (ok=true) state or the new (ok=false) state, never
        // ok=false with stale-but-still-fresh-epoch x/y/z.
        s->pos_seq.fetch_add(1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        s->pnp_ok_latest.store(false, std::memory_order_relaxed);
        // Intentionally do NOT bump result_epoch or zero x/y/z:
        // last-good values stay around for any caller that wants a
        // "last known" snapshot for logging.
        s->pos_seq.fetch_add(1, std::memory_order_release);
    }

    // Diag bookkeeping (protected by a lightweight mutex only during
    // bulk copy; fine even at 60 Hz). Snapshot the cumulative counters
    // here so the preview overlay below + the periodic stderr summary
    // both read the same just-updated values. Moved above the preview
    // block so the overlay can show "pnp_ok: N / M" correctly without
    // a one-frame lag.
    uint64_t frames_after = 0, blob_after = 0, pnp_after = 0;
    double last_x_after = 0, last_y_after = 0, last_z_after = 0;
    {
        std::lock_guard<std::mutex> lk(s->diag_mu);
        s->diag.frames_captured++;
        if (!blobs.empty()) s->diag.frames_with_any_blob++;
        if (r.ok)           s->diag.pnp_ok_count++;
        s->diag.last_n_blobs        = (int)blobs.size();
        s->diag.last_n_visible      = r.n_visible;
        s->diag.last_n_matched      = r.n_matched;
        s->diag.last_bright_thresh  = bright_thresh;
        s->diag.last_pnp_ok         = r.ok;
        s->diag.last_reject_reason  = r.reject_reason;
        if (r.ok) {
            s->diag.last_x_cm = r.x_cm;
            s->diag.last_y_cm = r.y_cm;
            s->diag.last_z_cm = r.z_cm;
        }
        frames_after = s->diag.frames_captured;
        blob_after   = s->diag.frames_with_any_blob;
        pnp_after    = s->diag.pnp_ok_count;
        last_x_after = s->diag.last_x_cm;
        last_y_after = s->diag.last_y_cm;
        last_z_after = s->diag.last_z_cm;
    }

    // Build the annotated preview frame. `vis` was already populated
    // (BGR copy of the capture) above, before we unlocked the IOSurface
    // - we mutate it here with overlays for the Qt preview dialog:
    // blobs as red filled dots, projected LEDs as hollow green circles,
    // matched pairs as yellow lines, plus a multi-line diagnostic
    // stack at top-left. This doubles as a visual debugger for
    // coordinate-frame conventions: if projected LEDs land in the
    // wrong corner of the frame relative to the actual blobs, the bug
    // is obvious.
    {
        // Detected blobs: small red filled circles.
        for (const auto& b : blobs) {
            cv::circle(vis, cv::Point((int)b.x, (int)b.y), 4,
                       cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
        }

        // Projected LED positions under the prior pose: hollow green
        // circles, with the LED index drawn next to each one so we can
        // tell which LED the solver thinks should be where.
        for (int i = 0; i < psvr_constellation::NUM_LEDS; ++i) {
            if (!r.visible[i]) continue;
            const cv::Point pt((int)r.projected[i].x, (int)r.projected[i].y);
            cv::circle(vis, pt, 10, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            char lab[8];
            std::snprintf(lab, sizeof lab, "%d", i);
            cv::putText(vis, lab, pt + cv::Point(12, 4),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }

        // Matched LED-blob pairs: yellow line connecting them. Easy to
        // spot when matching has gone wrong (lines fan out instead of
        // staying short).
        for (int i = 0; i < psvr_constellation::NUM_LEDS; ++i) {
            if (!r.visible[i]) continue;
            const int bj = r.matched_blob_idx[i];
            if (bj < 0 || bj >= (int)blobs.size()) continue;
            cv::line(vis,
                     cv::Point((int)r.projected[i].x, (int)r.projected[i].y),
                     cv::Point((int)blobs[bj].x,      (int)blobs[bj].y),
                     cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        }

        // Multi-line diagnostic stack at top-left. Shows the same
        // values the periodic [psvr-cam] stderr line carries, so a
        // user diagnosing the helmet pose from inside opentrack's
        // UI (i.e. without a terminal in view) sees the same signal
        // the developer sees. Compact 0.5 scale + 22 px line height
        // fits ~24 chars per line; the preview QLabel halves the
        // 1280x720 capture to ~640x360, so apparent text height is
        // ~11 px - readable but unobtrusive.
        constexpr double kRad2Deg = 180.0 / M_PI;
        const double y_deg = s->yaw_rad.load(std::memory_order_relaxed)   * kRad2Deg;
        const double p_deg = s->pitch_rad.load(std::memory_order_relaxed) * kRad2Deg;
        const double rr_deg= s->roll_rad.load(std::memory_order_relaxed)  * kRad2Deg;
        char lines[4][96];
        std::snprintf(lines[0], sizeof lines[0],
                      "blobs: %d   vis: %d   matched: %d",
                      (int)blobs.size(), r.n_visible, r.n_matched);
        std::snprintf(lines[1], sizeof lines[1],
                      "thresh: %d   reject: %s",
                      bright_thresh, r.reject_reason);
        std::snprintf(lines[2], sizeof lines[2],
                      "ypr: %+.0f %+.0f %+.0f   pos: %+.1f %+.1f %+.1f",
                      y_deg, p_deg, rr_deg,
                      r.ok ? r.x_cm : s->diag.last_x_cm,
                      r.ok ? r.y_cm : s->diag.last_y_cm,
                      r.ok ? r.z_cm : s->diag.last_z_cm);
        std::snprintf(lines[3], sizeof lines[3],
                      "pnp_ok: %llu / %llu  rms: %.1f",
                      (unsigned long long)pnp_after,
                      (unsigned long long)frames_after,
                      r.reprojection_rms);
        const int line_h = 22;
        int y = 24;
        for (const auto& ln : lines) {
            // Black drop-shadow then white text - readable on any bg
            // including a fully bright frame.
            cv::putText(vis, ln, cv::Point(12, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
            cv::putText(vis, ln, cv::Point(12, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            y += line_h;
        }

        // Multi-line status banner from the plugin (calibration state,
        // failure messages, etc.). Drawn larger and centered so it's
        // unmistakable. Each \n becomes a new line; we wrap implicitly
        // by also breaking text that exceeds 90% of the frame width on
        // word boundaries to handle phone-style narrow windows.
        std::string banner;
        cv::Scalar  banner_bgr{255, 255, 255};
        {
            std::lock_guard<std::mutex> blk(s->banner_mu);
            banner       = s->banner_text;
            banner_bgr   = s->banner_color_bgr;
        }
        if (!banner.empty()) {
            // 2x the previous size - user requested the warm-up text
            // to be more prominent. At 2.6 / thickness 6 the text
            // renders ~64 px tall in the source frame, ~32 px after
            // setScaledContents halves it down. Drop-shadow scales
            // with the font so edges stay crisp at the larger size.
            const double font_scale  = 2.6;
            const int    font_thick  = 6;
            const int    line_height = 96;
            const int    max_width   = (int)(vis.cols * 0.9);

            // Split on '\n' first.
            std::vector<std::string> lines;
            {
                std::string cur;
                for (char c : banner) {
                    if (c == '\n') { lines.push_back(cur); cur.clear(); }
                    else           { cur.push_back(c); }
                }
                lines.push_back(cur);
            }

            // Then word-wrap each line so it fits within max_width.
            std::vector<std::string> wrapped;
            for (const auto& src : lines) {
                std::string cur;
                std::string word;
                auto flush_word = [&](bool force_break) {
                    if (word.empty()) return;
                    std::string trial = cur.empty() ? word : (cur + " " + word);
                    int baseline = 0;
                    cv::Size sz = cv::getTextSize(trial,
                        cv::FONT_HERSHEY_SIMPLEX, font_scale, font_thick,
                        &baseline);
                    if (sz.width > max_width && !cur.empty()) {
                        wrapped.push_back(cur);
                        cur = word;
                    } else {
                        cur = trial;
                    }
                    word.clear();
                    if (force_break) {
                        wrapped.push_back(cur);
                        cur.clear();
                    }
                };
                for (char c : src) {
                    if (c == ' ') flush_word(false);
                    else          word.push_back(c);
                }
                flush_word(true);
            }

            // Vertically center the block.
            const int total_h = (int)wrapped.size() * line_height;
            int y = std::max(40, (vis.rows - total_h) / 2 + line_height);
            for (const auto& ln : wrapped) {
                int baseline = 0;
                cv::Size sz = cv::getTextSize(ln,
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, font_thick,
                    &baseline);
                const int x = std::max(10, (vis.cols - sz.width) / 2);
                // Black drop-shadow underneath for legibility on busy
                // backgrounds; colored text on top.
                cv::putText(vis, ln, cv::Point(x, y),
                            cv::FONT_HERSHEY_SIMPLEX, font_scale,
                            cv::Scalar(0, 0, 0), font_thick + 2, cv::LINE_AA);
                cv::putText(vis, ln, cv::Point(x, y),
                            cv::FONT_HERSHEY_SIMPLEX, font_scale,
                            banner_bgr, font_thick, cv::LINE_AA);
                y += line_height;
            }
        }

        if (frame_dumps_enabled()) {
            static int preview_dump_skip = 0;
            if ((preview_dump_skip++ % 30) == 0) {
                if (FILE* f = std::fopen("/tmp/psvr-preview.ppm", "wb")) {
                    std::fprintf(f, "P6\n%d %d\n255\n", vis.cols, vis.rows);
                    for (int y = 0; y < vis.rows; ++y) {
                        const cv::Vec3b* row = vis.ptr<cv::Vec3b>(y);
                        for (int x = 0; x < vis.cols; ++x) {
                            const unsigned char rgb[3] = {
                                row[x][2], row[x][1], row[x][0]
                            };
                            std::fwrite(rgb, 1, sizeof rgb, f);
                        }
                    }
                    std::fclose(f);
                }
            }
        }

        std::lock_guard<std::mutex> lk(s->preview_mu);
        s->preview_bgr = std::move(vis);
    }

    // Periodic [psvr-cam] stderr summary. One line per
    // PSVR_CAM_LOG_INTERVAL_FRAMES frames (~1 s at 30 fps), unconditional
    // on the diag-log toggle. Lets a user verify "did the camera see
    // any LEDs?" / "did PnP succeed?" from the same console they're
    // already watching for USB HID activity.
    if (frames_after > 0 &&
        (frames_after % (uint64_t)PSVR_CAM_LOG_INTERVAL_FRAMES) == 0) {
        // Reload the IMU rotation prior so the log line carries the
        // orientation the matcher actually consumed for this frame.
        // Converted to degrees for readability. yaw/pitch/roll are the
        // values that drove the visibility filter; if vis=1-3 while the
        // helmet is visibly facing the camera, those values are the
        // first place to look.
        constexpr double kRad2Deg = 180.0 / M_PI;
        const double y_deg = s->yaw_rad.load(std::memory_order_relaxed)   * kRad2Deg;
        const double p_deg = s->pitch_rad.load(std::memory_order_relaxed) * kRad2Deg;
        const double r_deg = s->roll_rad.load(std::memory_order_relaxed)  * kRad2Deg;
        std::fprintf(stderr,
            "[psvr-cam] frames=%llu  any_blob=%llu  pnp_ok=%llu  "
            "last: thresh=%d blobs=%d vis=%d matched=%d pnp=%s reject=%s "
            "ypr=[%+.1f %+.1f %+.1f] pos=[%+.1f %+.1f %+.1f]\n",
            (unsigned long long)frames_after,
            (unsigned long long)blob_after,
            (unsigned long long)pnp_after,
            bright_thresh,
            (int)blobs.size(), r.n_visible, r.n_matched,
            r.ok ? "OK" : "--",
            r.reject_reason,
            y_deg, p_deg, r_deg,
            last_x_after, last_y_after, last_z_after);
    }
}

} // namespace psvr_cam

@implementation PSVRCaptureDelegate

- (void)captureOutput:(AVCaptureOutput*)output
  didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
          fromConnection:(AVCaptureConnection*)connection {
    (void)output; (void)connection;
    if (!self.owner || !self.owner->running.load()) return;
    CVPixelBufferRef buf = CMSampleBufferGetImageBuffer(sampleBuffer);
    if (!buf) return;
    psvr_cam::process_frame(self.owner, buf);
}

@end
