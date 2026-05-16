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

#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>
#import <CoreMedia/CoreMedia.h>
#import <CoreVideo/CoreVideo.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <mutex>
#include <chrono>
#include <cstdio>

namespace psvr_cam {

// Freshness window for publish/consume. Camera runs at ~30 Hz; we allow
// up to ~15 frames (~500 ms) of staleness before the reader treats the
// position as "unknown" and falls back to its default (usually zero).
// Tuned to match the "degrades gracefully to IMU-only within ~500 ms"
// behavior the plugin's docs promise: short enough that a real loss
// of tracking (helmet leaves the frame) is reflected promptly, long
// enough that a few consecutive failed PnP frames (one blink-like
// LED dropout, a moving lamp briefly confusing the matcher) don't
// visibly snap the user's position to zero. The previous 200 ms
// window was tight enough that any minor hiccup cleared position;
// users perceived that as "position tracking doesn't do anything".
static constexpr double RESULT_STALE_SEC = 0.5;

// Periodic [psvr-cam] stderr summary cadence (frames). At ~30 Hz this
// emits one line per second - low enough to read at a glance without
// flooding the console. Independent of the diag-log toggle so users
// can verify the camera is actually seeing blobs / solving PnP from
// the same console they use to watch USB HID startup, without first
// enabling the verbose constellation log file.
static constexpr int PSVR_CAM_LOG_INTERVAL_FRAMES = 30;

// Minimum / maximum blob area in pixels for the extractor. The PSVR
// LEDs at ~60-100 cm from a 720p camera at ~70deg HFOV render as
// roughly 3-25 px diameter (5-500 px^2). The lower bound rejects
// single-pixel speckle and the morphological-opening leftovers that
// would otherwise inflate the candidate set into the 20+ range; the
// upper bound rejects bright spread-out areas like a ceiling lamp or
// a window in frame. 5 px^2 is small enough to keep a 2-3 px LED at
// ~150 cm range while still dropping the salt-and-pepper noise that
// every webcam produces after grayscale thresholding.
// DIAGNOSTIC v2: dropped all the way to 1.0 to admit even single-pixel
// LED dots. With area=1 plus circularity disabled below, virtually any
// bright pixel cluster passes. Matcher's RANSAC + facing-camera filter
// will discriminate real LEDs from noise. Tighten back toward 3-5
// once we know blobs are reaching the matcher.
static constexpr double BLOB_MIN_AREA_PX = 1.0;
static constexpr double BLOB_MAX_AREA_PX = 500.0;

// Minimum 4*PI*A/P^2 circularity. A geometric circle is 1.0; a real
// PSVR LED (with a tiny bit of motion blur / partial saturation tail)
// ranges 0.7-0.95. Highlights from room edges (monitor bezels, table
// edges, glasses frames, the helmet's own metal trim) are elongated
// and score 0.2-0.4. 0.55 was the original tight value; DIAGNOSTIC v2
// disables the gate (0.0) so reflections off helmet plastic, partial
// rim LEDs at off-axis poses, and pixelated dots all pass. The
// matcher's RANSAC + facing-camera filter discriminate real LEDs.
static constexpr double BLOB_MIN_CIRCULARITY = 0.0;

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
static constexpr int N_TARGET_BRIGHT_PIXELS = 1500;

// Pick a brightness threshold by walking the histogram from V=255
// down until cumulative pixel count reaches N_TARGET_BRIGHT_PIXELS.
// Result is clamped to [FLOOR, CEIL]. O(W*H) for the histogram +
// O(256) for the walk; ~0.5 ms total at 1280x720. No state -
// next-frame's threshold is independent of this frame's, so a
// momentary occlusion can't pin a bad value across frames.
static int adaptive_bright_threshold(const cv::Mat& gray) {
    int hist_size = 256;
    float range[] = {0.f, 256.f};
    const float* hist_range = range;
    cv::Mat hist;
    cv::calcHist(&gray, 1, nullptr, cv::Mat(), hist, 1,
                 &hist_size, &hist_range);
    int cum = 0;
    for (int v = 255; v >= 0; --v) {
        cum += static_cast<int>(hist.at<float>(v));
        if (cum >= N_TARGET_BRIGHT_PIXELS)
            return std::max(BRIGHT_THRESH_FLOOR,
                            std::min(BRIGHT_THRESH_CEIL, v));
    }
    return BRIGHT_THRESH_FLOOR;
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

    std::atomic<bool> running{false};

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
    std::atomic<uint64_t> pos_seq{0};
    double                pos_x_cm{0}, pos_y_cm{0}, pos_z_cm{0};
    double                result_epoch{0};      // monotonic wall time
    bool                  pnp_ok_latest{false};

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
        const double  epoch  = impl_->result_epoch;
        const double  x_v    = impl_->pos_x_cm;
        const double  y_v    = impl_->pos_y_cm;
        const double  z_v    = impl_->pos_z_cm;
        const bool    ok     = impl_->pnp_ok_latest;
        const uint64_t s2 = impl_->pos_seq.load(std::memory_order_acquire);
        if (s1 != s2) continue;         // writer ran during our read
        if (epoch == 0.0) return false;
        if (now_sec() - epoch > RESULT_STALE_SEC) return false;
        *x = x_v;
        *y = y_v;
        *z = z_v;
        return ok;
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

    @autoreleasepool {
        AVCaptureDevice* dev = pick_camera(impl_->desired_camera_name);
        if (!dev) {
            std::fprintf(stderr, "[psvr-cam] no camera device available\n");
            return false;
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
        session.sessionPreset = AVCaptureSessionPreset1280x720;
        if (![session canAddInput:input]) {
            std::fprintf(stderr, "[psvr-cam] session refused camera input\n");
            return false;
        }
        [session addInput:input];

        AVCaptureVideoDataOutput* output = [[AVCaptureVideoDataOutput alloc] init];
        output.alwaysDiscardsLateVideoFrames = YES;
        // Ask for packed BGRA so OpenCV can wrap the IOSurface directly
        // without a color-space conversion pass.
        output.videoSettings = @{
            (id)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA)
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

        [session startRunning];
        impl_->running.store(true);
        std::fprintf(stderr, "[psvr-cam] camera started: %s (%s)\n",
            dev.localizedName.UTF8String,
            dev.uniqueID.UTF8String);
        return true;
    }
}

void Worker::stop() {
    if (!impl_->running.load()) return;
    @autoreleasepool {
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

    // Zero-copy wrap of the BGRA plane. cv::Mat doesn't take ownership
    // so the IOSurface stays valid as long as we hold the pixel-buffer
    // lock. We do NOT modify bgra_view; we derive owned copies from it.
    cv::Mat bgra(h, w, CV_8UC4, base, stride);

    // Convert to an OpenCV-owned BGR buffer immediately. After this
    // line we no longer depend on the IOSurface and can unlock it.
    // We also need a BGR copy below for the BGR->grayscale conversion
    // (extractor input) and for overlay drawing, so doing it once
    // up-front is strictly cheaper than the previous arrangement
    // (split-then-copy-later).
    cv::cvtColor(bgra, s->bgr_owned, cv::COLOR_BGRA2BGR);

    CVPixelBufferUnlockBaseAddress(buf, kCVPixelBufferLock_ReadOnly);

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

    // DIAGNOSTIC: morph_open dropped to 2x2 from 3x3 and area gate
    // dropped (see BLOB_MIN_AREA_PX) to admit small LED projections.
    // Reverts to {3,3} once we know the LEDs are reaching the
    // extractor as bright pixels.
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {2, 2});
    cv::morphologyEx(s->mask, s->mask, cv::MORPH_OPEN, kernel);

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
                "[psvr-cam] frame brightness: min=%.0f max=%.0f thresh=%d\n",
                mn, mx, bright_thresh);
        }
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(s->mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

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
    std::vector<cv::Point2d> blobs;
    blobs.reserve(contours.size());
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
            blobs.emplace_back(cx_global, cy_global);
            continue;
        }
        const cv::Mat1b roi_gray = s->gray(roi);
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
        blobs.emplace_back(pos.x + roi.x, pos.y + roi.y);
    }

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

    psvr_constellation::Result r =
        s->solver_state.solve(blobs, w, h, yaw, pitch, roll, hfov_deg);

    // Seqlock publish: bracket the multi-double mutation with two
    // increments of pos_seq so any concurrent reader either sees the
    // pre-publish or post-publish state, never a half-written tuple.
    // pos_seq becomes ODD between the increments ("write in
    // progress") and EVEN again afterwards ("stable"). The
    // acq_rel ordering on the bracket pins the data writes between
    // them as far as other threads are concerned.
    if (r.ok) {
        s->pos_seq.fetch_add(1, std::memory_order_acq_rel);
        s->pos_x_cm        = r.x_cm;
        s->pos_y_cm        = r.y_cm;
        s->pos_z_cm        = r.z_cm;
        s->pnp_ok_latest   = true;
        s->result_epoch    = now_sec();
        s->pos_seq.fetch_add(1, std::memory_order_acq_rel);
    } else {
        // Failure path: only the pnp_ok flag changes. Same seqlock
        // bracket so a concurrent reader either sees the previous
        // good (ok=true) state or the new (ok=false) state, never
        // ok=false with stale-but-still-fresh-epoch x/y/z.
        s->pos_seq.fetch_add(1, std::memory_order_acq_rel);
        s->pnp_ok_latest = false;
        // Intentionally do NOT bump result_epoch or zero x/y/z:
        // last-good values stay around for any caller that wants a
        // "last known" snapshot for logging.
        s->pos_seq.fetch_add(1, std::memory_order_acq_rel);
    }

    // Build the annotated preview frame. `vis` was already populated
    // (BGR copy of the capture) above, before we unlocked the IOSurface
    // - we mutate it here with overlays for the Qt preview dialog:
    // blobs as red filled dots, projected LEDs as hollow green circles,
    // matched pairs as yellow lines, plus a status banner. This doubles
    // as a visual debugger for coordinate-frame conventions: if projected
    // LEDs land in the wrong corner of the frame relative to the actual
    // blobs, the bug is obvious.
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

        // Solver-state banner at the top-left: blob/match/PnP numbers
        // live. Sized so it stays readable after the preview QLabel
        // scales the 1280x720 capture down to ~640x360 in the
        // tracker-frame (half-size), which halves apparent font
        // height. Drawn black-then-white for legibility on any bg.
        char solver_status[256];
        std::snprintf(solver_status, sizeof solver_status,
                      "blobs=%d  matched=%d  pnp=%s  rms=%.1f  pos=[%+.1f %+.1f %+.1f]",
                      (int)blobs.size(), r.n_matched,
                      r.ok ? "OK" : "--",
                      r.reprojection_rms,
                      r.ok ? r.x_cm : 0.0,
                      r.ok ? r.y_cm : 0.0,
                      r.ok ? r.z_cm : 0.0);
        cv::putText(vis, solver_status, cv::Point(16, 36),
                    cv::FONT_HERSHEY_SIMPLEX, 0.9,
                    cv::Scalar(0, 0, 0), 5, cv::LINE_AA);
        cv::putText(vis, solver_status, cv::Point(16, 36),
                    cv::FONT_HERSHEY_SIMPLEX, 0.9,
                    cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

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

        std::lock_guard<std::mutex> lk(s->preview_mu);
        s->preview_bgr = std::move(vis);
    }

    // Diag bookkeeping (protected by a lightweight mutex only during
    // bulk copy; fine even at 60 Hz). Snapshot the cumulative counters
    // here so the periodic stderr summary below can read them without
    // re-acquiring the mutex.
    uint64_t frames_after = 0, blob_after = 0, pnp_after = 0;
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
            r.ok ? r.x_cm : 0.0,
            r.ok ? r.y_cm : 0.0,
            r.ok ? r.z_cm : 0.0);
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
