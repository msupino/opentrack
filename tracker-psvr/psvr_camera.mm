/* PSVR camera-based LED constellation worker (implementation).
 *
 * Pipeline per frame (driven by AVFoundation's capture delegate on a
 * private dispatch queue):
 *
 *   1. Receive CMSampleBuffer, wrap its pixel buffer as a cv::Mat
 *      without copying (IOSurface-backed, planar BGRA).
 *   2. Extract blue-dominant bright blobs: threshold on (B - max(R,G))
 *      + min absolute B brightness. Gives a clean mask even with face
 *      skin and monitor glare around the headset.
 *   3. Find contours, compute centroid + area per contour. Drop tiny
 *      and gigantic ones (noise / lamp reflections).
 *   4. Pass the list of blob centroids to the constellation module
 *      along with the latest IMU rotation prior; it does LED-ID and
 *      solvePnP and returns a 6-DOF position if successful.
 *   5. Publish the result via atomic store of three doubles + a
 *      monotonic result_epoch. Consumers read_position() compares
 *      epoch to RESULT_STALE_SEC wall time and returns freshness.
 *
 * Notes on camera selection
 * -------------------------
 * We ask AVFoundation for the first "default" built-in video device,
 * which on MacBooks is the lid camera. A user-facing picker is a follow
 * -up; the current design supports it trivially by storing a uniqueID
 * in settings and using deviceWithUniqueID: at start() time.
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
// up to ~5 frames (~160ms) of staleness before the reader treats the
// position as "unknown" and falls back to its default (usually zero).
// Longer windows let position survive a single missed frame without
// the pose visibly snapping; shorter windows feel more responsive when
// the camera occasionally hiccups.
static constexpr double RESULT_STALE_SEC = 0.2;

// Minimum / maximum blob area in pixels for the extractor. The PSVR
// LEDs at ~60cm from a 720p camera image at ~70deg HFOV occupy roughly
// 3-12 px diameter (7-100 px^2). The bounds reject single-pixel noise
// (< 4 px^2) and very large bright areas like a ceiling lamp in frame
// (> 500 px^2). Tuned empirically; adjust if a different camera or
// lighting produces consistently under- or over-sized blobs.
static constexpr double BLOB_MIN_AREA_PX = 4.0;
static constexpr double BLOB_MAX_AREA_PX = 500.0;

// Blob acceptance: a pixel is kept if EITHER
//   (a) it's blue-dominant: (B - max(R,G)) > BLUENESS_THRESH and
//       B > B_ABS_MIN   -- the typical daytime / lit-room case where
//       PSVR LEDs saturate just the blue channel (B ~ 255, R,G < 128)
//       while skin / room lighting rarely produces real blue-dominance.
//   OR
//   (b) it's fully saturated: min(R,G,B) > SAT_ABS_MIN   -- the dark-
//       room case where auto-exposure cranks gain and the LEDs blow
//       out to white (R=G=B=255). Blue-dominance would miss those
//       entirely because max(R,G) catches up to B.
// The two rules cover the two dominant lighting regimes. Noise stays
// out because (a) demands blue-dominance and (b) demands all three
// channels be near-saturated, which skin/clothing/light-gray surfaces
// don't reach even under heavy exposure.
static constexpr int BLUENESS_THRESH = 80;
static constexpr int B_ABS_MIN       = 160;
static constexpr int SAT_ABS_MIN     = 220;

static double now_sec() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
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
    cv::Mat channels_[4];
    cv::Mat blueness;
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
};

Worker::Worker() : impl_(std::make_unique<Impl>()) {}

Worker::~Worker() { stop(); }

bool Worker::is_running() const { return impl_->running.load(); }

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

bool Worker::start() {
    if (impl_->running.load()) return true;

    @autoreleasepool {
        // Pick the default built-in video camera. A follow-up can expose
        // a picker in the settings dialog and remember a uniqueID.
        AVCaptureDevice* dev =
            [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
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
    // lock. We do NOT modify bgra_view; we derive single-channel
    // temporaries from it.
    cv::Mat bgra(h, w, CV_8UC4, base, stride);

    // Split into B, G, R, A. OpenCV does a per-plane memcpy here —
    // unavoidable for channelwise ops but fast; 720p BGRA is ~3.5 MB.
    cv::split(bgra, s->channels_);

    // Blue-dominant branch: blueness = max(0, B - max(R,G)), then
    //   mask_blue = (blueness > BLUENESS_THRESH) AND (B > B_ABS_MIN)
    cv::Mat maxRG;
    cv::max(s->channels_[2], s->channels_[1], maxRG);   // [2]=R, [1]=G
    cv::subtract(s->channels_[0], maxRG, s->blueness);  // saturating subtract
    cv::Mat bright, mask_blue;
    cv::threshold(s->blueness, mask_blue, BLUENESS_THRESH, 255, cv::THRESH_BINARY);
    cv::threshold(s->channels_[0], bright, B_ABS_MIN, 255, cv::THRESH_BINARY);
    cv::bitwise_and(mask_blue, bright, mask_blue);

    // Saturated-bright branch: mask_sat = min(R,G,B) > SAT_ABS_MIN
    // (all three channels near 255 -> blown-out LED in a dark room).
    cv::Mat minRG, minAll, mask_sat;
    cv::min(s->channels_[2], s->channels_[1], minRG);
    cv::min(minRG, s->channels_[0], minAll);
    cv::threshold(minAll, mask_sat, SAT_ABS_MIN, 255, cv::THRESH_BINARY);

    // Union: a blob qualifies under either regime.
    cv::bitwise_or(mask_blue, mask_sat, s->mask);

    // Small morphological opening to kill speckle noise, then contour find.
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
    cv::morphologyEx(s->mask, s->mask, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(s->mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Per-contour centroid + area.
    std::vector<cv::Point2d> blobs;
    blobs.reserve(contours.size());
    for (const auto& c : contours) {
        const double area = cv::contourArea(c);
        if (area < BLOB_MIN_AREA_PX || area > BLOB_MAX_AREA_PX) continue;
        const cv::Moments m = cv::moments(c);
        if (m.m00 <= 0) continue;
        blobs.emplace_back(m.m10 / m.m00, m.m01 / m.m00);
    }

    // Copy the frame into an OpenCV-owned BGR buffer BEFORE unlocking
    // the IOSurface. The downstream overlay-drawing block needs to
    // read pixel data, but `bgra` is just a zero-copy view of the
    // CVPixelBuffer; once we unlock it the AVCaptureSession is free
    // to recycle that memory for the next frame, and reading bgra
    // afterwards is a use-after-free (corrupted preview, occasional
    // segfault). Allocating `vis` here keeps the rest of the
    // function safe and lets us drop the lock immediately.
    cv::Mat vis;
    cv::cvtColor(bgra, vis, cv::COLOR_BGRA2BGR);

    CVPixelBufferUnlockBaseAddress(buf, kCVPixelBufferLock_ReadOnly);

    // Hand off to constellation stage. It may return no_solution; in
    // that case we still record the blob count so the user can tell
    // whether the camera even saw LEDs vs. a PnP / ID failure.
    const double yaw   = s->yaw_rad.load(std::memory_order_relaxed);
    const double pitch = s->pitch_rad.load(std::memory_order_relaxed);
    const double roll  = s->roll_rad.load(std::memory_order_relaxed);

    psvr_constellation::Result r =
        s->solver_state.solve(blobs, w, h, yaw, pitch, roll);

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
    // bulk copy; fine even at 60 Hz).
    {
        std::lock_guard<std::mutex> lk(s->diag_mu);
        s->diag.frames_captured++;
        if (!blobs.empty()) s->diag.frames_with_any_blob++;
        if (r.ok)           s->diag.pnp_ok_count++;
        s->diag.last_n_blobs   = (int)blobs.size();
        s->diag.last_n_matched = r.n_matched;
        s->diag.last_pnp_ok    = r.ok;
        if (r.ok) {
            s->diag.last_x_cm = r.x_cm;
            s->diag.last_y_cm = r.y_cm;
            s->diag.last_z_cm = r.z_cm;
        }
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
