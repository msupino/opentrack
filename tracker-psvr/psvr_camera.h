/* Camera-based LED constellation worker for the PSVR tracker.
 *
 * AVFoundation capture + OpenCV blue-blob extractor + (constellation PnP
 * when the matching module is linked in). Runs on its own dispatch queue
 * so the Qt main thread and the HID/CFRunLoop thread stay responsive.
 *
 * Gated by PSVR_HAS_CAMERA at build time (defined by CMakeLists when
 * OpenCV is available). The plugin always compiles a stubbed version of
 * this class so the rest of the plugin can be written as if camera
 * tracking is always available; when the define is unset the stub just
 * returns "no position".
 *
 * Threading contract
 * ------------------
 * * start()/stop()/is_running() are called from the UI (Qt) thread.
 * * set_prior() is called from the HID/CFRunLoop thread every IMU sample
 *   (up to ~2 kHz). It's just an atomic store of three doubles.
 * * get_position()/diag() are called from the HID/CFRunLoop thread when
 *   process_group wants to build a pose output or diag-log row.
 * * The AVFoundation capture delegate fires on a private dispatch queue.
 *   It reads the latest prior, runs the extractor + PnP, and atomically
 *   publishes the result.
 *
 * No locks are held across any OpenCV call: the hot path is entirely
 * lock-free via std::atomic<double>s. A single mutex guards bulk-copy
 * of the Diag snapshot for logging.
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace psvr_cam {

// Per-frame camera-tracking result.
struct Result {
    double x_cm{0};       // PSVR center in camera frame, cm (+X right, +Y up, -Z forward)
    double y_cm{0};
    double z_cm{0};
    int    n_blobs{0};    // how many blue blobs passed the extractor
    int    n_matched{0};  // how many matched to LEDs (constellation stage)
    bool   pnp_ok{false}; // true if solvePnP accepted the match this frame
};

// Diagnostic snapshot, safe to read from any thread via diag().
struct Diag {
    uint64_t frames_captured{0};
    uint64_t frames_with_any_blob{0};
    uint64_t pnp_ok_count{0};
    int      last_n_blobs{0};
    int      last_n_matched{0};
    bool     last_pnp_ok{false};
    double   last_x_cm{0}, last_y_cm{0}, last_z_cm{0};
};

class Worker {
public:
    Worker();
    ~Worker();

    // Idempotent. Returns false on AVFoundation / permission / OpenCV
    // initialization failure; the plugin falls back to IMU-only on false.
    // When no camera is usable (stub build or no camera found), this
    // returns false immediately without side effects.
    bool start();

    // Optional camera-name preference, written by the plugin from the
    // settings dialog BEFORE start() is called. Empty string keeps the
    // legacy "AVFoundation default" behavior (lid camera on MacBooks);
    // a non-empty value is matched against AVCaptureDevice uniqueID
    // first, then localizedName; an unmatched value also falls back to
    // the default. Storage is just a std::string; the only writer is
    // the Qt UI thread before start(), the only reader is start()
    // itself, so no synchronization beyond happens-before from the
    // single-shot call ordering is required.
    void set_desired_camera_name(const std::string& s);

    // Idempotent. Always safe to call.
    void stop();

    bool is_running() const;

    // Fast path: HID/CFRunLoop thread publishes the latest IMU attitude
    // so the camera pipeline can (a) project the 9-LED model into image
    // space as a prior for blob-to-LED matching, and (b) produce a
    // position output that's spatially consistent with the IMU's
    // rotation at the same moment in time. Radians.
    void set_rotation_prior(double yaw_rad, double pitch_rad, double roll_rad);

    // Returns true with x/y/z populated (cm) if a recent PnP result is
    // available and fresh (< RESULT_STALE_SEC). Otherwise false and the
    // caller should fall back to its previous behavior (e.g. zero).
    bool get_position(double* x_cm, double* y_cm, double* z_cm) const;

    // Fetch the latest preview frame as packed RGB888 (3 bytes per
    // pixel, no padding, top-left origin). The frame is already
    // annotated with debug overlays: red dots for detected blobs,
    // green circles for projected LED positions at the prior pose,
    // yellow lines for matched pairs, and a text overlay with the
    // solver's latest state. Returns true if a frame is available and
    // resizes `out` to w*h*3 bytes; false on first frames before
    // capture has started.
    bool fetch_preview_rgb(std::vector<uint8_t>* out,
                           int* out_w, int* out_h) const;

    // Push a status banner to be drawn on top of every preview frame.
    // Multi-line via embedded '\n'. Empty text removes the banner.
    // Color is RGB 0-255. Mutex-protected; safe to call from any
    // thread - the camera-callback thread reads under the same mutex
    // exactly once per frame.
    void set_status_banner(const std::string& text,
                           int r, int g, int b);

    Diag diag() const;

    // Opaque private state. Declared public only so the Objective-C
    // capture delegate can hold a pointer to it (Obj-C @property types
    // must name complete-or-forward-declared types). The full layout
    // stays in psvr_camera.mm; no consumer outside that file can do
    // anything with an `Impl*`.
    struct Impl;
private:
    std::unique_ptr<Impl> impl_;
};

} // namespace psvr_cam
