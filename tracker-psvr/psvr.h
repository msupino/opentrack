/* PSVR IMU tracker for opentrack (macOS).
 * Reads IMU sensor data from the PlayStation VR headset over USB HID
 * and provides yaw/pitch/roll to opentrack directly (no UDP bridge needed).
 */
#pragma once

#include "api/plugin-api.hpp"
#include "options/options.hpp"

#include <atomic>
#include <array>
#include <memory>
#include <thread>
#include <vector>
#include <mutex>
#include <cstdio>
#include <cstdint>

#include <QObject>
#include <QIcon>
#include <QCheckBox>
#include <QDialogButtonBox>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QWidget>

#include <IOKit/hid/IOHIDManager.h>
#include <CoreFoundation/CoreFoundation.h>

// Optional webcam-based position tracker; full definition in
// psvr_camera.h, included by psvr.cpp only.
#ifdef PSVR_HAS_CAMERA
namespace psvr_cam { class Worker; }
#endif

using namespace options;

struct psvr_settings : opts {
    value<bool> enable_mirror;
    value<bool> enable_diag_log;
    // Optional: use a webcam to track the PSVR's blue LED constellation
    // for head POSITION (X/Y/Z). Orthogonal to the IMU path, which
    // always provides rotation. Defaults off because the first frame
    // triggers macOS's Camera TCC prompt; users opt in via the dialog.
    value<bool> enable_camera;
    // EXPERIMENTAL: send a periodic HID command to the PSVR to defeat
    // its built-in 8-minute auto-sleep. The command byte and interval
    // are both ini-tweakable so we can iterate over the unexplored
    // command space (PSVRFramework documented 0x11 / 0x17 / 0x21 /
    // 0x23, but PS4 firmware sends 0x1A and 0x1F too, with unknown
    // semantics) without rebuilding. Default OFF because the wrong
    // command byte may disrupt the IMU stream the way periodic
    // re-activation did when we tried it earlier; users opt in to
    // iterate.
    //
    // To use: set keepalive-enable=true, leave keepalive-cmd at 0x1F
    // for first probe, watch /tmp/psvr-diag.log for the first 12 min.
    // If the headset stays awake AND IMU rate stays ~200 Hz, the byte
    // works. If IMU rate drops to 0 or jitters, change keepalive-cmd
    // to 0x1A and try again. Reasonable values to try in order:
    //   0x1F, 0x1A, 0x40, 0xA0, 0x15
    //
    // keepalive_cmd is a QString rather than int so the ini can use
    // hex notation ("0x1F") which is the natural representation for
    // a HID command byte. We parse with QString::toInt(nullptr, 0),
    // so "31", "0x1F", "0o37" all work; bad input falls back to 0x1F.
    value<bool>    keepalive_enable;
    value<QString> keepalive_cmd;
    value<int>     keepalive_interval_s;
    psvr_settings() :
        opts("psvr-tracker"),
        // Default OFF: turning the mirror on is what triggers macOS's
        // Screen Recording (plus System Audio, on macOS 15+) permission
        // prompt, so we shouldn't ask for that permission on first run
        // of a fresh install. Users who want the SBS mirror opt in
        // via the checkbox in the tracker settings dialog.
        enable_mirror(b, "enable-sbs-mirror", false),
        enable_diag_log(b, "enable-diag-log", false),
        enable_camera(b, "enable-camera", false),
        keepalive_enable(b, "keepalive-enable", false),
        keepalive_cmd(b, "keepalive-cmd", QStringLiteral("0x1F")),
        keepalive_interval_s(b, "keepalive-interval-s", 60)
    {}
};

class PSVRTracker : public ITracker
{
public:
    PSVRTracker();
    ~PSVRTracker() override;

    module_status start_tracker(QFrame*) override;
    void data(double* data) override;

private:
    std::atomic<double> yaw_{0}, pitch_{0}, roll_{0};
    std::atomic<bool> stop_{false};

    // Automatic gyro-bias calibration.
    // At startup we spend ~2s sampling the gyro while the headset is held
    // still; the averaged raw gyro reading on each axis IS the bias, so
    // we just subtract it from every subsequent sample. Without this, the
    // yaw axis drifts at ~0.27 deg/s (~16 deg/min) because tiny DC offsets
    // in the gyro integrate unchecked. After calibration drift drops by
    // one to two orders of magnitude. Touched only by the HID report
    // thread except for the atomic guard, which gates whether the filter
    // publishes poses to opentrack.
    static constexpr int CALIB_SAMPLES = 400;   // ~2s at 200Hz (2 groups per 100Hz report)

    // Discard the first second of HID samples after the stream starts
    // before we begin averaging for the gyro-bias estimate. Rationale:
    // the PSVR's IMU appears to deliver a short transient (inconsistent
    // DT, stale or partially-initialized values) for the first few
    // hundred ms after the 0x17/0x11 activation burst completes. Those
    // samples are real-looking but systematically biased, and averaging
    // them in produces a slightly worse bias estimate than the same
    // 400 samples taken one second later. One second (200 samples at
    // 200 Hz) is plenty to get past the transient without making the
    // whole calibration noticeably longer.
    static constexpr int CALIB_WARMUP_SAMPLES = 200;
    int calib_warmup_count_{0};

    std::atomic<bool> calibrated_{false};
    int calib_count_{0};
    double calib_gyro_accum_[3]{};
    double calib_accel_accum_[3]{};
    double gyro_bias_[3]{};

    // "Start pressed but headset isn't really usable" detection.
    //
    // Two failure modes exist in practice:
    //   * NO_DATA: the HID stream is silent. Happens when the USB cable
    //     is unplugged, Input Monitoring permission hasn't been granted
    //     to opentrack, or the PSVR's processor unit has no power. The
    //     worker loop detects this when calib_count_ stays zero past
    //     NO_DATA_TIMEOUT_SEC wall time.
    //   * NO_GRAVITY: HID reports arrive but the accelerometer is
    //     streaming near-zero samples, meaning the headset itself is
    //     powered OFF (the in-line box is alive on USB but the HMD
    //     hasn't been woken up with its power button). Detected when
    //     the averaged |accel| over the 2s calibration window is
    //     outside a generous gravity band. Without this check the
    //     user's gyro bias gets calibrated against pure noise and the
    //     tracker drifts wildly on any real head motion afterwards.
    //
    // On either failure we leave `calibrated_` false so no pose is ever
    // published, and the UI banner flips from "calibrating" to a red
    // error message with recovery instructions.
    static constexpr int CALIB_FAIL_NONE       = 0;
    static constexpr int CALIB_FAIL_NO_DATA    = 1;
    static constexpr int CALIB_FAIL_NO_GRAVITY = 2;
    // The headset was on the user's head when calibration was
    // requested. Calibration has to happen on a flat still surface
    // so breathing / pulse / body sway don't leak into the bias
    // average; the HMD's own proximity bit in the HID report tells
    // us when it's being worn, so we refuse up front instead of
    // silently producing a bad calibration.
    static constexpr int CALIB_FAIL_WORN       = 3;
    // Hand-held / shaking during calibration: any raw gyro sample
    // during the 2-second averaging window exceeded the "clearly
    // not sitting still" threshold. At rest the post-bias gyro
    // peaks at ~2 dps; deliberate hand motion is ≥5-10 dps, so the
    // separation is clean.
    static constexpr int CALIB_FAIL_MOVING     = 4;
    std::atomic<int> calib_failure_{CALIB_FAIL_NONE};

    // Peak |raw gyro| (dps, any axis) seen during the calibration
    // averaging window. Checked at the end; if above
    // CALIB_MAX_MOTION_DPS, the calibration is rejected as
    // motion-contaminated.
    double calib_peak_gyro_dps_{0.0};
    static constexpr double CALIB_MAX_MOTION_DPS = 5.0;

    // Auto-retry counter for the motion-rejection path. Typical first-
    // calibration failure pattern: the gyro's post-activation settling
    // transient extends ~0.5 s past the CALIB_WARMUP window on a cold
    // PSVR, so the averaging window catches a decaying tail spike that
    // pushes peak |gyro| above 5 dps and flunks the check. By the time
    // the user notices and clicks Re-calibrate, the stream has been
    // running for many more seconds and the sensor is rock stable,
    // which is why Re-calibrate usually "just works" where Start
    // didn't. We remove the user-visible failure for those first few
    // attempts by silently resetting the accumulators and retrying up
    // to CALIB_AUTO_RETRIES times before surfacing CALIB_FAIL_MOVING.
    // The counter is replenished when the user manually clicks
    // Re-calibrate (in case they *are* holding the headset and the
    // retries should all fail so they see the banner).
    static constexpr int CALIB_AUTO_RETRIES = 2;
    int calib_auto_retries_left_{CALIB_AUTO_RETRIES};

    // Status bits parsed out of the 64-byte HID input report, byte 8:
    //   bit 0 - Worn (proximity sensor detects head contact)
    //   bit 1 - INVERTED DisplayActive (0 = display on, 1 = off)
    //   bit 3 - Muted
    //   bit 4 - EarphonesConnected
    // And byte 48 = firmware-side IMU calibration state:
    //   255 = cold boot, 0-3 = calibrating, 4 = calibrated
    // These are set from report_cb on the HID thread; atomics are
    // overkill for readers on the same thread but match the style
    // of the other cross-thread state here and enable future readers
    // (UI thread / filters) to consume them safely.
    std::atomic<bool>    worn_{false};
    std::atomic<bool>    display_active_{false};
    std::atomic<uint8_t> fw_cal_status_{255};
    std::atomic<uint16_t> ir_sensor_{0};     // 0=far, 1023=near

    // Wall-clock (CFAbsoluteTime) of the last HID input report consumed.
    // Used for on-stall keepalive recovery: if the PSVR auto-sleeps
    // after ~8 min of inactivity, HID reports stop arriving and this
    // value stops updating; the worker loop detects that and fires an
    // activation burst to wake the headset. Touched only from the
    // CFRunLoop thread (both report_cb and worker_loop), so no atomic
    // is needed. A zero value means "no report has arrived yet this
    // session"; the worker's stall watchdog skips the check in that
    // state to give USB startup its full NO_DATA_TIMEOUT_SEC window.
    double last_report_time_{0.0};

    // Per-sample device timestamp tracking for gyro integration dt.
    // Each IMU sample group in the PSVR HID report carries a 24-bit
    // microsecond tick counter in its first 4 bytes (wraps at
    // 0xFFFFFF). We compute dt from the tick delta rather than from
    // wall-clock — wall-clock has OS-scheduler jitter of hundreds of
    // microseconds under system load; the device clock is the IMU's
    // hardware sample-rate crystal and is rock-steady.
    // `last_sample_tick_valid_` gates the very first sample since
    // there's no prior reference; it falls back to DT_FALLBACK once,
    // then uses real deltas from then on.
    // Cross-referenced with OpenHMD/drv_psvr and PSVRFramework, which
    // both use this approach. Expected delta: ~500 us (= 2000 Hz).
    uint32_t last_sample_tick_{0};
    bool     last_sample_tick_valid_{false};
    // How long the HID stream must be silent before we try to wake
    // the headset. 5 s is comfortably above the PSVR's per-report
    // jitter (~0.5 ms at 2000 Hz) and below the ~8 min auto-sleep
    // timer that motivated this feature, so in practice this only
    // fires after the headset has genuinely dozed off.
    static constexpr double STALL_RECOVERY_SEC = 5.0;

    // Wall-clock (CFAbsoluteTime) marker set just before the worker
    // enters its main runloop; used by the NO_DATA watchdog and the
    // silent-stream nudge below.
    //
    // On macOS, IOHIDManagerOpen + the 0x17/0x11/0x23 activation
    // burst can take 10-20 s before the first HID input report
    // reaches us on a cold-boot PSVR (the exact latency depends on
    // USB port, hub chain, whether another process just opened the
    // device, and whether the headset's firmware is booting fresh
    // from deep-off). A shorter timeout false-positives on slow-
    // but-healthy startups; a genuinely-off headset produces no
    // reports ever, so 25 s is still responsive for real failures.
    //
    // At the NUDGE boundary (half the full timeout), if we've still
    // heard nothing, we re-fire the activation burst ONCE, since a
    // common failure mode is that the first activation was sent
    // while the PSVR's own firmware was still booting and it got
    // dropped on the floor. nudge_sent_ is the idempotency guard:
    // we only nudge once per session to avoid the stream-stall
    // feedback loop the old 10-s periodic keepalive had.
    double worker_start_time_{0.0};
    bool   nudge_sent_{false};
    static constexpr double NO_DATA_NUDGE_SEC    = 12.0;
    static constexpr double NO_DATA_TIMEOUT_SEC  = 25.0;

    // Flipped true on the first HID report ever consumed. Used by
    // the UI poll timer to upgrade the "waiting for USB" banner to
    // the "calibrating" banner so the 2-second promise stays honest.
    // Cleared on Re-calibrate so the banner pair repeats correctly.
    std::atomic<bool> samples_flowing_{false};

    // Accel magnitude band (in g) accepted as "actually measuring
    // gravity". A still PSVR reads |a| ~1.00g; a PSVR-off-but-USB-alive
    // condition reads near 0. The band is generous enough that even a
    // wobbly hand-held calibration passes; tightening it would produce
    // false positives.
    static constexpr double GRAVITY_MIN_G = 0.6;
    static constexpr double GRAVITY_MAX_G = 1.4;

    // Online bias tracking ("ZUPT"). The initial 2s calibration leaves
    // some residual gyro bias, and that residual changes slowly with
    // temperature as the IMU warms up. Yaw has no gravity reference
    // so every mdeg/s of residual integrates to pure drift forever.
    //
    // Fix: while the headset sits STILL we know the true angular
    // velocity is zero, so the raw gyro reading IS the current bias.
    // We low-pass gyro_bias_ toward the raw gyro during stillness,
    // with a slow enough gain that brief pauses between real head
    // motions don't contaminate it. Only runs post-calibration.
    //
    // Stillness = (a) |accel| close to 1g, (b) post-bias gyro small
    // on all axes, (c) the above has held for STILL_MIN_STREAK samples
    // already (so a 1-sample accidental zero doesn't start tracking).
    // (No accel-quiet check is performed for ZUPT bias updates: the
    // gyro measures angular velocity, not linear, so linear motion
    // of the headset has no effect on what the bias estimate should
    // be. Removing the accel check lets ZUPT keep correcting biases
    // even while the user is walking around with the headset on, as
    // long as the corresponding gyro axis is not rotating.)
    // 3.0 dps and 200 samples (100 ms @ 2000 Hz) for the gyro side of
    // the still detector. Reasoning:
    //   * The PSVR's ±2000 dps gyro has a per-sample post-bias noise
    //     peak of ~2 dps observed in diag logs at true rest (with the
    //     correct GYRO_LSB_PER_DEG=16.384 scale). 3 dps gives comfortable
    //     margin above noise so ZUPT can actually engage when stationary.
    //   * 3 dps is also the threshold that catches any real intentional
    //     head/headset motion: even slow user-perceptible rotation is
    //     ≥ 5 dps in practice (a 360° turn in 1 minute = 6 dps). True
    //     "below 3 dps" motion is the noise floor or a dead-still device.
    //   * 200 samples (100 ms) confirmation buffer prevents single-
    //     sample noise dips from triggering ZUPT during real motion.
    //   * Earlier tightening to 0.5 dps was based on noise floor
    //     observed at the WRONG gyro scale (8x too small); correcting
    //     GYRO_LSB_PER_DEG raised both real motion AND noise by 8x,
    //     so the threshold has to come up to match.
    static constexpr double STILL_MAX_GYRO_DPS   = 3.0;  // per axis, post-bias
    static constexpr int    STILL_MIN_STREAK     = 200;
    // Per-sample EMA gain for bias tracking. Gives ~10s half-life on
    // the bias estimate during continuous stillness; if you want
    // faster settling during warm-up raise this, but going too high
    // leaks real slow head motion into the bias.
    static constexpr double ZUPT_BIAS_ALPHA      = 3.5e-4;
    // Per-axis stillness streak. Each gyro axis maintains its own
    // counter, so a fast yaw turn doesn't reset the pitch and roll
    // bias-tracking streaks. This means pitch and roll bias keep
    // refining whenever the user holds those axes still (which is
    // most of the time, even during a yaw turn — most natural head
    // motion is around one axis at a time). The yaw axis is the
    // payoff axis: it's the only rotation that has no gravity
    // reference, so its bias estimate is the only one whose error
    // ever shows up in the user's view as drift.
    int still_streak_axis_[3]{0, 0, 0};

    // dt bounds for gyro integration. Protects against garbage tick
    // values (e.g., a stale buffered sample at session start) without
    // hard-failing: clamp, integrate, keep going.
    static constexpr double DT_FALLBACK = 0.0005;  // first sample, ~2000 Hz
    static constexpr double DT_MIN      = 0.0001;  // 10 kHz upper rate cap
    static constexpr double DT_MAX      = 0.05;    // 20 Hz lower rate cap
    std::thread worker_;
    // runloop_ is published by the worker as it enters its loop and cleared
    // when the loop exits. The destructor reads it from the UI thread to
    // call CFRunLoopStop; atomic keeps that cross-thread read well-defined.
    std::atomic<CFRunLoopRef> runloop_{nullptr};
    psvr_settings s_;

    // Every matched PSVR needs its own HID input-report buffer that lives
    // as long as the callback is registered. A single shared `static`
    // buffer here would be written into from multiple HID devices in
    // parallel, so each device gets its own heap-allocated fixed-size
    // buffer whose address stays stable even if the vector reallocates
    // (the unique_ptrs hold the stable pointers; the vector's own moves
    // only shuffle the unique_ptrs). The HID callback context (`this`)
    // is used instead of the buffer pointer for callback dispatch.
    struct matched_device {
        IOHIDDeviceRef device;
        std::unique_ptr<std::array<uint8_t, 256>> report_buf;
    };
    std::mutex devices_mu_;
    std::vector<matched_device> devices_;

    // Calibration status UI, installed in the tracker's area of the main
    // opentrack window while the ~2s gyro-bias sampling runs, cleared
    // once `calibrated_` flips to true. Created/accessed only on the
    // UI thread (start_tracker runs there, and the QTimer we install
    // also fires on it).
    QFrame*      tracker_frame_{nullptr};  // kept for re-calibration
    QLabel*      calib_label_{nullptr};
    QTimer*      calib_poll_timer_{nullptr};
    QPushButton* recal_button_{nullptr};   // visible after calibration

    // Handshake between the UI thread (re-calibrate button click) and
    // the HID worker thread (which owns the calibration accumulators).
    // The worker clears this and resets state inside process_group so
    // the non-atomic accumulators are only ever touched on that thread.
    std::atomic<bool> recalibrate_request_{false};

    void install_calibration_banner();    // UI-thread only

    // Update the calibration QLabel (text + CSS), then if the camera
    // preview is active, also push the same text to the camera-worker
    // overlay so the message appears on the live video and the QLabel
    // can be hidden (avoiding the "two banners stacked" visual). Color
    // is RGB 0-255; CSS-side colors come from the shared constants
    // table in psvr.cpp. UI-thread only.
    void publish_status_banner(QLabel* lbl,
                               const QString& text,
                               const char* css_color_hex,
                               int r, int g, int b);
    void show_recalibrate_button();       // UI-thread only

    // Optional diagnostic log. Enabled by a checkbox in the settings
    // dialog ("Write /tmp/psvr-diag.log"); off by default. When on,
    // process_group writes one tab-separated row per second covering
    // pose, gyro-after-bias, accel, accel-derived pitch/roll and IMU
    // sample rate. Touched from the HID worker thread only; opened
    // in start_tracker on the UI thread before the worker spawns and
    // closed in the destructor after the worker has joined, so no
    // file-handle race is possible.
    FILE* diag_log_{nullptr};
    double diag_last_log_time_{0};
    double diag_last_rate_time_{0};
    uint64_t diag_sample_count_{0};
    uint64_t diag_rate_base_count_{0};
    double diag_last_sample_rate_{0};
    double diag_start_time_{0};

    // Optional webcam-based position tracker. Forward-declared so the
    // header doesn't leak AVFoundation/OpenCV types to consumers. The
    // unique_ptr is non-null only when camera tracking is enabled AND
    // the Worker::start() call at Start-time returned true. The full
    // type is included in psvr.cpp where both the ctor (which creates
    // the Worker) and dtor (which must see the complete type) live.
#ifdef PSVR_HAS_CAMERA
    std::unique_ptr<psvr_cam::Worker> camera_worker_;
#endif
    // Head position in opentrack units (cm), published by the camera
    // worker when PnP succeeds. When the camera is disabled or has no
    // fresh solution, these stay at 0 (or the last good value); data()
    // reads them via std::memory_order_relaxed.
    std::atomic<double> head_x_{0}, head_y_{0}, head_z_{0};
    // Set by data() consumers so diag logging can tell which fields
    // are actually going out to opentrack. Redundant with
    // camera_worker_->is_running() when enabled; kept for log clarity
    // when it's not.
    std::atomic<bool> camera_position_fresh_{false};

    // Inline camera preview: a QLabel added to the same tracker-frame
    // that opentrack passes to start_tracker, so the live feed shows
    // up exactly where the other tracker plugins' previews do (below
    // the settings area in the main window) rather than in a separate
    // popup. The QTimer ticks on the GUI thread and pulls RGB data
    // from the camera worker. Created on start_tracker, deleted in
    // the destructor via deleteLater. Buffer reused across ticks.
    QLabel*              camera_preview_label_{nullptr};
    QTimer*              camera_preview_timer_{nullptr};
    std::vector<uint8_t> camera_preview_buf_;

    // EXPERIMENTAL: periodic HID keepalive timer aimed at defeating
    // the PSVR's 8-minute auto-sleep. Only created when settings.
    // keepalive_enable is true. Owned by tracker_frame_ so Qt's
    // parent chain cleans it up. The timer fires send_raw_to_all
    // with the user-configured command byte (default 0x1F) and an
    // empty payload; we log each fire to diag_log_ so the post-mortem
    // shows whether the keepalive correlated with the headset
    // staying awake or not.
    QTimer* keepalive_timer_{nullptr};

    void worker_loop();
    void send_activation(IOHIDDeviceRef device);
    void send_activation_to_all();           // iterates devices_ under the lock
    void send_cinematic_mode_to_all();       // 0x23 VRMode(OFF) to all devices
    void send_raw_to_all(uint8_t cmd, const uint8_t* payload, size_t len);

    static void report_cb(void* context, IOReturn result, void* sender,
                          IOHIDReportType type, uint32_t reportID,
                          uint8_t* report, CFIndex reportLength);
    static void device_matched_cb(void* context, IOReturn result, void* sender,
                                  IOHIDDeviceRef device);
    static void device_removed_cb(void* context, IOReturn result, void* sender,
                                  IOHIDDeviceRef device);

    void process_group(const uint8_t* buf);
};

class PSVRDialog : public ITrackerDialog
{
    Q_OBJECT
public:
    PSVRDialog();
    void register_tracker(ITracker*) override {}
    void unregister_tracker() override {}
    // Opt into being shown as a "Tracker" tab inside the global Options
    // dialog (in addition to the wrench-icon standalone dialog). The
    // standalone path still works; embedding just adds a convenience tab
    // alongside Shortcuts/Output/Game-detection.
    bool embeddable() noexcept override { return true; }
    // When embedded as a tab, the Options dialog has its own OK/Cancel
    // button box at the bottom and delegates to our save()/reload() on
    // accept/reject. Hide our own QDialogButtonBox to avoid the duplicate
    // button row, and implement save()/reload() so the Options OK button
    // persists our settings the same way the standalone OK does.
    void set_buttons_visible(bool x) override;
    void save() override;
    void reload() override;
private:
    psvr_settings s_;
    QCheckBox* mirror_box_{nullptr};
    QCheckBox* diag_log_box_{nullptr};
    QCheckBox* camera_box_{nullptr};
    // Held as a member so set_buttons_visible() can hide it when this
    // dialog is rendered as an embedded tab inside the global Options
    // dialog (which provides its own OK/Cancel and would otherwise stack
    // a duplicate button row underneath ours).
    QDialogButtonBox* buttons_{nullptr};
};

class PSVRMetadata : public Metadata
{
    Q_OBJECT
    QString name() override { return tr("PSVR (IMU)"); }
    QIcon icon() override { return QIcon(":/images/opentrack.png"); }
};
