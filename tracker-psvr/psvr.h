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
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include <IOKit/hid/IOHIDManager.h>
#include <CoreFoundation/CoreFoundation.h>

using namespace options;

struct psvr_settings : opts {
    value<bool> enable_mirror;
    value<bool> enable_diag_log;
    psvr_settings() :
        opts("psvr-tracker"),
        // Default OFF: turning the mirror on is what triggers macOS's
        // Screen Recording (plus System Audio, on macOS 15+) permission
        // prompt, so we shouldn't ask for that permission on first run
        // of a fresh install. Users who want the SBS mirror opt in
        // via the checkbox in the tracker settings dialog.
        enable_mirror(b, "enable-sbs-mirror", false),
        enable_diag_log(b, "enable-diag-log", false)
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

    // Local gravity magnitude as measured by THIS specific PSVR's
    // accelerometer during the calibration pass. Captured at the end
    // of calibration purely as a diagnostic / log entry: it tells us
    // by how much this unit's accel scale differs from the ideal 1.0g
    // (commonly off by 1-5% from the factory). Not consumed by the
    // current ZUPT detector, which is gyro-only - kept around because
    // it's a one-line capture and a useful signal for any future
    // accel-aware filter (e.g., gravity-magnitude-aware tilt rejection).
    double calib_gravity_mag_{1.0};

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
    // How long the HID stream must be silent before we try to wake
    // the headset. 5 s is comfortably above the PSVR's per-report
    // jitter (~0.5 ms at 2000 Hz) and below the ~8 min auto-sleep
    // timer that motivated this feature, so in practice this only
    // fires after the headset has genuinely dozed off.
    static constexpr double STALL_RECOVERY_SEC = 5.0;

    // Wall-clock (CFAbsoluteTime) marker set just before the worker
    // enters its main runloop; used purely for the NO_DATA timeout.
    //
    // The timeout is deliberately generous: on macOS,
    // IOHIDManagerOpen + the 0x17/0x11 activation burst can take
    // 5-10 s before the first HID input report reaches us (the
    // exact latency depends on USB port, hub chain, whether another
    // process had the device open, etc.). A shorter timeout false-
    // positives on slow-but-healthy startups. A genuinely-off
    // headset produces no reports ever, so 15 s is still responsive
    // for real failures.
    double worker_start_time_{0.0};
    static constexpr double NO_DATA_TIMEOUT_SEC = 15.0;

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

    // Wall-clock (CFAbsoluteTime) of the last consumed sample. Used to
    // compute the actual dt for gyro integration on each sample, instead
    // of trusting a hardcoded constant. Diag logs revealed the PSVR
    // delivers ~2000 Hz of process_group calls (1000 Hz HID reports,
    // 2 groups each), not the 200 Hz the original code assumed; using
    // a 0.005 s constant integrated yaw 10x too fast on real motion.
    // Now we measure dt every sample, with a sane bound so a one-shot
    // stream stall doesn't blow up integration with a 5 s "step".
    double last_sample_time_{0.0};
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

    void worker_loop();
    void send_activation(IOHIDDeviceRef device);
    void send_activation_to_all();  // iterates devices_ under the lock

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
private:
    psvr_settings s_;
    QCheckBox* mirror_box_{nullptr};
    QCheckBox* diag_log_box_{nullptr};
};

class PSVRMetadata : public Metadata
{
    Q_OBJECT
    QString name() override { return tr("PSVR (IMU)"); }
    QIcon icon() override { return QIcon(":/images/opentrack.png"); }
};
