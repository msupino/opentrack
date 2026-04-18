/* PSVR IMU tracker - reads sensor data directly from the PlayStation VR
 * over USB HID and feeds yaw/pitch/roll to opentrack.
 *
 * Axis mapping determined empirically for PSVR CUH-ZVR2 worn on head:
 *   YAW   = -gyro[0]     (gravity axis)
 *   PITCH = +gyro[2]
 *   ROLL  = -gyro[1]
 */
#include "psvr.h"
#include "psvr_mirror.h"

#include <cmath>
#include <cstring>
#include <QDebug>
#include <QCheckBox>
#include <QDialogButtonBox>
#include <QLabel>
#include <QVBoxLayout>
#include <QPushButton>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {
constexpr int   PSVR_VID = 0x054c;
constexpr int   PSVR_PID = 0x09af;
// PSVR IMU scale factors. Confirmed against multiple open-source PSVR
// reverse-engineering projects (OpenHMD, psmove, psvr-ble): the gyro
// is configured at ±2000 dps full scale (16.384 LSB/dps) and the accel
// at ±2g full scale (1024 LSB/g). The previous gyro constant of 131.2
// was the value for a ±250 dps gyro, which made every integrated angle
// 8x too small — invisible on pitch/roll because the complementary
// filter snaps them back to the accel's gravity reading every sample,
// but very visible on yaw, which has no absolute reference and would
// register a 90° physical rotation as ~11° of integrated yaw.
constexpr double GYRO_LSB_PER_DEG = 16.384;
constexpr double ACCEL_LSB_PER_G  = 1024.0;
// dt is now measured per-sample; see PSVRTracker::last_sample_time_ for
// rationale (the assumed 200 Hz constant was off by 10x — the PSVR
// actually emits ~2000 Hz of process_group calls).
constexpr double ALPHA = 0.98;    // complementary filter weight

// Axis remap: which raw IMU axis drives which output rotation.
// Determined empirically for PSVR CUH-ZVR2 worn on head:
//   - YAW   = -gyro[0]  (vertical axis; sign flipped so a left turn
//                        of the head produces a left turn of the view)
//   - PITCH = +gyro[2]  (look up = view up)
//   - ROLL  = -gyro[1]  (ear-to-shoulder = view tilts the same way;
//                        was +1 originally, but on the worn headset
//                        that produced an ear-to-shoulder-reversed
//                        roll for every reporter)
constexpr int    YAW_SRC = 0, PITCH_SRC = 2, ROLL_SRC = 1;
constexpr double YAW_SIGN = -1.0, PITCH_SIGN = 1.0, ROLL_SIGN = -1.0;

int16_t read_i16(const uint8_t* b)
{
    return (int16_t)(b[0] | (b[1] << 8));
}

// Accelerometer-only pitch/roll in degrees from a gravity-aligned accel
// sample. Matches the formulas used inline in the complementary filter.
double accel_pitch_deg(const double a[3])
{
    return PITCH_SIGN *
        std::atan2(a[PITCH_SRC],
                   std::sqrt(a[ROLL_SRC]*a[ROLL_SRC] + a[YAW_SRC]*a[YAW_SRC]))
        * 180.0 / M_PI;
}
double accel_roll_deg(const double a[3])
{
    return ROLL_SIGN *
        std::atan2(a[ROLL_SRC],
                   std::sqrt(a[PITCH_SRC]*a[PITCH_SRC] + a[YAW_SRC]*a[YAW_SRC]))
        * 180.0 / M_PI;
}
}

PSVRTracker::PSVRTracker() = default;

PSVRTracker::~PSVRTracker()
{
    stop_ = true;
    if (CFRunLoopRef rl = runloop_.load(std::memory_order_acquire))
        CFRunLoopStop(rl);
    if (worker_.joinable())
        worker_.join();
    psvr_mirror_stop();
    // deleteLater is safe even if these widgets already went away with
    // their parent frame; it's a no-op on nullptr.
    if (calib_poll_timer_) { calib_poll_timer_->stop(); calib_poll_timer_->deleteLater(); calib_poll_timer_ = nullptr; }
    if (calib_label_)      { calib_label_->deleteLater(); calib_label_ = nullptr; }
    if (recal_button_)     { recal_button_->deleteLater(); recal_button_ = nullptr; }
    if (diag_log_)         { std::fclose(diag_log_); diag_log_ = nullptr; }
}

// Build (or rebuild) the amber "Calibrating, hold still" banner and the
// 100ms poll timer that reacts to worker-thread state changes. Factored
// out so the Re-calibrate button can re-run it mid-session.
// UI thread only.
void PSVRTracker::install_calibration_banner()
{
    if (!tracker_frame_ || !tracker_frame_->layout())
        return;

    if (recal_button_) {
        recal_button_->hide();
    }

    calib_label_ = new QLabel(tracker_frame_);
    calib_label_->setWordWrap(true);
    calib_label_->setAlignment(Qt::AlignCenter);
    // Initial stage: "waiting for USB". IOHIDManagerOpen + the PSVR's
    // activation burst can take several seconds to deliver the first
    // report, during which no calibration samples are being consumed.
    // Lying "Calibrating (~2 s)" during that window would mislead the
    // user when they see 10+ seconds of "calibration" banner. Once the
    // first sample arrives (samples_flowing_ flips true), the poll
    // timer upgrades this label to the honest amber calibration text.
    calib_label_->setStyleSheet(
        "QLabel { color: #268bd2; font-weight: bold; padding: 6px; "
        "border: 1px solid #268bd2; border-radius: 4px; }");
    calib_label_->setText(QObject::tr(
        "Waiting for PSVR on USB…\n"
        "The headset's processor unit can take 5-15 seconds to reply\n"
        "after Start (especially on first connection). If this banner\n"
        "stays up for more than ~15 s the watchdog will surface a\n"
        "specific error below."));
    tracker_frame_->layout()->addWidget(calib_label_);

    calib_poll_timer_ = new QTimer();
    calib_poll_timer_->setInterval(100);
    QLabel* lbl = calib_label_;
    QTimer* tmr = calib_poll_timer_;
    // Per-lambda flag so we only flip from "waiting" to "calibrating"
    // text once per banner lifecycle. Captured by reference via a
    // shared_ptr because lambdas are copy-constructed by Qt's signal
    // machinery and a plain mutable bool wouldn't be shared.
    auto upgraded = std::make_shared<bool>(false);
    QObject::connect(tmr, &QTimer::timeout, [this, lbl, tmr, upgraded]() {
        // Check the failure path first: if the worker flagged a problem
        // we repaint the banner red, stop polling, and surface the
        // Re-calibrate button so the user can retry without full Stop.
        const int failure = calib_failure_.load(std::memory_order_acquire);
        if (failure != CALIB_FAIL_NONE) {
            if (lbl == calib_label_) {
                QString msg;
                switch (failure) {
                case CALIB_FAIL_NO_DATA:
                    msg = QObject::tr(
                        "No data received from the PSVR.\n"
                        "• Is the USB cable connected?\n"
                        "• Is the headset's in-line power box lit up?\n"
                        "• Is Input Monitoring granted to opentrack in\n"
                        "  System Settings > Privacy & Security?\n"
                        "Fix the issue, then press Re-calibrate below.");
                    break;
                case CALIB_FAIL_NO_GRAVITY:
                    msg = QObject::tr(
                        "The PSVR headset appears to be OFF.\n"
                        "Data is flowing from the USB processor unit, but\n"
                        "the accelerometer is not measuring gravity —\n"
                        "the headset itself is asleep.\n"
                        "Press the power button on the in-line remote\n"
                        "until the screen wakes, then press Re-calibrate.");
                    break;
                case CALIB_FAIL_WORN:
                    msg = QObject::tr(
                        "Can't calibrate while the headset is worn.\n"
                        "Calibration needs the PSVR still on a flat\n"
                        "surface (desk, table) so gyro bias can be\n"
                        "measured without body-motion contamination.\n"
                        "Take it off, put it down, and press\n"
                        "Re-calibrate. You can put it back on once\n"
                        "the amber 'Calibrating' banner clears.");
                    break;
                case CALIB_FAIL_MOVING:
                    msg = QObject::tr(
                        "Calibration rejected — the headset moved.\n"
                        "During the 2-second averaging window, the\n"
                        "gyroscope measured more motion than the noise\n"
                        "floor. If you were holding the PSVR or walking\n"
                        "with it, set it down on a flat still surface\n"
                        "(desk, table) and press Re-calibrate.\n"
                        "If you were not moving it, a bump, knock or\n"
                        "cable tug is usually the cause.");
                    break;
                default:
                    msg = QObject::tr("PSVR calibration failed.");
                    break;
                }
                lbl->setStyleSheet(
                    "QLabel { color: #dc322f; font-weight: bold; padding: 6px; "
                    "border: 1px solid #dc322f; border-radius: 4px; }");
                lbl->setText(msg);
            }
            tmr->stop();
            if (tmr == calib_poll_timer_) {
                tmr->deleteLater();
                calib_poll_timer_ = nullptr;
            }
            show_recalibrate_button();
            return;
        }
        // "Waiting for USB" → "Calibrating" transition. Once the HID
        // report stream actually starts flowing, upgrade the blue
        // waiting banner to the amber calibrating banner so the user's
        // expectation matches what's really happening from here on.
        if (!*upgraded
            && samples_flowing_.load(std::memory_order_acquire)) {
            if (lbl == calib_label_) {
                lbl->setStyleSheet(
                    "QLabel { color: #b58900; font-weight: bold; padding: 6px; "
                    "border: 1px solid #b58900; border-radius: 4px; }");
                lbl->setText(QObject::tr(
                    "Calibrating gyroscope (~3 s)\n"
                    "Keep the PSVR PERFECTLY STILL on a flat surface.\n"
                    "DO NOT put it on your head yet — head motion during\n"
                    "calibration will bake drift into the tracker.\n"
                    "(1 s sensor-stream warmup + 2 s bias averaging.)"));
            }
            *upgraded = true;
        }
        if (!calibrated_.load(std::memory_order_acquire))
            return;
        // Success path: remove the amber banner, surface the button.
        if (lbl == calib_label_) {
            lbl->deleteLater();
            calib_label_ = nullptr;
        }
        tmr->stop();
        if (tmr == calib_poll_timer_) {
            tmr->deleteLater();
            calib_poll_timer_ = nullptr;
        }
        show_recalibrate_button();
    });
    calib_poll_timer_->start();
}

// Install (or re-show) the Re-calibrate button. Clicking asks the HID
// worker to reset its calibration accumulators at the top of the next
// report; we also tear down any current banner and install a fresh one
// so the UI reflects the "re-calibrating" state immediately.
// UI thread only.
void PSVRTracker::show_recalibrate_button()
{
    if (!tracker_frame_ || !tracker_frame_->layout())
        return;
    if (!recal_button_) {
        recal_button_ = new QPushButton(
            QObject::tr("Re-calibrate gyroscope"), tracker_frame_);
        recal_button_->setToolTip(QObject::tr(
            "Rerun the 2-second gyro-bias calibration without stopping "
            "the tracker. Put the PSVR back on a flat still surface first."));
        tracker_frame_->layout()->addWidget(recal_button_);
        QObject::connect(recal_button_, &QPushButton::clicked, [this]() {
            // Tear down the leftover banner/timer from a previous cycle
            // before installing a fresh one, so we don't leak widgets.
            if (calib_poll_timer_) {
                calib_poll_timer_->stop();
                calib_poll_timer_->deleteLater();
                calib_poll_timer_ = nullptr;
            }
            if (calib_label_) {
                calib_label_->deleteLater();
                calib_label_ = nullptr;
            }
            // Ask the HID worker to restart calibration. The worker
            // owns the non-atomic accumulators, so it performs the
            // reset itself at the top of process_group.
            recalibrate_request_.store(true, std::memory_order_release);
            install_calibration_banner();
        });
    }
    recal_button_->show();
}

module_status PSVRTracker::start_tracker(QFrame* frame)
{
    stop_ = false;

    // Remember the frame so Re-calibrate clicks can re-install the UI.
    tracker_frame_ = frame;
    if (frame) {
        if (!frame->layout())
            frame->setLayout(new QVBoxLayout(frame));
        install_calibration_banner();
    }

    // Diagnostic log setup - must happen on this thread before the worker
    // spawns, so the file handle is visible to it via the ordinary
    // happens-before provided by std::thread construction. Column header
    // documents the schema so log files are self-describing.
    if (s_.enable_diag_log) {
        diag_log_ = std::fopen("/tmp/psvr-diag.log", "w");
        if (diag_log_) {
            diag_start_time_ = CFAbsoluteTimeGetCurrent();
            diag_last_log_time_ = 0;
            diag_last_rate_time_ = 0;
            diag_sample_count_ = 0;
            diag_rate_base_count_ = 0;
            diag_last_sample_rate_ = 0;
            std::fprintf(diag_log_,
                "# PSVR tracker diagnostic log. Columns (tab-separated):\n"
                "# 1 abs_time (CFAbsoluteTime, seconds)\n"
                "# 2 uptime_s   (since tracker Start)\n"
                "# 3 phase      (CALIB or RUN)\n"
                "# 4-6 yaw pitch roll     (filtered pose, deg)\n"
                "# 7-9 gyro_y gyro_p gyro_r (post-bias, deg/s; axes match 4-6)\n"
                "# 10-12 bias_0 bias_1 bias_2 (raw-axis gyro bias, deg/s -\n"
                "#          evolves during ZUPT stillness-tracking)\n"
                "# 13-15 acc_0 acc_1 acc_2   (accelerometer, g)\n"
                "# 16 |acc|  (g; should be ~1.00 at rest)\n"
                "# 17-18 acc_pitch acc_roll  (gravity-only pitch/roll, deg)\n"
                "# 19 imu_rate_hz (IMU samples/sec; expected ~200)\n"
                "# 20 cum_imu_samples\n"
                "# 21 still_streak_yaw (consecutive samples where YAW gyro\n"
                "#    is below STILL_MAX_GYRO_DPS; once > STILL_MIN_STREAK\n"
                "#    (200 samples = 100 ms at 2000 Hz), ZUPT actively\n"
                "#    refines the yaw-axis bias toward the raw gyro reading.\n"
                "#    Yaw is the most user-relevant axis since it has no\n"
                "#    gravity reference; pitch and roll have separate per-\n"
                "#    axis streaks not logged here.)\n"
                "# 22 worn     (0/1; proximity sensor says head is present)\n"
                "# 23 disp_on  (0/1; HMD display is active/awake)\n"
                "# 24 fw_cal   (firmware-reported IMU calibration state:\n"
                "#              255=cold boot, 0-3=calibrating, 4=calibrated)\n"
                "# 25 ir       (raw proximity reading; 0=far, 1023=near)\n");
            std::fflush(diag_log_);
        }
    }

    worker_ = std::thread([this]{ worker_loop(); });
    if (s_.enable_mirror)
        psvr_mirror_start();
    return {};
}

void PSVRTracker::data(double* data)
{
    // opentrack data layout: [x y z yaw pitch roll]
    data[0] = 0; data[1] = 0; data[2] = 0;
    data[3] = yaw_.load(std::memory_order_relaxed);
    data[4] = pitch_.load(std::memory_order_relaxed);
    data[5] = roll_.load(std::memory_order_relaxed);
}

void PSVRTracker::process_group(const uint8_t* buf)
{
    // UI-thread request to re-run calibration. Reset all calibration
    // state in this same thread (we own it here, so no lock needed),
    // then fall through to the normal calibrate-or-filter logic below.
    if (recalibrate_request_.exchange(false, std::memory_order_acq_rel)) {
        calib_count_ = 0;
        calib_warmup_count_ = 0;
        calib_peak_gyro_dps_ = 0.0;
        calib_gyro_accum_[0]  = calib_gyro_accum_[1]  = calib_gyro_accum_[2]  = 0;
        calib_accel_accum_[0] = calib_accel_accum_[1] = calib_accel_accum_[2] = 0;
        still_streak_axis_[0] = still_streak_axis_[1] = still_streak_axis_[2] = 0;
        calibrated_.store(false, std::memory_order_release);
        calib_failure_.store(CALIB_FAIL_NONE, std::memory_order_release);
        // Re-calibration is always requested mid-stream (we already
        // have samples flowing), but clear the flag so the banner
        // briefly re-shows "calibrating" rather than "waiting for USB".
        samples_flowing_.store(true, std::memory_order_release);
        // Reset the silent-stream watchdog baseline so a dead stream
        // gets re-reported on the next attempt instead of instantly.
        worker_start_time_ = CFAbsoluteTimeGetCurrent();
        qDebug() << "PSVR: re-calibration requested - resetting bias state";
    }

    // First-sample-ever notification to the UI. exchange is idempotent
    // after the first call so this is just a single cache write per
    // sample after warm-up; the compare-then-branch gets predicted.
    if (!samples_flowing_.load(std::memory_order_relaxed))
        samples_flowing_.store(true, std::memory_order_release);

    int16_t g_raw[3];
    g_raw[0] = read_i16(buf + 4);
    g_raw[1] = read_i16(buf + 6);
    g_raw[2] = read_i16(buf + 8);

    int16_t a_raw[3];
    a_raw[0] = read_i16(buf + 10) >> 4;
    a_raw[1] = read_i16(buf + 12) >> 4;
    a_raw[2] = read_i16(buf + 14) >> 4;

    double g[3], a[3];
    for (int i = 0; i < 3; i++) {
        g[i] = g_raw[i] / GYRO_LSB_PER_DEG;
        a[i] = a_raw[i] / ACCEL_LSB_PER_G;
    }

    // Phase 1: gyro-bias calibration. For the first ~2s of samples we
    // accumulate raw gyro and accel, and publish NO pose (yaw/pitch/roll
    // stay at 0 -> opentrack sees "no motion yet"). When done we store
    // the bias, seed pitch/roll from the averaged accelerometer reading
    // (so the filter doesn't have to converge onto gravity afterwards)
    // and flip the "calibrated" guard.
    if (!calibrated_.load(std::memory_order_acquire)) {
        // If the worker (or a prior accel-magnitude check) already
        // flagged the session as unusable, stop consuming samples.
        // calibrated_ will never flip, so no pose will ever publish.
        if (calib_failure_.load(std::memory_order_relaxed) != CALIB_FAIL_NONE)
            return;
        // Refuse to calibrate while worn: breathing, pulse and body
        // sway all contaminate the bias average if the headset is on
        // someone's head, producing bad drift characteristics that
        // ZUPT has to clean up later. Cheaper to just wait for a
        // flat-surface calibration.
        if (worn_.load(std::memory_order_relaxed)) {
            qWarning() << "PSVR: calibration refused - headset is worn. "
                          "Place it still on a flat surface and press "
                          "Re-calibrate.";
            calib_failure_.store(CALIB_FAIL_WORN,
                                 std::memory_order_release);
            return;
        }
        // Warmup: swallow the first second of post-activation samples
        // without folding them into the averages. See the comment on
        // CALIB_WARMUP_SAMPLES in psvr.h for why these samples are
        // systematically worse than the ones that follow.
        if (calib_warmup_count_ < CALIB_WARMUP_SAMPLES) {
            ++calib_warmup_count_;
            return;
        }
        for (int i = 0; i < 3; i++) {
            calib_gyro_accum_[i]  += g[i];
            calib_accel_accum_[i] += a[i];
            // Track peak raw-gyro magnitude seen in this window.
            // If the user is holding the headset (not letting it
            // sit on a surface), even gentle motion produces raw
            // values well above rest-noise. We check after the
            // window closes and reject the calibration before it
            // can contaminate the bias estimate.
            const double m = std::fabs(g[i]);
            if (m > calib_peak_gyro_dps_)
                calib_peak_gyro_dps_ = m;
        }
        if (++calib_count_ >= CALIB_SAMPLES) {
            // Motion check: reject calibration if the headset was
            // moved during the averaging window. At rest, raw gyro
            // peaks at ~2-3 dps from noise + residual bias; any real
            // motion (hand shake, placement bump) produces ≥5 dps.
            // Abort BEFORE computing bias so the bad samples never
            // get baked in; user can re-try with Re-calibrate.
            if (calib_peak_gyro_dps_ > CALIB_MAX_MOTION_DPS) {
                qWarning().nospace()
                    << "PSVR: calibration REJECTED - headset moved during "
                    << "averaging (peak |gyro|=" << calib_peak_gyro_dps_
                    << " dps > " << CALIB_MAX_MOTION_DPS << " dps). "
                    << "Put it on a flat still surface and press Re-calibrate.";
                if (diag_log_) {
                    const double now = CFAbsoluteTimeGetCurrent();
                    std::fprintf(diag_log_,
                        "%.3f\t%7.2f\tCALIB_FAIL_MOVING"
                        "\t-\t-\t-"
                        "\t-\t-\t-"
                        "\t-\t-\t-"
                        "\t-\t-\t-\t%5.3f"   // |acc| column hijacked to hold peak-gyro
                        "\t-\t-"
                        "\t-\t%d\t-"
                        "\t-\t-\t-\t-\n",
                        now, now - diag_start_time_,
                        calib_peak_gyro_dps_, calib_count_);
                    std::fflush(diag_log_);
                }
                calib_failure_.store(CALIB_FAIL_MOVING,
                                     std::memory_order_release);
                return;
            }
            double avg_a[3];
            for (int i = 0; i < 3; i++) {
                gyro_bias_[i] = calib_gyro_accum_[i] / calib_count_;
                avg_a[i]      = calib_accel_accum_[i] / calib_count_;
            }
            // Gravity sanity check. At this point avg_a should be
            // approximately the local gravity vector (|~1g|) if the
            // HMD was lying still. If it's far from 1g, the headset is
            // either off-but-USB-alive (streaming near-zero accel) or
            // the user was swinging it around — either way, we must
            // NOT bake the gyro offsets we just averaged, since they
            // include whatever garbage the IMU was emitting.
            const double avg_mag = std::sqrt(avg_a[0]*avg_a[0]
                                           + avg_a[1]*avg_a[1]
                                           + avg_a[2]*avg_a[2]);
            if (avg_mag < GRAVITY_MIN_G || avg_mag > GRAVITY_MAX_G) {
                qWarning().nospace()
                    << "PSVR: calibration REJECTED - |avg_accel|=" << avg_mag
                    << "g is not gravity; avg=("
                    << avg_a[0] << ", " << avg_a[1] << ", " << avg_a[2]
                    << "). Headset likely OFF.";
                if (diag_log_) {
                    const double now = CFAbsoluteTimeGetCurrent();
                    std::fprintf(diag_log_,
                        "%.3f\t%7.2f\tCALIB_FAIL_NO_GRAVITY"
                        "\t-\t-\t-"
                        "\t-\t-\t-"
                        "\t-\t-\t-"
                        "\t%+6.3f\t%+6.3f\t%+6.3f\t%5.3f"
                        "\t-\t-"
                        "\t-\t%d\t-"
                        "\t-\t-\t-\t-\n",
                        now, now - diag_start_time_,
                        avg_a[0], avg_a[1], avg_a[2], avg_mag,
                        calib_count_);
                    std::fflush(diag_log_);
                }
                calib_failure_.store(CALIB_FAIL_NO_GRAVITY,
                                     std::memory_order_release);
                return;
            }
            const double seed_pitch = accel_pitch_deg(avg_a);
            const double seed_roll  = accel_roll_deg(avg_a);
            pitch_.store(seed_pitch, std::memory_order_relaxed);
            roll_.store(seed_roll,   std::memory_order_relaxed);
            yaw_.store(0.0,          std::memory_order_relaxed);
            // Record this device's actual rest-state |accel|; the ZUPT
            // stillness detector compares each sample's |accel| to
            // THIS reference instead of a hardcoded 1.0 g, so units-
            // mis-scaled accels don't perpetually fail "still".
            calib_gravity_mag_ = avg_mag;
            calibrated_.store(true, std::memory_order_release);
            qDebug() << "PSVR: calibrated."
                     << "gyro bias (deg/s):" << gyro_bias_[0] << gyro_bias_[1] << gyro_bias_[2]
                     << " seed pitch:" << seed_pitch << " seed roll:" << seed_roll
                     << " local gravity mag:" << calib_gravity_mag_ << "g";
            if (diag_log_) {
                const double now = CFAbsoluteTimeGetCurrent();
                std::fprintf(diag_log_,
                    "%.3f\t%7.2f\tCALIB_DONE"
                    "\t%+8.3f\t%+8.3f\t%+8.3f"
                    "\t-\t-\t-"
                    "\t%+6.3f\t%+6.3f\t%+6.3f"
                    "\t%+6.3f\t%+6.3f\t%+6.3f\t-"
                    "\t%+7.2f\t%+7.2f"
                    "\t-\t%d\t-"
                    "\t-\t-\t-\t-\n",
                    now, now - diag_start_time_,
                    0.0, seed_pitch, seed_roll,
                    gyro_bias_[0], gyro_bias_[1], gyro_bias_[2],
                    avg_a[0], avg_a[1], avg_a[2],
                    seed_pitch, seed_roll,
                    calib_count_);
                std::fflush(diag_log_);
            }
        }
        return;
    }

    // Phase 2: normal operation. Subtract the bias computed above from
    // every gyro sample before integrating. Keep the raw (pre-bias)
    // reading around so the ZUPT tracker below can low-pass the bias
    // toward it during stillness.
    const double g_raw_dps[3] = { g[0], g[1], g[2] };
    for (int i = 0; i < 3; i++)
        g[i] -= gyro_bias_[i];

    // ZUPT (zero-velocity update): while the headset sits still we
    // know its true angular velocity is zero, so any nonzero gyro
    // reading IS the current bias. Drag gyro_bias_ toward the raw
    // reading with a slow EMA so thermal drift of the bias baked in
    // at Start gets corrected during natural pauses in head motion.
    // We require a short "already-still" streak before updating so a
    // one-sample dip in gyro/accel doesn't start tracking mid-motion.
    // Per-axis ZUPT: each gyro axis has its own stillness streak and
    // its own bias update. So a fast yaw turn breaks ONLY the yaw-axis
    // streak; pitch and roll bias keep refining throughout because
    // those axes are not rotating during a yaw turn (and similarly for
    // pitch/roll motions). The yaw-axis update is the user-visible win
    // because yaw has no gravity reference and integrates pure bias
    // forever; the pitch and roll updates marginally improve transient
    // response during fast head motion. No accel-quiet check: gyros
    // measure angular velocity, not linear, so linear motion of the
    // headset doesn't invalidate the bias estimate.
    for (int i = 0; i < 3; i++) {
        if (std::fabs(g[i]) < STILL_MAX_GYRO_DPS) {
            ++still_streak_axis_[i];
            if (still_streak_axis_[i] > STILL_MIN_STREAK) {
                gyro_bias_[i] +=
                    ZUPT_BIAS_ALPHA * (g_raw_dps[i] - gyro_bias_[i]);
            }
        } else {
            still_streak_axis_[i] = 0;
        }
    }

    const double gy = YAW_SIGN   * g[YAW_SRC];
    const double gp = PITCH_SIGN * g[PITCH_SRC];
    const double gr = ROLL_SIGN  * g[ROLL_SRC];

    const double acc_pitch = accel_pitch_deg(a);
    const double acc_roll  = accel_roll_deg(a);

    // Per-sample dt for gyro integration. Bounded so a transient stream
    // stall (one big gap) can't inject a giant integration step that
    // sends pose flying. The first sample after Start uses a sane
    // fallback because last_sample_time_ is 0; subsequent samples use
    // the real measured interval.
    const double now_ts = CFAbsoluteTimeGetCurrent();
    double dt;
    if (last_sample_time_ <= 0.0) {
        dt = DT_FALLBACK;
    } else {
        dt = now_ts - last_sample_time_;
        if (dt < DT_MIN) dt = DT_MIN;
        if (dt > DT_MAX) dt = DT_MAX;
    }
    last_sample_time_ = now_ts;

    double yaw   = yaw_.load(std::memory_order_relaxed)   + gy * dt;
    double pitch = ALPHA * (pitch_.load(std::memory_order_relaxed) + gp * dt) + (1.0 - ALPHA) * acc_pitch;
    double roll  = ALPHA * (roll_.load(std::memory_order_relaxed)  + gr * dt) + (1.0 - ALPHA) * acc_roll;

    if (yaw >  180.0) yaw -= 360.0;
    if (yaw < -180.0) yaw += 360.0;

    yaw_.store(yaw,     std::memory_order_relaxed);
    pitch_.store(pitch, std::memory_order_relaxed);
    roll_.store(roll,   std::memory_order_relaxed);

    // Optional diagnostic log. One row per second. `g[]` here is
    // post-bias (deg/s), `a[]` is accel in g. We recompute acc-only
    // pitch/roll so the row captures a gravity-only reference next
    // to the filtered pose - useful for seeing filter divergence.
    if (diag_log_) {
        ++diag_sample_count_;
        const double now = CFAbsoluteTimeGetCurrent();
        if (now - diag_last_rate_time_ >= 1.0) {
            const double dt = now - diag_last_rate_time_;
            if (diag_last_rate_time_ > 0.0 && dt > 0.0) {
                diag_last_sample_rate_ =
                    double(diag_sample_count_ - diag_rate_base_count_) / dt;
            }
            diag_last_rate_time_ = now;
            diag_rate_base_count_ = diag_sample_count_;
        }
        if (now - diag_last_log_time_ >= 1.0) {
            diag_last_log_time_ = now;
            const double acc_mag =
                std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
            std::fprintf(diag_log_,
                "%.3f\t%7.2f\tRUN\t%+8.3f\t%+8.3f\t%+8.3f"
                "\t%+7.3f\t%+7.3f\t%+7.3f"
                "\t%+6.3f\t%+6.3f\t%+6.3f"
                "\t%+6.3f\t%+6.3f\t%+6.3f\t%5.3f"
                "\t%+7.2f\t%+7.2f"
                "\t%.1f\t%llu\t%d"
                "\t%d\t%d\t%u\t%u\n",
                now, now - diag_start_time_,
                yaw, pitch, roll,
                g[YAW_SRC], g[PITCH_SRC], g[ROLL_SRC],
                gyro_bias_[0], gyro_bias_[1], gyro_bias_[2],
                a[0], a[1], a[2], acc_mag,
                acc_pitch, acc_roll,
                diag_last_sample_rate_,
                (unsigned long long)diag_sample_count_,
                still_streak_axis_[YAW_SRC],
                worn_.load(std::memory_order_relaxed) ? 1 : 0,
                display_active_.load(std::memory_order_relaxed) ? 1 : 0,
                (unsigned)fw_cal_status_.load(std::memory_order_relaxed),
                (unsigned)ir_sensor_.load(std::memory_order_relaxed));
            std::fflush(diag_log_);
        }
    }
}

void PSVRTracker::report_cb(void* context, IOReturn, void*,
                            IOHIDReportType, uint32_t,
                            uint8_t* report, CFIndex reportLength)
{
    if (reportLength < 64 || !context) return;
    auto* self = static_cast<PSVRTracker*>(context);
    // Stamp BEFORE processing so on-stall logic in worker_loop has the
    // freshest possible "we heard from the headset" timestamp.
    self->last_report_time_ = CFAbsoluteTimeGetCurrent();

    // Byte 8 = headset-state flags. Field layout documented in
    // psvr.h; semantics adopted from gusmanb/PSVRFramework's
    // sensor-report parser. Logged to diag and, crucially, used
    // to refuse calibration while the headset is being worn.
    const uint8_t flags = report[8];
    const bool was_worn = self->worn_.exchange(
        (flags & 0x01) != 0, std::memory_order_acq_rel);
    const bool is_worn  = self->worn_.load(std::memory_order_relaxed);
    self->display_active_.store(
        (flags & 0x02) == 0, std::memory_order_relaxed);
    self->fw_cal_status_.store(report[48], std::memory_order_relaxed);
    // 10-bit IR proximity reading spans bytes 55-56 (big-endian).
    self->ir_sensor_.store(
        uint16_t(report[55]) | (uint16_t(report[56]) << 8),
        std::memory_order_relaxed);

    // Log worn transitions - useful for tracing "why did tracking
    // stop" in user reports (headset slid off = worn→false, often
    // followed ~8 minutes later by the PSVR's own auto-sleep).
    if (was_worn != is_worn) {
        qDebug() << "PSVR: worn" << (is_worn ? "TRUE" : "FALSE")
                 << "(IR="
                 << self->ir_sensor_.load(std::memory_order_relaxed)
                 << ")";
    }

    self->process_group(report + 16);
    self->process_group(report + 32);
}

void PSVRTracker::send_activation(IOHIDDeviceRef device)
{
    // Output-report write helper. Logs non-success returns because a
    // failed activation is a legitimate diagnostic clue ("headset went
    // to sleep", "USB stalled", "permission revoked") — silently dropping
    // the return value hides real problems.
    auto send_cmd = [device](uint8_t cmd, const uint8_t* payload, size_t len) {
        uint8_t report[64]{};
        report[0] = cmd;
        report[1] = 0x00;
        report[2] = 0xAA;
        report[3] = static_cast<uint8_t>(len);
        if (payload && len > 0) std::memcpy(report + 4, payload, len);
        IOReturn r = IOHIDDeviceSetReport(device, kIOHIDReportTypeOutput,
                                          cmd, report, 4 + len);
        if (r != kIOReturnSuccess) {
            qWarning().nospace()
                << "PSVR: activation cmd 0x" << Qt::hex << cmd
                << " failed, IOReturn=0x" << r;
        }
    };

    // Full activation sequence, cross-checked against gusmanb/PSVRFramework
    // (PSVR.cs) and psmoveservice's MorpheusHMD driver:
    //
    //   0x17 SetHeadsetPower(ON)       — wakes the HMD display + enables
    //                                    the sensor chain
    //   0x11 EnableTracking(0xFFFFFF00) — the "magic" payload that turns
    //                                    on full IMU tracking AND the
    //                                    9 blue beacon LEDs on the front
    //                                    of the headset. A payload of
    //                                    {0x01,0,0,0} partially works
    //                                    but gives degraded tracking.
    //   0x23 SetVRMode(ON)             — switches the display into
    //                                    1920×1080 split-screen layout
    //                                    (960+960 per eye), which is
    //                                    what the SBS mirror expects.
    //                                    The HMD remembers its last
    //                                    mode across power cycles, so
    //                                    this is only observably useful
    //                                    when coming from a prior
    //                                    cinematic-mode session.
    const uint8_t set_headset_on[4] = {0x01, 0x00, 0x00, 0x00};
    send_cmd(0x17, set_headset_on, sizeof(set_headset_on));

    // 0xFFFFFF00 (little-endian) + 4 trailing zero bytes, exactly as
    // per PSVRFramework's GetEnableVRTracking() and PSMoveService's
    // morpheus_enable_tracking().
    const uint8_t enable_tracking[8] = {
        0x00, 0xFF, 0xFF, 0xFF,
        0x00, 0x00, 0x00, 0x00
    };
    send_cmd(0x11, enable_tracking, sizeof(enable_tracking));

    const uint8_t set_vr_mode[4] = {0x01, 0x00, 0x00, 0x00};
    send_cmd(0x23, set_vr_mode, sizeof(set_vr_mode));
}

void PSVRTracker::send_activation_to_all()
{
    std::lock_guard<std::mutex> lk(devices_mu_);
    for (auto& md : devices_)
        send_activation(md.device);
}

void PSVRTracker::device_matched_cb(void* context, IOReturn, void*, IOHIDDeviceRef device)
{
    if (!context || !device) return;
    auto* self = static_cast<PSVRTracker*>(context);

    // Allocate this device's own report buffer BEFORE registering the
    // callback; the pointer we hand to IOKit must remain valid for the
    // lifetime of the registration.
    auto buf = std::make_unique<std::array<uint8_t, 256>>();
    uint8_t* buf_ptr = buf->data();
    {
        std::lock_guard<std::mutex> lk(self->devices_mu_);
        CFRetain(device);
        self->devices_.push_back({device, std::move(buf)});
    }

    self->send_activation(device);

    IOHIDDeviceRegisterInputReportCallback(device, buf_ptr, 256,
                                           report_cb, context);
}

void PSVRTracker::device_removed_cb(void* context, IOReturn, void*, IOHIDDeviceRef device)
{
    if (!context || !device) return;
    auto* self = static_cast<PSVRTracker*>(context);
    std::lock_guard<std::mutex> lk(self->devices_mu_);
    for (auto it = self->devices_.begin(); it != self->devices_.end(); ) {
        if (it->device == device) {
            CFRelease(it->device);
            it = self->devices_.erase(it);
        } else {
            ++it;
        }
    }
}

void PSVRTracker::worker_loop()
{
    runloop_.store(CFRunLoopGetCurrent(), std::memory_order_release);

    IOHIDManagerRef mgr = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone);
    if (!mgr) {
        qWarning() << "PSVR: IOHIDManagerCreate failed";
        return;
    }

    CFMutableDictionaryRef match = CFDictionaryCreateMutable(
        kCFAllocatorDefault, 2,
        &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    int vid = PSVR_VID, pid = PSVR_PID;
    CFNumberRef cf_vid = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &vid);
    CFNumberRef cf_pid = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &pid);
    CFDictionarySetValue(match, CFSTR(kIOHIDVendorIDKey),  cf_vid);
    CFDictionarySetValue(match, CFSTR(kIOHIDProductIDKey), cf_pid);
    CFRelease(cf_vid);
    CFRelease(cf_pid);

    IOHIDManagerSetDeviceMatching(mgr, match);
    CFRelease(match);

    IOHIDManagerRegisterDeviceMatchingCallback(mgr, device_matched_cb, this);
    IOHIDManagerRegisterDeviceRemovalCallback(mgr, device_removed_cb, this);
    IOHIDManagerScheduleWithRunLoop(
        mgr, runloop_.load(std::memory_order_relaxed), kCFRunLoopDefaultMode);

    IOReturn ret = IOHIDManagerOpen(mgr, kIOHIDOptionsTypeNone);
    if (ret != kIOReturnSuccess) {
        qWarning() << "PSVR: IOHIDManagerOpen failed (0x"
                   << QString::number(ret, 16)
                   << ") - grant Input Monitoring permission to opentrack in System Settings";
    }

    // Keepalive strategy: on-stall recovery rather than a blind timer.
    //
    // The old periodic-keepalive design fired the activation burst
    // (0x17 SetHeadsetPower + 0x11 EnableTracking) every 10 s, but the
    // burst itself interrupts the IMU stream for ~1-2 s every time it's
    // sent. That produced visible "freeze, recover" stalls on every
    // 10 s boundary. Reviewing PSMoveService's Morpheus driver and
    // dylanmckay/psvr-protocol confirms there is no documented "light"
    // keepalive that doesn't reboot the sensor chain — 0x17, 0x11, and
    // 0x23 all stall the stream.
    //
    // Instead, we watch `last_report_time_` (stamped in report_cb). If
    // it stops advancing for STALL_RECOVERY_SEC while calibration is
    // complete, the PSVR has likely auto-slept (the headset's ~8 min
    // inactivity sleep is the common trigger). We then fire the
    // activation burst to wake it. The user pays the ~1-2 s recovery
    // stall only at the moment of wake-up, which is free — the stream
    // was silent for at least 5 s before we noticed.

    worker_start_time_ = CFAbsoluteTimeGetCurrent();

    while (!stop_) {
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.1, true);

        // Silent-stream detection. If no HID reports ever arrive in the
        // first NO_DATA_TIMEOUT_SEC of Start, either the USB cable is
        // unplugged, the processor box is dead, or Input Monitoring
        // permission was refused. calib_count_ is safe to read here
        // without synchronization: it's written only from report_cb,
        // which dispatches on this very CFRunLoop thread.
        if (calib_failure_.load(std::memory_order_relaxed) == CALIB_FAIL_NONE
            && !calibrated_.load(std::memory_order_relaxed)
            && calib_count_ == 0
            && (CFAbsoluteTimeGetCurrent() - worker_start_time_) > NO_DATA_TIMEOUT_SEC)
        {
            qWarning() << "PSVR: no HID reports within"
                       << NO_DATA_TIMEOUT_SEC
                       << "s of Start - headset off / unplugged / permissions missing";
            calib_failure_.store(CALIB_FAIL_NO_DATA,
                                 std::memory_order_release);
        }

        // Post-calibration stall-recovery keepalive. Only arms once
        // calibration has completed AND at least one report has been
        // seen (last_report_time_ > 0), so fresh-start USB latency
        // doesn't race this path. If the HID stream has been silent
        // for STALL_RECOVERY_SEC, re-fire the activation burst; that
        // wakes the PSVR from auto-sleep. Update last_report_time_
        // to "now" so we don't re-fire every tick while waiting for
        // the stream to come back; the next real report will overwrite
        // it.
        if (calibrated_.load(std::memory_order_relaxed)
            && last_report_time_ > 0.0)
        {
            const double idle = CFAbsoluteTimeGetCurrent() - last_report_time_;
            if (idle > STALL_RECOVERY_SEC) {
                qWarning().nospace()
                    << "PSVR: HID stream silent for " << idle
                    << "s - firing activation burst to wake the headset";
                send_activation_to_all();
                last_report_time_ = CFAbsoluteTimeGetCurrent();
            }
        }
    }

    CFRunLoopRef rl = runloop_.load(std::memory_order_acquire);
    IOHIDManagerUnscheduleFromRunLoop(mgr, rl, kCFRunLoopDefaultMode);
    IOHIDManagerClose(mgr, kIOHIDOptionsTypeNone);
    CFRelease(mgr);

    {
        std::lock_guard<std::mutex> lk(devices_mu_);
        for (auto& md : devices_) CFRelease(md.device);
        devices_.clear();
    }

    runloop_.store(nullptr, std::memory_order_release);
}

PSVRDialog::PSVRDialog()
{
    setWindowTitle(QObject::tr("PSVR tracker settings"));

    auto* layout = new QVBoxLayout(this);

    auto* header = new QLabel(QObject::tr(
        "PlayStation VR IMU tracker for macOS.\n\n"
        "Reads the PSVR headset's gyroscope and accelerometer over USB HID "
        "and provides yaw/pitch/roll to opentrack. Requires Input Monitoring "
        "permission for opentrack in System Settings.\n\n"
        "Calibration: after USB comes up (up to ~15 s on first start), "
        "the tracker waits one second for the sensor stream to stabilize, "
        "then spends two seconds measuring the gyro's resting bias. "
        "Keep the PSVR still on a flat surface during the whole window — "
        "not on your head — or accumulated head motion will become "
        "baked-in drift. Press \"Re-calibrate\" later to redo it."));
    header->setWordWrap(true);
    layout->addWidget(header);

    mirror_box_ = new QCheckBox(QObject::tr(
        "Mirror the main display side-by-side onto the PSVR screen "
        "(requires Screen Recording permission)"));
    mirror_box_->setChecked(s_.enable_mirror);
    layout->addWidget(mirror_box_);

    diag_log_box_ = new QCheckBox(QObject::tr(
        "Write diagnostic log to /tmp/psvr-diag.log "
        "(pose, gyro, accel, bias, sample rate; one row per second)"));
    diag_log_box_->setChecked(s_.enable_diag_log);
    layout->addWidget(diag_log_box_);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);
    QObject::connect(buttons, &QDialogButtonBox::accepted, this, [this]() {
        s_.enable_mirror   = mirror_box_->isChecked();
        s_.enable_diag_log = diag_log_box_->isChecked();
        s_.b->save();
        accept();
    });
    QObject::connect(buttons, &QDialogButtonBox::rejected, this, [this]() {
        s_.b->reload();
        reject();
    });
}

OPENTRACK_DECLARE_TRACKER(PSVRTracker, PSVRDialog, PSVRMetadata)
