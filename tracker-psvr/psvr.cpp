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

#ifdef PSVR_HAS_CAMERA
// Pulls in the full Worker type so the unique_ptr<Worker> members in
// psvr.h are destructible here and we can call start/stop/diag. Not
// included from psvr.h to keep AVFoundation/OpenCV out of every file
// that transitively includes the tracker header (only psvr.cpp needs
// them).
#include "psvr_camera.h"
#endif

#include <cmath>
#include <cstring>
#include <QDebug>
#include <QCheckBox>
#include <QDialogButtonBox>
#include <QImage>
#include <QLabel>
#include <QPixmap>
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
// dt is measured per-sample from the device's own 24-bit microsecond
// timestamp; see the dt-computation block in process_group.
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
    // Send cinematic-mode-on before we tear down the HID devices so
    // the PSVR drops out of 960×2 VR split-screen mode on shutdown.
    // Quick courtesy to the user: it means after Stop, the headset is
    // a normal display again rather than stuck showing a distorted
    // stereo layout. Borrowed from OpenHMD's close_device() pattern.
    send_cinematic_mode_to_all();

    stop_ = true;
    if (CFRunLoopRef rl = runloop_.load(std::memory_order_acquire))
        CFRunLoopStop(rl);
    if (worker_.joinable())
        worker_.join();
    psvr_mirror_stop();
#ifdef PSVR_HAS_CAMERA
    // Drop the camera worker BEFORE the UI teardown below so its
    // dispatch queue stops delivering frames while the plugin's Qt
    // widgets are still alive (the delegate never touches widgets,
    // but this also guarantees no late-arriving diag-log writes
    // after diag_log_ is closed).
    if (camera_worker_) {
        camera_worker_->stop();
        camera_worker_.reset();
    }
    // Tear down the preview. Stop the timer first so no more
    // fetch_preview calls race with the Worker destruction above,
    // then deleteLater both children of tracker_frame_ (the label and
    // the timer) so Qt's parent-chain clean-up on the next event-loop
    // spin removes them without blocking here. If opentrack already
    // destroyed tracker_frame_ first, Qt will have already taken them
    // along; guard with a null check.
    if (camera_preview_timer_) {
        camera_preview_timer_->stop();
        camera_preview_timer_->deleteLater();
        camera_preview_timer_ = nullptr;
    }
    if (camera_preview_label_) {
        camera_preview_label_->hide();
        camera_preview_label_->deleteLater();
        camera_preview_label_ = nullptr;
    }
#endif
    // deleteLater is safe even if these widgets already went away with
    // their parent frame; it's a no-op on nullptr.
    if (calib_poll_timer_) { calib_poll_timer_->stop(); calib_poll_timer_->deleteLater(); calib_poll_timer_ = nullptr; }
    if (calib_label_)      { calib_label_->deleteLater(); calib_label_ = nullptr; }
    if (recal_button_)     { recal_button_->deleteLater(); recal_button_ = nullptr; }
    if (keepalive_timer_)  { keepalive_timer_->stop(); keepalive_timer_->deleteLater(); keepalive_timer_ = nullptr; }
    if (diag_log_)         { std::fclose(diag_log_); diag_log_ = nullptr; }
}

// Push a status message to the calibration QLabel and (when the
// camera preview is active) also to the camera-worker overlay. When
// the camera is on we additionally hide the QLabel so the same
// message doesn't appear both stacked above and over the video.
// css_color_hex selects the QLabel border/text color ("#268bd2",
// "#b58900", "#dc322f"); r/g/b are the matching color for the
// over-video text. UI thread only.
void PSVRTracker::publish_status_banner(QLabel* lbl, const QString& text,
                                        const char* css_color_hex,
                                        int r, int g, int b)
{
    if (lbl) {
        char css[256];
        std::snprintf(css, sizeof css,
            "QLabel { color: %s; font-weight: bold; padding: 6px; "
            "border: 1px solid %s; border-radius: 4px; }",
            css_color_hex, css_color_hex);
        lbl->setStyleSheet(css);
        lbl->setText(text);
    }
#ifdef PSVR_HAS_CAMERA
    if (camera_worker_ && camera_worker_->is_running()) {
        if (lbl) lbl->hide();
        camera_worker_->set_status_banner(text.toStdString(), r, g, b);
    }
#endif
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
    tracker_frame_->layout()->addWidget(calib_label_);
    // Initial stage: "waiting for USB". IOHIDManagerOpen + the PSVR's
    // activation burst can take several seconds to deliver the first
    // report, during which no calibration samples are being consumed.
    // Lying "Calibrating (~2 s)" during that window would mislead the
    // user when they see 10+ seconds of "calibration" banner. Once the
    // first sample arrives (samples_flowing_ flips true), the poll
    // timer upgrades this label to the honest amber calibration text.
    // Solarized blue 0x268bd2 = (38, 139, 210).
    publish_status_banner(calib_label_, QObject::tr(
        "Waiting for PSVR on USB… cold-boot activation can take up to "
        "20 seconds. If the stream doesn't start after ~12 s the driver "
        "will retry activation once automatically; an error banner "
        "appears only if nothing arrives by ~25 s."),
        "#268bd2", 38, 139, 210);

    // Parent the timer to tracker_frame_ so Qt's parent-chain
    // teardown destroys it automatically if opentrack tears down the
    // frame before our destructor runs (e.g. user clicks Stop while
    // the lambda is in flight). Without a parent, the QTimer outlives
    // tracker_frame_ and its lambda fires with a dangling lbl pointer.
    calib_poll_timer_ = new QTimer(tracker_frame_);
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
                        "• Is Input Monitoring granted to opentrack in "
                        "System Settings > Privacy & Security?\n"
                        "Fix the issue, then press Re-calibrate below.");
                    break;
                case CALIB_FAIL_NO_GRAVITY:
                    msg = QObject::tr(
                        "The PSVR headset appears to be OFF.\n"
                        "Data is flowing from the USB processor unit, "
                        "but the accelerometer is not measuring gravity "
                        "— the headset itself is asleep.\n"
                        "Press the power button on the in-line remote "
                        "until the screen wakes, then press Re-calibrate.");
                    break;
                case CALIB_FAIL_WORN:
                    msg = QObject::tr(
                        "Can't calibrate while the headset is worn.\n"
                        "Calibration needs the PSVR still on a flat "
                        "surface (desk, table) so gyro bias can be "
                        "measured without body-motion contamination.\n"
                        "Take it off, put it down, and press Re-calibrate. "
                        "You can put it back on once the amber "
                        "'Calibrating' banner clears.");
                    break;
                case CALIB_FAIL_MOVING:
                    msg = QObject::tr(
                        "Calibration rejected — the headset moved.\n"
                        "During the 2-second averaging window, the "
                        "gyroscope measured more motion than the noise "
                        "floor. If you were holding the PSVR or walking "
                        "with it, set it down on a flat still surface "
                        "(desk, table) and press Re-calibrate.\n"
                        "If you were not moving it, a bump, knock or "
                        "cable tug is usually the cause.");
                    break;
                default:
                    msg = QObject::tr("PSVR calibration failed.");
                    break;
                }
                // Solarized red 0xdc322f = (220, 50, 47).
                publish_status_banner(lbl, msg, "#dc322f", 220, 50, 47);
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
                // Solarized amber 0xb58900 = (181, 137, 0).
                publish_status_banner(lbl, QObject::tr(
                    "Calibrating gyroscope (~3 s)\n"
                    "Keep the PSVR PERFECTLY STILL on a flat surface. "
                    "DO NOT put it on your head yet — head motion during "
                    "calibration will bake drift into the tracker.\n"
                    "(1 s sensor-stream warmup + 2 s bias averaging.)"),
                    "#b58900", 181, 137, 0);
            }
            *upgraded = true;
        }
        if (!calibrated_.load(std::memory_order_acquire))
            return;
        // Success path: remove the amber banner, surface the button,
        // and clear the camera-overlay banner so the live preview is
        // unobstructed for actual tracking.
        if (lbl == calib_label_) {
            lbl->deleteLater();
            calib_label_ = nullptr;
        }
#ifdef PSVR_HAS_CAMERA
        if (camera_worker_)
            camera_worker_->set_status_banner("", 255, 255, 255);
#endif
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
        // Insert ABOVE the calibration banner in the tracker frame's
        // layout. The banner grows to 6-7 lines on error states and
        // can overflow a small tracker panel; keeping the button at
        // the TOP means it's visible regardless of banner height, so
        // users always have the recovery action within reach.
        auto* vbox = qobject_cast<QVBoxLayout*>(tracker_frame_->layout());
        if (vbox)
            vbox->insertWidget(0, recal_button_);
        else
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
    }

    // Spin up the camera worker FIRST (before installing the
    // calibration banner) when camera mode is on. install_calibration_
    // _banner's first publish_status_banner call needs to see a live
    // camera_worker_ so it can route the "Waiting for PSVR" text to
    // the video overlay instead of the QLabel above it. Otherwise the
    // initial waiting text gets stuck in the QLabel until the next
    // state change re-publishes (which can take 25 s if the headset
    // isn't responding) and the user sees a banner stacked above the
    // preview for the whole startup window. Order matters here.
#ifdef PSVR_HAS_CAMERA
    if (s_.enable_camera) {
        camera_worker_ = std::make_unique<psvr_cam::Worker>();
        if (!camera_worker_->start()) {
            qDebug() << "[psvr] camera worker failed to start; position will be zero";
            camera_worker_.reset();
        }
    }
#endif

    if (frame)
        install_calibration_banner();

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
                "# 25 ir       (raw proximity reading; 0=far, 1023=near)\n"
                "# 26 cam_frames       (cumulative camera frames captured;\n"
                "#                       0 if camera disabled or failed to start)\n"
                "# 27 cam_frames_blob  (cumulative frames where >=1 blob\n"
                "#                       passed the extractor thresholds)\n"
                "# 28 cam_frames_pnp   (cumulative frames where solvePnP\n"
                "#                       accepted the LED identification)\n"
                "# 29 cam_last_blobs   (blob count on most recent frame)\n"
                "# 30 cam_last_matched (blobs matched to LEDs on last frame)\n"
                "# 31 cam_last_ok      (0/1; PnP succeeded last frame)\n"
                "# 32-34 cam_x cam_y cam_z (last accepted head position, cm;\n"
                "#                       camera frame: +X right, +Y up,\n"
                "#                       -Z forward)\n");
            std::fflush(diag_log_);
        }
    }

    worker_ = std::thread([this]{ worker_loop(); });
    if (s_.enable_mirror)
        psvr_mirror_start();

#ifdef PSVR_HAS_CAMERA
    // The camera worker itself was started above (before
    // install_calibration_banner) so the initial banner could route
    // to its overlay. Here we attach the preview QLabel that displays
    // its annotated frames inside tracker_frame_, the same spot
    // neuralnet / easy / pt put their previews. Widget and timer are
    // children of tracker_frame_ so opentrack's frame teardown
    // cleans them up automatically.
    if (camera_worker_ && camera_worker_->is_running() && tracker_frame_) {
        camera_preview_label_ = new QLabel(tracker_frame_);
        camera_preview_label_->setAlignment(Qt::AlignCenter);
        camera_preview_label_->setMinimumSize(320, 180);
        camera_preview_label_->setScaledContents(true);
        camera_preview_label_->setSizePolicy(
            QSizePolicy::Expanding, QSizePolicy::Expanding);
        if (auto* lay = tracker_frame_->layout())
            lay->addWidget(camera_preview_label_);

        camera_preview_timer_ = new QTimer(tracker_frame_);
        camera_preview_timer_->setInterval(33);  // ~30 Hz
        // Qt 6 on macOS throttles (sometimes entirely stops) CoarseTimer
        // callbacks when the app is backgrounded, then fails to catch up
        // cleanly on foreground - the preview would freeze on an old
        // frame until the user clicks inside the window. PreciseTimer
        // uses a higher-priority dispatch source that keeps firing
        // predictably across focus changes, trading a few extra
        // microseconds of scheduling overhead for a reliable preview.
        camera_preview_timer_->setTimerType(Qt::PreciseTimer);
        QObject::connect(camera_preview_timer_, &QTimer::timeout,
            [this]() {
                if (!camera_worker_ || !camera_preview_label_) return;
                int w = 0, h = 0;
                if (!camera_worker_->fetch_preview_rgb(&camera_preview_buf_, &w, &h))
                    return;
                if (w <= 0 || h <= 0) return;
                // QImage wraps the buffer without copying; QPixmap::fromImage
                // then copies pixels into the pixmap, so we're free to
                // mutate camera_preview_buf_ on the next tick.
                QImage img(camera_preview_buf_.data(), w, h,
                           3 * w, QImage::Format_RGB888);
                camera_preview_label_->setPixmap(QPixmap::fromImage(img));
                // Belt-and-suspenders: setPixmap already schedules an
                // update(), but on macOS after a background->foreground
                // cycle Qt's paint-dirty tracking can get confused and
                // the freshly-set pixmap sits on a "clean" widget that
                // never actually paints. Forcing update() here
                // guarantees the next paint cycle delivers our frame.
                camera_preview_label_->update();
            });
        camera_preview_timer_->start();
    }
#endif

    // EXPERIMENTAL keepalive: spin up a periodic HID-command sender
    // aimed at defeating the PSVR's 8-minute auto-sleep. Off by
    // default; user opts in via [psvr-tracker] keepalive-enable=true
    // in the ini. The byte sent and the interval are also ini-tweakable
    // so we can iterate over the unexplored HID command space without
    // rebuilding. Each fire is logged to the diag log so we can
    // correlate with whether the headset stayed awake.
    if (s_.keepalive_enable && tracker_frame_) {
        const int interval_ms = std::max(5000, (int)s_.keepalive_interval_s * 1000);
        // Parse keepalive-cmd with auto-base detection: accepts
        // "0x1F" (hex), "31" (decimal), "0o37" (octal). Bad input
        // falls through to 0x1F default - safer than zero, which
        // would silently send a no-op every minute.
        bool ok = false;
        const int parsed = static_cast<QString>(s_.keepalive_cmd).toInt(&ok, 0);
        const uint8_t cmd = ok ? (uint8_t)(parsed & 0xff) : 0x1F;
        if (!ok) {
            qWarning().nospace()
                << "PSVR: keepalive-cmd '" << static_cast<QString>(s_.keepalive_cmd)
                << "' didn't parse as int; falling back to 0x1F";
        }
        keepalive_timer_ = new QTimer(tracker_frame_);
        keepalive_timer_->setInterval(interval_ms);
        keepalive_timer_->setTimerType(Qt::CoarseTimer);  // sub-second
                                                          // precision unnecessary
        QObject::connect(keepalive_timer_, &QTimer::timeout, [this, cmd]() {
            send_raw_to_all(cmd, nullptr, 0);
            qDebug().nospace() << "PSVR: keepalive HID cmd 0x"
                               << QString::number(cmd, 16) << " sent";
            if (diag_log_) {
                const double now = CFAbsoluteTimeGetCurrent();
                std::fprintf(diag_log_,
                    "%.3f\t%7.2f\tKEEPALIVE\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-"
                    "\t-\t-\t-\t-\t-\t-\t-\t-\t0x%02x\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\n",
                    now, now - diag_start_time_, (unsigned)cmd);
                std::fflush(diag_log_);
            }
        });
        keepalive_timer_->start();
        qDebug().nospace() << "PSVR: keepalive enabled - cmd 0x"
                           << QString::number(cmd, 16)
                           << " every " << (interval_ms / 1000) << "s";
    }
    return {};
}

void PSVRTracker::data(double* data)
{
    // opentrack data layout: [x y z yaw pitch roll].
    //
    // Rotation always comes from the IMU. Position comes from the
    // camera constellation worker when it has a fresh PnP solution;
    // when it doesn't (camera disabled, no LEDs in frame, PnP failed),
    // we emit zeros so opentrack's Center command lands at the
    // calibrated origin and no spurious translations get injected.
    //
    // Fusion trackers can still run this plugin as the rotation source
    // and a different tracker as the position source; in that case the
    // Fusion plugin strips our zeros and uses the camera tracker's
    // output. The two paths are orthogonal - enabling camera tracking
    // here doesn't affect the Fusion case.
    double x = 0, y = 0, z = 0;
    bool have_pos = false;
#ifdef PSVR_HAS_CAMERA
    if (camera_worker_ && camera_worker_->is_running()) {
        have_pos = camera_worker_->get_position(&x, &y, &z);
    }
#endif
    camera_position_fresh_.store(have_pos, std::memory_order_relaxed);
    if (have_pos) {
        head_x_.store(x, std::memory_order_relaxed);
        head_y_.store(y, std::memory_order_relaxed);
        head_z_.store(z, std::memory_order_relaxed);
    } else {
        x = y = z = 0;
    }
    data[0] = x; data[1] = y; data[2] = z;
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
        // Replenish auto-retry budget: the user explicitly asked for a
        // fresh attempt, so we want the same forgiving behavior a cold
        // Start has. (And if they ARE holding the headset, the auto-
        // retries will each fail in ~3 s and then surface the banner.)
        calib_auto_retries_left_ = CALIB_AUTO_RETRIES;
        calibrated_.store(false, std::memory_order_release);
        calib_failure_.store(CALIB_FAIL_NONE, std::memory_order_release);
        // Re-calibration is always requested mid-stream (we already
        // have samples flowing), but clear the flag so the banner
        // briefly re-shows "calibrating" rather than "waiting for USB".
        samples_flowing_.store(true, std::memory_order_release);
        // Reset the silent-stream watchdog baseline so a dead stream
        // gets re-reported on the next attempt instead of instantly.
        worker_start_time_ = CFAbsoluteTimeGetCurrent();
        // Re-arm the single-shot activation nudge so a user-triggered
        // recovery can benefit from it even within one session.
        nudge_sent_ = false;
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
        // We used to short-circuit calibration on the "worn" proximity
        // bit here, but that bit is unreliable as a "head present"
        // signal: any flat surface a few cm in front of the visor
        // trips it (e.g. headset placed visor-down on a desk, which
        // is a perfectly valid calibration position). The CALIB_FAIL_
        // MOVING gate at the end of the averaging window already
        // catches the actual problem the worn check was guarding
        // against - a worn headset always shows enough head motion
        // (breathing + pulse + sway, peak |gyro| ~5-15 dps) to
        // exceed CALIB_MAX_MOTION_DPS, so calibration fails cleanly
        // there without false rejections from the proximity sensor.
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
                // If we still have auto-retries left, silently reset the
                // accumulators and try once more - see the comment on
                // CALIB_AUTO_RETRIES in psvr.h for the rationale. The
                // user never sees the failure banner in the common
                // "PSVR just finished activating, tail spike landed
                // inside the averaging window" case. Only when all
                // retries have been consumed does CALIB_FAIL_MOVING
                // get surfaced to the UI.
                if (calib_auto_retries_left_ > 0) {
                    qDebug().nospace()
                        << "PSVR: calibration auto-retry (peak |gyro|="
                        << calib_peak_gyro_dps_ << " dps); "
                        << calib_auto_retries_left_ << " retries left";
                    --calib_auto_retries_left_;
                    calib_count_         = 0;
                    calib_warmup_count_  = 0;
                    calib_peak_gyro_dps_ = 0.0;
                    calib_gyro_accum_[0]  = calib_gyro_accum_[1]  = calib_gyro_accum_[2]  = 0;
                    calib_accel_accum_[0] = calib_accel_accum_[1] = calib_accel_accum_[2] = 0;
                    return;
                }
                qWarning().nospace()
                    << "PSVR: calibration REJECTED - headset moved during "
                    << "averaging (peak |gyro|=" << calib_peak_gyro_dps_
                    << " dps > " << CALIB_MAX_MOTION_DPS << " dps) after "
                    << CALIB_AUTO_RETRIES << " auto-retries. "
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
            calibrated_.store(true, std::memory_order_release);
            qDebug() << "PSVR: calibrated."
                     << "gyro bias (deg/s):" << gyro_bias_[0] << gyro_bias_[1] << gyro_bias_[2]
                     << " seed pitch:" << seed_pitch << " seed roll:" << seed_roll
                     << " local gravity mag:" << avg_mag << "g";
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

    // Per-sample dt from the PSVR's own 24-bit microsecond timestamp
    // (at bytes 0-3 of each sample group). This is the IMU's hardware
    // sample-rate crystal, so it's immune to OS scheduler jitter that
    // plagues a wall-clock-based dt. The counter wraps at 0xFFFFFF
    // microseconds (~16.7 s); the rollover handling below follows
    // OpenHMD's pattern: the uint32_t subtraction produces a very
    // large value when wrapping, we detect and normalize. Bounded to
    // [DT_MIN, DT_MAX] to defang any garbage reading (e.g., a stale
    // buffered sample from a previous session, which the PSVR is
    // documented to sometimes emit at session start).
    const uint32_t tick =
          (uint32_t)buf[0]
        | ((uint32_t)buf[1] << 8)
        | ((uint32_t)buf[2] << 16)
        | ((uint32_t)buf[3] << 24);
    double dt;
    if (last_sample_tick_valid_) {
        uint32_t tick_delta = tick - last_sample_tick_;
        if (tick_delta > 0xFFFFFFu)   // 24-bit counter wrapped
            tick_delta += 0x1000000u;
        dt = tick_delta * 1e-6;       // microseconds → seconds
        if (dt < DT_MIN) dt = DT_MIN;
        if (dt > DT_MAX) dt = DT_MAX;
    } else {
        dt = DT_FALLBACK;
        last_sample_tick_valid_ = true;
    }
    last_sample_tick_ = tick;

    double yaw   = yaw_.load(std::memory_order_relaxed)   + gy * dt;
    double pitch = ALPHA * (pitch_.load(std::memory_order_relaxed) + gp * dt) + (1.0 - ALPHA) * acc_pitch;
    double roll  = ALPHA * (roll_.load(std::memory_order_relaxed)  + gr * dt) + (1.0 - ALPHA) * acc_roll;

    if (yaw >  180.0) yaw -= 360.0;
    if (yaw < -180.0) yaw += 360.0;

    yaw_.store(yaw,     std::memory_order_relaxed);
    pitch_.store(pitch, std::memory_order_relaxed);
    roll_.store(roll,   std::memory_order_relaxed);

#ifdef PSVR_HAS_CAMERA
    // Push the freshest attitude to the camera worker, which uses it
    // as a prior for projecting the 9-LED model into image space when
    // matching blobs. Cheap (3 atomic stores); called at ~2 kHz IMU
    // sample rate; the camera pipeline reads it at ~30 Hz. Converting
    // to radians here means the camera code doesn't have to know or
    // care about opentrack's degree convention.
    if (camera_worker_) {
        const double deg2rad = M_PI / 180.0;
        camera_worker_->set_rotation_prior(yaw   * deg2rad,
                                           pitch * deg2rad,
                                           roll  * deg2rad);
    }
#endif

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
            // Camera (constellation) diagnostics. When the camera
            // worker isn't running, we log zeros; consumers can tell
            // by seeing constant-zero columns that the feature was
            // disabled for this session.
            uint64_t cam_frames = 0, cam_any_blob = 0, cam_pnp_ok = 0;
            int cam_last_blobs = 0, cam_last_matched = 0;
            int cam_last_ok = 0;
            double cam_x = 0, cam_y = 0, cam_z = 0;
#ifdef PSVR_HAS_CAMERA
            if (camera_worker_ && camera_worker_->is_running()) {
                const auto d = camera_worker_->diag();
                cam_frames       = d.frames_captured;
                cam_any_blob     = d.frames_with_any_blob;
                cam_pnp_ok       = d.pnp_ok_count;
                cam_last_blobs   = d.last_n_blobs;
                cam_last_matched = d.last_n_matched;
                cam_last_ok      = d.last_pnp_ok ? 1 : 0;
                cam_x = d.last_x_cm;
                cam_y = d.last_y_cm;
                cam_z = d.last_z_cm;
            }
#endif
            std::fprintf(diag_log_,
                "%.3f\t%7.2f\tRUN\t%+8.3f\t%+8.3f\t%+8.3f"
                "\t%+7.3f\t%+7.3f\t%+7.3f"
                "\t%+6.3f\t%+6.3f\t%+6.3f"
                "\t%+6.3f\t%+6.3f\t%+6.3f\t%5.3f"
                "\t%+7.2f\t%+7.2f"
                "\t%.1f\t%llu\t%d"
                "\t%d\t%d\t%u\t%u"
                "\t%llu\t%llu\t%llu\t%d\t%d\t%d\t%+6.1f\t%+6.1f\t%+6.1f\n",
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
                (unsigned)ir_sensor_.load(std::memory_order_relaxed),
                (unsigned long long)cam_frames,
                (unsigned long long)cam_any_blob,
                (unsigned long long)cam_pnp_ok,
                cam_last_blobs, cam_last_matched, cam_last_ok,
                cam_x, cam_y, cam_z);
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
    // 10-bit IR proximity reading spans bytes 55-56, little-endian
    // (low byte at 55, high byte at 56). Verified against PSVRFramework's
    // sensor-frame decoder and against the values logged into our diag
    // (smooth 0-1023 sweep as a hand approaches the visor). An older
    // version of this comment said "big-endian" which contradicted the
    // code; the code was right.
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

// Send a single HID output report to every matched device. Used for
// courtesy commands like "set cinematic mode" on shutdown.
void PSVRTracker::send_raw_to_all(uint8_t cmd,
                                  const uint8_t* payload, size_t len)
{
    std::lock_guard<std::mutex> lk(devices_mu_);
    uint8_t report[64]{};
    report[0] = cmd;
    report[1] = 0x00;
    report[2] = 0xAA;
    report[3] = static_cast<uint8_t>(len);
    if (payload && len > 0) std::memcpy(report + 4, payload, len);
    for (auto& md : devices_)
        IOHIDDeviceSetReport(md.device, kIOHIDReportTypeOutput,
                             cmd, report, 4 + len);
}

// Put the headset into cinematic (single-screen) display mode. Called
// on tracker shutdown so the PSVR isn't stuck in 960×2 VR split-
// screen mode afterwards — a small UX courtesy borrowed from OpenHMD's
// drv_psvr close_device(). The in-line remote's mode button can also
// do this manually, but automating it means the user doesn't have to.
void PSVRTracker::send_cinematic_mode_to_all()
{
    const uint8_t vrmode_off[4] = {0x00, 0x00, 0x00, 0x00};
    send_raw_to_all(0x23, vrmode_off, sizeof(vrmode_off));
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
            && calib_count_ == 0)
        {
            const double elapsed =
                CFAbsoluteTimeGetCurrent() - worker_start_time_;

            // Nudge: at the halfway point to the hard timeout, if we
            // still haven't heard from the PSVR, re-fire the
            // activation burst ONCE. Common cause of silent startup
            // is that our first activation arrived while the PSVR's
            // own firmware was mid-boot and got dropped; retrying
            // after the firmware has had a chance to settle
            // frequently wakes it up without needing user action.
            if (!nudge_sent_ && elapsed > NO_DATA_NUDGE_SEC) {
                qWarning() << "PSVR: no HID reports within"
                           << NO_DATA_NUDGE_SEC
                           << "s of Start - retrying activation burst";
                send_activation_to_all();
                nudge_sent_ = true;
            }

            // Hard fail only once the full timeout has elapsed. By
            // this point we've given the PSVR the full cold-boot
            // window and one retry; if it's still silent it's not
            // coming back on its own.
            if (elapsed > NO_DATA_TIMEOUT_SEC) {
                qWarning() << "PSVR: no HID reports within"
                           << NO_DATA_TIMEOUT_SEC
                           << "s of Start - headset off / unplugged /"
                              " permissions missing";
                calib_failure_.store(CALIB_FAIL_NO_DATA,
                                     std::memory_order_release);
            }
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

    camera_box_ = new QCheckBox(QObject::tr(
        "Enable camera-based position tracking [experimental]\n"
        "Uses the PSVR's built-in blue LEDs + a webcam to recover head "
        "X/Y/Z. First activation requests Camera permission. Blob "
        "detection runs on every frame; PnP solver lands in a follow-"
        "up commit, so today this only records diagnostic data."));
    camera_box_->setChecked(s_.enable_camera);
    layout->addWidget(camera_box_);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);
    QObject::connect(buttons, &QDialogButtonBox::accepted, this, [this]() {
        s_.enable_mirror   = mirror_box_->isChecked();
        s_.enable_diag_log = diag_log_box_->isChecked();
        s_.enable_camera   = camera_box_->isChecked();
        s_.b->save();
        accept();
    });
    QObject::connect(buttons, &QDialogButtonBox::rejected, this, [this]() {
        s_.b->reload();
        reject();
    });
}

OPENTRACK_DECLARE_TRACKER(PSVRTracker, PSVRDialog, PSVRMetadata)
