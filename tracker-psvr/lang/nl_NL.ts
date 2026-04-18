<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="nl_NL">
<context>
    <name>PSVRMetadata</name>
    <message>
        <source>PSVR (IMU)</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <source>No data received from the PSVR.
• Is the USB cable connected?
• Is the headset&apos;s in-line power box lit up?
• Is Input Monitoring granted to opentrack in
  System Settings &gt; Privacy &amp; Security?
Fix the issue, then press Re-calibrate below.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Waiting for PSVR on USB…
The headset&apos;s processor unit can take 5-15 seconds to reply
after Start (especially on first connection). If this banner
stays up for more than ~15 s the watchdog will surface a
specific error below.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>The PSVR headset appears to be OFF.
Data is flowing from the USB processor unit, but
the accelerometer is not measuring gravity —
the headset itself is asleep.
Press the power button on the in-line remote
until the screen wakes, then press Re-calibrate.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>PSVR calibration failed.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Calibrating gyroscope (~3 s)
Keep the PSVR PERFECTLY STILL on a flat surface.
DO NOT put it on your head yet — head motion during
calibration will bake drift into the tracker.
(1 s sensor-stream warmup + 2 s bias averaging.)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Re-calibrate gyroscope</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Rerun the 2-second gyro-bias calibration without stopping the tracker. Put the PSVR back on a flat still surface first.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>PSVR tracker settings</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>PlayStation VR IMU tracker for macOS.

Reads the PSVR headset&apos;s gyroscope and accelerometer over USB HID and provides yaw/pitch/roll to opentrack. Requires Input Monitoring permission for opentrack in System Settings.

Calibration: after USB comes up (up to ~15 s on first start), the tracker waits one second for the sensor stream to stabilize, then spends two seconds measuring the gyro&apos;s resting bias. Keep the PSVR still on a flat surface during the whole window — not on your head — or accumulated head motion will become baked-in drift. Press &quot;Re-calibrate&quot; later to redo it.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Write diagnostic log to /tmp/psvr-diag.log (pose, gyro, accel, bias, sample rate; one row per second)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Mirror the main display side-by-side onto the PSVR screen (requires Screen Recording permission)</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
