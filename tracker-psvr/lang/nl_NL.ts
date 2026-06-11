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
        <source>PSVR calibration failed.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Waiting for PSVR on USB… cold-boot activation can take up to 20 seconds. If the stream doesn&apos;t start after ~12 s the driver will retry activation once automatically; an error banner appears only if nothing arrives by ~25 s.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>No data received from the PSVR.
• Is the USB cable connected?
• Is the headset&apos;s in-line power box lit up?
• Is Input Monitoring granted to opentrack in System Settings &gt; Privacy &amp; Security?
Fix the issue, then press Re-calibrate below.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>PSVR not detected on USB.
• Check the cable from the helmet&apos;s processor unit to the Mac.
• Make sure the processor unit is powered (its small LED is on).
• Grant Input Monitoring permission to opentrack in System Settings &gt; Privacy &amp; Security.
Fix the issue, then press Re-calibrate below.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>The PSVR headset appears to be OFF.
Data is flowing from the USB processor unit, but the accelerometer is not measuring gravity — the headset itself is asleep.
Press the power button on the in-line remote until the screen wakes, then press Re-calibrate.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Can&apos;t calibrate while the headset is worn.
Calibration needs the PSVR still on a flat surface (desk, table) so gyro bias can be measured without body-motion contamination.
Take it off, put it down, and press Re-calibrate. You can put it back on once the amber &apos;Calibrating&apos; banner clears.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Calibration rejected — the headset moved.
During the 2-second averaging window, the gyroscope measured more motion than the noise floor. If you were holding the PSVR or walking with it, set it down on a flat still surface (desk, table) and press Re-calibrate.
If you were not moving it, a bump, knock or cable tug is usually the cause.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Calibrating gyroscope (~3 s)
Keep the PSVR PERFECTLY STILL on a flat surface. DO NOT put it on your head yet — head motion during calibration will bake drift into the tracker.
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
        <source>Mirror main display onto PSVR (side-by-side)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Captures the main display and presents it as a stereoscopic side-by-side image on the PSVR screen. Requires Screen Recording permission.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Write diagnostic log</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Writes one row per second to /tmp/psvr-diag.log: pose, gyro, accel, bias, sample rate. Useful for debugging tracking issues.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Enable camera-based position tracking [experimental]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Uses the PSVR&apos;s built-in blue LEDs and a webcam to recover head X/Y/Z position. First activation requests Camera permission. Blob detection runs every frame; PnP solver lands in a follow-up commit, so today this only records diagnostic data.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Camera</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>(default camera)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source> (not connected)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Camera used for the LED-constellation tracker above. &quot;(default camera)&quot; picks whatever AVFoundation considers primary (usually the lid camera on MacBooks). A USB webcam or PS Camera generally tracks better.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Camera HFOV</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source> °</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Horizontal field of view of the camera selected above, in degrees. Used by the LED-constellation PnP solver to recover head distance correctly; a mismatched HFOV lands tracking at the wrong Z. Find this value on the camera&apos;s spec sheet, or experimentally tune until the on-screen tracking distance matches reality. Common values: 70 (typical lid webcam), 78 (FaceTime HD), 80 (UGREEN), 75 (PS Camera), 120 (GoPro).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Auto (set HFOV by camera type)</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
