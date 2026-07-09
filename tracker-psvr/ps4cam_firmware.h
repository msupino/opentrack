/* PS4 Camera (OV580) firmware uploader — public surface.
 *
 * Background
 * ----------
 * The PS4 Camera (Sony's stereo webcam, originally designed to track
 * the PSVR's blue LEDs) houses an OmniVision OV580 ASIC. The OV580
 * powers up running a stub bootloader that does NOT expose a UVC
 * interface — AVFoundation sees nothing useful, the camera does not
 * appear in any picker, and any "open the PS4 Camera" attempt fails.
 *
 * Before the device is usable as a webcam, the host must upload a
 * ~65 KB firmware blob over USB control transfers. After the upload
 * + a final "boot" control transfer, the OV580 detaches and re-enters
 * USB enumeration ~1 second later as a regular UVC webcam (PID 0x058a
 * for PS4, 0x058c for PS5). From that point on AVFoundation treats
 * it like any other camera.
 *
 * Why this lives in the PSVR plugin
 * ---------------------------------
 * The PS4 Camera is literally the camera Sony built for tracking the
 * PSVR's blue-LED constellation. Users who plug one in via the
 * PlayStation Camera adapter expect it to "just work" with our LED
 * tracker — the same way it would on a PS4. External tools
 * (psmoveapi's `psmove camera-firmware`, PSVRTracker's bring-up
 * script) require the user to know the camera exists, install a
 * separate project, and run a CLI command before opentrack can see
 * the device. Folding the 200 LOC of upload logic into tracker-psvr
 * removes that friction and the camera shows up in the dropdown
 * automatically.
 *
 * Contract
 * --------
 * `ensure_firmware_uploaded()` is the one public entry point.
 *
 *   - Returns true when an already-UVC PS4/PS5 Camera is detected
 *     (no work needed), OR when a boot-mode OV580 was found and the
 *     firmware upload completed successfully AND macOS subsequently
 *     re-enumerated the device as a UVC video source within the
 *     poll window.
 *   - Returns false when no PS4/PS5 Camera is plugged in at all
 *     (the common case for users without one — callers should
 *     silently continue), or when an upload was attempted but
 *     failed at some step.
 *
 * The function is idempotent and cheap when no boot-mode device is
 * present: it just enumerates libusb devices once, finds no match,
 * and returns. Safe to call from the Qt UI thread (dialog populate)
 * and from the camera worker's start() path on the worker dispatch
 * queue.
 *
 * All logging uses the existing `[psvr-cam]` stderr prefix. We
 * deliberately stay silent in the "no device found" case so that
 * every user who doesn't own a PS4 Camera doesn't see noise every
 * time the dialog opens or the tracker starts.
 *
 * Reference implementation: psmoveapi/src/utils/ps4_camera_firmware.cpp
 * by Thomas Perl, which in turn cites Antonio Jose Ramos Marquez's
 * PS4EYECam driver as the protocol source.
 */
#pragma once

namespace ps4cam {

bool ensure_firmware_uploaded();

} // namespace ps4cam
