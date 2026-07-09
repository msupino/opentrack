/* PS4 Camera (OV580) firmware uploader — implementation.
 *
 * Talks to a boot-mode OV580 over libusb control transfers, ships the
 * 65 KB firmware blob (`ps4cam_firmware_blob.h`), pokes the device to
 * boot, and then waits for AVFoundation to publish the resulting UVC
 * device so the rest of the plugin can open it the normal way.
 *
 * Protocol
 * --------
 * Reverse-engineered by the PS4EYECam project (Antonio Jose Ramos
 * Marquez, GPLv2); ported in pared-down form by psmoveapi (Thomas Perl,
 * BSD); this file follows the psmoveapi reference path:
 *
 *   - Boot-mode device:  VID 0x05a9 / PID 0x0580 (OmniVision)
 *   - Configuration 1, interface 0.
 *   - Stream the firmware in 512-byte chunks via control transfer:
 *         bmRequestType = 0x40 (vendor, host->device)
 *         bRequest      = 0x00
 *         wValue        = (offset & 0xffff)
 *         wIndex        = 20 + (offset >> 16)
 *   - Final 1-byte "boot now" transfer at wValue=0x2200, wIndex=0x8018.
 *     macOS reports it as failed (the device is already disconnecting);
 *     that's the documented success signal on this OS.
 *
 * After the boot transfer the OV580 re-enumerates on the same bus as
 * a UVC webcam (PID 0x058a for PS4, 0x058c for PS5). We then ask
 * AVFoundation to confirm the device appeared, polling every 100 ms
 * for up to 3 s. Once visible, AVCaptureDevice/AVCaptureSession can
 * open it like any other camera.
 *
 * Concurrency / safety
 * --------------------
 * ensure_firmware_uploaded() does its own libusb_init/libusb_exit per
 * call so there is no shared context to keep alive between calls.
 * libusb is thread-safe across separate contexts, so multiple call
 * sites (start() and the dialog's "populate-and-then-repopulate"
 * QTimer) can race without interfering. The function is also
 * idempotent: if a UVC PS4/PS5 Camera is already enumerated when we
 * look, we skip straight to "true" without touching USB.
 *
 * No `qDebug()` calls anywhere — this file is only called from
 * non-frame-rate paths (worker start / dialog populate), but we keep
 * stderr-only logging consistent with the rest of psvr_camera.mm so
 * users see one coherent `[psvr-cam] ...` stream.
 */

#import "ps4cam_firmware.h"
#import "ps4cam_firmware_blob.h"

#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>

#include <libusb.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>

namespace ps4cam {

namespace {

// Canonical VID/PIDs cited by psmoveapi, ps4eye, and the PSVR
// reverse-engineering community.
constexpr uint16_t kVidOmnivision        = 0x05a9;
constexpr uint16_t kPidOv580BootMode     = 0x0580;
constexpr uint16_t kPidPs4Camera         = 0x058a;
constexpr uint16_t kPidPs5Camera         = 0x058c;

// Control transfer parameters (host -> device, vendor, request 0).
constexpr uint8_t  kReqType              = 0x40;
constexpr uint8_t  kReq                  = 0x00;
constexpr size_t   kChunkSize            = 512;
constexpr uint16_t kBootValue            = 0x2200;
constexpr uint16_t kBootIndex            = 0x8018;
constexpr uint8_t  kBootPayload          = 0x5b;
constexpr unsigned kCtlTimeoutMs         = 1000;

// AVFoundation re-enumeration poll. Empirically the device reappears
// ~500–1500 ms after the boot transfer; 3000 ms gives comfortable
// headroom without blocking the UI thread too long.
constexpr int      kUvcWaitTotalMs       = 3000;
constexpr int      kUvcWaitStepMs        = 100;

bool device_is(const libusb_device_descriptor& desc,
               uint16_t vid, uint16_t pid)
{
    return desc.idVendor == vid && desc.idProduct == pid;
}

// Returns YES if AVFoundation currently sees a video device whose
// human-readable name plausibly identifies the OV580-based PS4/PS5
// Camera. Sony's USB descriptor strings render as "OV580 CAMERA" or
// "USB Camera-OV580" depending on firmware revision, so we match by
// substring rather than exact name. Case-insensitive.
bool av_uvc_ov580_present()
{
    @autoreleasepool {
        NSMutableArray<AVCaptureDeviceType>* types = [NSMutableArray array];
        if (@available(macOS 14.0, *)) {
            [types addObject:AVCaptureDeviceTypeExternal];
        } else {
            [types addObject:(AVCaptureDeviceType)@"AVCaptureDeviceTypeExternalUnknown"];
        }
        // Include the builtin type too so we don't accidentally miss
        // a Camera that AVFoundation classifies oddly post-reboot.
        [types addObject:AVCaptureDeviceTypeBuiltInWideAngleCamera];

        AVCaptureDeviceDiscoverySession* ds =
            [AVCaptureDeviceDiscoverySession
                discoverySessionWithDeviceTypes:types
                                      mediaType:AVMediaTypeVideo
                                       position:AVCaptureDevicePositionUnspecified];
        for (AVCaptureDevice* d in ds.devices) {
            NSString* name = d.localizedName.lowercaseString ?: @"";
            if ([name containsString:@"ov580"]) return true;
            // The PS4 Camera adapter sometimes presents the device as
            // "USB Camera" with no OV580 mention; fall back to a
            // uniqueID/manufacturer signal where possible. uniqueID
            // on USB cameras is typically "<vid><pid>-…"; match the
            // OmniVision VID prefix as a secondary signal.
            NSString* uid = d.uniqueID.lowercaseString ?: @"";
            if ([uid containsString:@"05a9"] &&
                ([uid containsString:@"058a"] || [uid containsString:@"058c"])) {
                return true;
            }
        }
        return false;
    }
}

// Look through the libusb device list once. Sets *out_boot_dev to a
// boot-mode OV580 if found (caller owns nothing; libusb retains the
// reference for the lifetime of the device list, which the caller
// holds via `devs`). Returns true if an already-UVC PS4/PS5 Camera
// is plugged in — in that case there is no work to do.
bool scan_devices(libusb_device** devs, ssize_t count,
                  libusb_device** out_boot_dev)
{
    *out_boot_dev = nullptr;
    bool uvc_present = false;
    for (ssize_t i = 0; i < count; ++i) {
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(devs[i], &desc) != 0) continue;
        if (device_is(desc, kVidOmnivision, kPidPs4Camera) ||
            device_is(desc, kVidOmnivision, kPidPs5Camera)) {
            uvc_present = true;
        } else if (device_is(desc, kVidOmnivision, kPidOv580BootMode)) {
            *out_boot_dev = devs[i];
        }
    }
    return uvc_present;
}

// Push the firmware blob to the OV580 over libusb control transfers
// and trigger the boot. Returns true if every chunk shipped AND the
// boot transfer behaved as the protocol expects (the device drops
// off the bus on the final write, which libusb surfaces as
// LIBUSB_ERROR_NO_DEVICE on Linux and as -1 on macOS).
bool upload(libusb_device* dev)
{
    libusb_device_handle* handle = nullptr;
    int rc = libusb_open(dev, &handle);
    if (rc != 0) {
        std::fprintf(stderr,
            "[psvr-cam] PS4 Camera firmware: libusb_open failed (%d: %s)\n",
            rc, libusb_strerror((libusb_error)rc));
        return false;
    }

    // Best-effort reset + claim. libusb_set_configuration may fail
    // with LIBUSB_ERROR_BUSY on macOS if some other driver already
    // owns the device, but that's fine for a boot-mode OV580 (the
    // OS has no UVC driver bound to it yet). Errors here are fatal
    // for the upload but we still want to release / close to keep
    // the next attempt clean.
    auto cleanup_open = [&]() {
        if (handle) {
            libusb_release_interface(handle, 0);
            libusb_close(handle);
            handle = nullptr;
        }
    };

    rc = libusb_reset_device(handle);
    if (rc != 0) {
        std::fprintf(stderr,
            "[psvr-cam] PS4 Camera firmware: libusb_reset_device failed (%d: %s)\n",
            rc, libusb_strerror((libusb_error)rc));
        cleanup_open();
        return false;
    }
    rc = libusb_set_configuration(handle, 1);
    if (rc != 0 && rc != LIBUSB_ERROR_BUSY) {
        std::fprintf(stderr,
            "[psvr-cam] PS4 Camera firmware: libusb_set_configuration failed (%d: %s)\n",
            rc, libusb_strerror((libusb_error)rc));
        cleanup_open();
        return false;
    }
    rc = libusb_claim_interface(handle, 0);
    if (rc != 0) {
        std::fprintf(stderr,
            "[psvr-cam] PS4 Camera firmware: libusb_claim_interface failed (%d: %s)\n",
            rc, libusb_strerror((libusb_error)rc));
        cleanup_open();
        return false;
    }

    // Stream firmware in 512-byte chunks. The final chunk may be short.
    for (size_t offset = 0; offset < kPS4CamFirmwareSize; offset += kChunkSize) {
        const size_t remaining = kPS4CamFirmwareSize - offset;
        const size_t this_chunk = remaining < kChunkSize ? remaining : kChunkSize;
        // libusb_control_transfer takes a non-const data pointer even
        // for host->device transfers; cast away const on our blob's
        // pointer since libusb is just reading from it.
        uint8_t* data = const_cast<uint8_t*>(&kPS4CamFirmware[offset]);
        const uint16_t wValue = static_cast<uint16_t>(offset & 0xffff);
        const uint16_t wIndex = static_cast<uint16_t>(20 + (offset >> 16));
        const int sent = libusb_control_transfer(
            handle, kReqType, kReq, wValue, wIndex,
            data, static_cast<uint16_t>(this_chunk), kCtlTimeoutMs);
        if (sent != static_cast<int>(this_chunk)) {
            std::fprintf(stderr,
                "[psvr-cam] PS4 Camera firmware: chunk at %zu of %zu failed "
                "(libusb_control_transfer = %d, expected %zu)\n",
                offset, kPS4CamFirmwareSize, sent, this_chunk);
            cleanup_open();
            return false;
        }
    }

    // Final "boot now" transfer. Per the upstream reference, this is
    // expected to "fail" on macOS with rc == -1 because the device
    // is already detaching. Anything else (e.g. a clean ACK) would be
    // surprising but not fatal — the device's behavior is what we
    // verify next via the AVFoundation poll, so we treat this as
    // best-effort and don't error out on either branch.
    uint8_t boot_byte = kBootPayload;
    (void)libusb_control_transfer(
        handle, kReqType, kReq, kBootValue, kBootIndex,
        &boot_byte, 1, kCtlTimeoutMs);

    // Deliberately do NOT release/close: the device has already
    // disconnected, and calling release on a vanished handle on macOS
    // produces a benign but noisy IOReturn error. We let libusb_exit
    // (after this function returns) tear down the context, which
    // reaps the dangling handle cleanly.

    return true;
}

// Sleep for `step_ms` between AVFoundation polls. Total time bounded
// by kUvcWaitTotalMs.
bool wait_for_uvc_appearance()
{
    using clock = std::chrono::steady_clock;
    const auto deadline =
        clock::now() + std::chrono::milliseconds(kUvcWaitTotalMs);
    while (clock::now() < deadline) {
        if (av_uvc_ov580_present()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(kUvcWaitStepMs));
    }
    return av_uvc_ov580_present();
}

} // namespace

bool ensure_firmware_uploaded()
{
    libusb_context* ctx = nullptr;
    if (libusb_init(&ctx) != 0) {
        // libusb itself unavailable — nothing to do. Silent.
        return false;
    }

    libusb_device** devs = nullptr;
    const ssize_t count = libusb_get_device_list(ctx, &devs);
    if (count < 0) {
        libusb_exit(ctx);
        return false;
    }

    libusb_device* boot_dev = nullptr;
    const bool uvc_present = scan_devices(devs, count, &boot_dev);

    if (uvc_present) {
        // PS4/PS5 Camera already initialized (e.g. we previously
        // uploaded firmware this session, or the device was bring-
        // upped externally). Nothing to do. Stay silent: we don't
        // want to spam every dialog open with confirmation noise.
        libusb_free_device_list(devs, 1);
        libusb_exit(ctx);
        return true;
    }

    if (!boot_dev) {
        // No PS4 Camera attached at all. Silent — most users won't
        // own one and don't need to see a message about it.
        libusb_free_device_list(devs, 1);
        libusb_exit(ctx);
        return false;
    }

    std::fprintf(stderr,
        "[psvr-cam] PS4 Camera detected in Boot Mode, uploading firmware...\n");
    const bool ok = upload(boot_dev);
    // After upload() the boot_dev pointer is logically dead (device
    // disconnected). Free the device list before doing AVFoundation
    // work to release libusb's per-device reference counts.
    libusb_free_device_list(devs, 1);

    if (!ok) {
        libusb_exit(ctx);
        std::fprintf(stderr,
            "[psvr-cam] PS4 Camera firmware upload failed; "
            "leaving device in Boot Mode\n");
        return false;
    }

    std::fprintf(stderr,
        "[psvr-cam] PS4 Camera firmware uploaded (%zu bytes), "
        "waiting for UVC re-enumeration...\n",
        kPS4CamFirmwareSize);

    const bool ready = wait_for_uvc_appearance();
    libusb_exit(ctx);

    if (ready) {
        std::fprintf(stderr,
            "[psvr-cam] PS4 Camera ready (UVC enumeration complete)\n");
    } else {
        std::fprintf(stderr,
            "[psvr-cam] PS4 Camera firmware uploaded but UVC device did "
            "not appear within %d ms; the camera may still come up "
            "shortly — try again if it doesn't\n",
            kUvcWaitTotalMs);
    }
    return ready;
}

} // namespace ps4cam
