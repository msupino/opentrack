/* psvr-poweron.c - one-shot PSVR activation utility.
 *
 * Enumerates Sony PSVR (VID 0x054c / PID 0x09af) via IOKit HID,
 * sends the three-command activation sequence exactly as the
 * tracker-psvr opentrack plugin does on device-match, and exits.
 *
 * Useful when the headset has powered down (~8 minute auto-sleep,
 * or manual off) and you want to wake it without starting the
 * full tracker. Needs Input Monitoring permission for the binary
 * that runs it (i.e., the terminal / parent process), because
 * IOHIDManagerOpen on a non-keyboard HID device is gated on that.
 *
 * Build:
 *   clang -O2 -Wall -Wextra -o dev/psvr-poweron dev/psvr-poweron.c \
 *         -framework IOKit -framework CoreFoundation
 *
 * Exit codes:
 *   0  at least one PSVR received the activation burst
 *   1  IOHIDManagerOpen failed (check Input Monitoring permission)
 *   2  no PSVR matched within 3 seconds (not plugged in / USB dead)
 */

#include <IOKit/hid/IOHIDManager.h>
#include <CoreFoundation/CoreFoundation.h>
#include <stdio.h>
#include <string.h>

#define PSVR_VID 0x054cu
#define PSVR_PID 0x09afu

static int g_found = 0;

static void send_cmd(IOHIDDeviceRef device, uint8_t cmd,
                     const uint8_t *payload, size_t len)
{
    uint8_t report[64] = {0};
    report[0] = cmd;
    report[1] = 0x00;
    report[2] = 0xAA;
    report[3] = (uint8_t)len;
    if (payload && len > 0) memcpy(report + 4, payload, len);
    IOReturn r = IOHIDDeviceSetReport(device, kIOHIDReportTypeOutput,
                                      cmd, report, 4 + len);
    if (r != kIOReturnSuccess)
        fprintf(stderr, "  cmd 0x%02x FAILED: IOReturn=0x%08x\n", cmd, r);
    else
        printf("  cmd 0x%02x OK\n", cmd);
}

static void device_matched_cb(void *ctx, IOReturn result, void *sender,
                              IOHIDDeviceRef device)
{
    (void)ctx; (void)result; (void)sender;
    g_found = 1;
    printf("PSVR matched (VID=0x%04x PID=0x%04x), sending activation:\n",
           PSVR_VID, PSVR_PID);

    /* 0x17 SetHeadsetPower(ON) - wakes the HMD display + sensor chain */
    const uint8_t on_4[4] = {0x01, 0x00, 0x00, 0x00};
    send_cmd(device, 0x17, on_4, sizeof(on_4));

    /* 0x11 EnableTracking(0xFFFFFF00) - full IMU tracking + blue LEDs */
    const uint8_t enable_tracking[8] = {
        0x00, 0xFF, 0xFF, 0xFF,
        0x00, 0x00, 0x00, 0x00
    };
    send_cmd(device, 0x11, enable_tracking, sizeof(enable_tracking));

    /* 0x23 SetVRMode(ON) - 960+960 split-screen VR display layout */
    send_cmd(device, 0x23, on_4, sizeof(on_4));

    printf("done.\n");

    /* Exit the runloop now that we've serviced one device. If multiple
     * PSVRs are ever connected we'd want to wait for them all, but
     * this is a developer utility — the common case is one. */
    CFRunLoopStop(CFRunLoopGetCurrent());
}

int main(void)
{
    IOHIDManagerRef mgr = IOHIDManagerCreate(kCFAllocatorDefault,
                                             kIOHIDOptionsTypeNone);
    if (!mgr) {
        fprintf(stderr, "IOHIDManagerCreate failed\n");
        return 1;
    }

    CFMutableDictionaryRef match = CFDictionaryCreateMutable(
        kCFAllocatorDefault, 2,
        &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    int vid = PSVR_VID, pid = PSVR_PID;
    CFNumberRef cvid = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &vid);
    CFNumberRef cpid = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &pid);
    CFDictionarySetValue(match, CFSTR(kIOHIDVendorIDKey),  cvid);
    CFDictionarySetValue(match, CFSTR(kIOHIDProductIDKey), cpid);
    CFRelease(cvid); CFRelease(cpid);

    IOHIDManagerSetDeviceMatching(mgr, match);
    CFRelease(match);

    IOHIDManagerRegisterDeviceMatchingCallback(mgr, device_matched_cb, NULL);
    IOHIDManagerScheduleWithRunLoop(mgr, CFRunLoopGetCurrent(),
                                    kCFRunLoopDefaultMode);

    IOReturn r = IOHIDManagerOpen(mgr, kIOHIDOptionsTypeNone);
    if (r != kIOReturnSuccess) {
        fprintf(stderr,
            "IOHIDManagerOpen failed (0x%08x). If this is 0xe00002c5,\n"
            "grant Input Monitoring permission to the terminal/shell\n"
            "process running this tool in System Settings > Privacy.\n", r);
        CFRelease(mgr);
        return 1;
    }

    /* Wait up to 3 seconds for matching + activation to land. */
    CFRunLoopRunInMode(kCFRunLoopDefaultMode, 3.0, false);

    IOHIDManagerClose(mgr, kIOHIDOptionsTypeNone);
    CFRelease(mgr);

    if (!g_found) {
        fprintf(stderr,
            "no PSVR matched within 3 s. Check:\n"
            "  - USB cable plugged in at both ends\n"
            "  - PSVR processor box is receiving power (blue light)\n"
            "  - another process isn't holding the device exclusively\n");
        return 2;
    }
    return 0;
}
