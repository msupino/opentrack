/* psvr-poweron.c - one-shot PSVR activation utility.
 *
 * Enumerates Sony PSVR (VID 0x054c / PID 0x09af) via IOKit HID,
 * sends the three-command activation sequence exactly as the
 * tracker-psvr opentrack plugin does on device-match, and exits.
 *
 * The PSVR exposes MULTIPLE HID interfaces on the same VID/PID (the
 * sensor stream and the control interface, at minimum). Activation
 * only takes effect on the control interface — and SetReport on the
 * wrong interface can return success while doing nothing — so this
 * tool mirrors the plugin's send_activation_to_all(): it waits for
 * the whole interface set to enumerate, then sends the burst to
 * EVERY matched device. (The original version fired on the first
 * match and exited, which silently no-oped whenever the sensor
 * interface happened to enumerate first.)
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
 *   0  at least one interface accepted the full activation burst
 *   1  IOHIDManagerOpen failed (check Input Monitoring permission)
 *   2  no PSVR matched within 3 seconds (not plugged in / USB dead)
 *   3  PSVR matched but every SetReport failed
 */

#include <IOKit/hid/IOHIDManager.h>
#include <CoreFoundation/CoreFoundation.h>
#include <stdio.h>
#include <string.h>

#define PSVR_VID 0x054cu
#define PSVR_PID 0x09afu

#define MAX_DEVICES 8
static IOHIDDeviceRef g_devices[MAX_DEVICES];
static int g_n_devices = 0;

/* Collect every matched interface; the burst is sent after the
 * enumeration window closes, not from inside the callback. */
static void device_matched_cb(void *ctx, IOReturn result, void *sender,
                              IOHIDDeviceRef device)
{
    (void)ctx; (void)result; (void)sender;
    if (g_n_devices < MAX_DEVICES)
        g_devices[g_n_devices++] = device;
}

static long device_long_prop(IOHIDDeviceRef dev, CFStringRef key)
{
    long v = -1;
    CFTypeRef ref = IOHIDDeviceGetProperty(dev, key);
    if (ref && CFGetTypeID(ref) == CFNumberGetTypeID())
        CFNumberGetValue((CFNumberRef)ref, kCFNumberLongType, &v);
    return v;
}

/* Returns 1 if the command was accepted, 0 otherwise. */
static int send_cmd(IOHIDDeviceRef device, uint8_t cmd,
                    const uint8_t *payload, size_t len)
{
    uint8_t report[64] = {0};
    report[0] = cmd;
    report[1] = 0x00;   /* "unknown, 0 always works" header byte */
    report[2] = 0xAA;
    report[3] = (uint8_t)len;
    if (payload && len > 0) memcpy(report + 4, payload, len);
    IOReturn r = IOHIDDeviceSetReport(device, kIOHIDReportTypeOutput,
                                      cmd, report, 4 + len);
    if (r != kIOReturnSuccess) {
        fprintf(stderr, "    cmd 0x%02x FAILED: IOReturn=0x%08x\n", cmd, r);
        return 0;
    }
    printf("    cmd 0x%02x OK\n", cmd);
    return 1;
}

/* Full activation burst, identical to the plugin's send_activation().
 * Returns the number of commands (0..3) that were accepted. */
static int send_activation(IOHIDDeviceRef device)
{
    int ok = 0;

    /* 0x17 SetHeadsetPower(ON) - wakes the HMD display + sensor chain */
    const uint8_t on_4[4] = {0x01, 0x00, 0x00, 0x00};
    ok += send_cmd(device, 0x17, on_4, sizeof(on_4));

    /* 0x11 EnableTracking(0xFFFFFF00) - full IMU tracking + blue LEDs */
    const uint8_t enable_tracking[8] = {
        0x00, 0xFF, 0xFF, 0xFF,
        0x00, 0x00, 0x00, 0x00
    };
    ok += send_cmd(device, 0x11, enable_tracking, sizeof(enable_tracking));

    /* 0x23 SetVRMode(ON) - 960+960 split-screen VR display layout */
    ok += send_cmd(device, 0x23, on_4, sizeof(on_4));

    return ok;
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

    /* Enumeration window: wait up to 3 s for the FIRST match, then a
     * further 0.5 s so the device's remaining sibling interfaces (which
     * arrive as separate matched callbacks within milliseconds) are all
     * collected before we fire. */
    CFAbsoluteTime deadline = CFAbsoluteTimeGetCurrent() + 3.0;
    while (g_n_devices == 0 && CFAbsoluteTimeGetCurrent() < deadline)
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.1, true);
    if (g_n_devices > 0)
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.5, false);

    int total_ok = 0;
    if (g_n_devices > 0) {
        printf("PSVR: %d HID interface(s) matched, sending activation to all:\n",
               g_n_devices);
        for (int i = 0; i < g_n_devices; ++i) {
            const long up = device_long_prop(g_devices[i],
                                             CFSTR(kIOHIDPrimaryUsagePageKey));
            const long u  = device_long_prop(g_devices[i],
                                             CFSTR(kIOHIDPrimaryUsageKey));
            printf("  interface %d (usage page 0x%02lx usage 0x%02lx):\n",
                   i, up, u);
            total_ok += send_activation(g_devices[i]);
        }
        printf("done: %d command(s) accepted across %d interface(s).\n",
               total_ok, g_n_devices);
    }

    IOHIDManagerClose(mgr, kIOHIDOptionsTypeNone);
    CFRelease(mgr);

    if (g_n_devices == 0) {
        fprintf(stderr,
            "no PSVR matched within 3 s. Check:\n"
            "  - USB cable plugged in at both ends\n"
            "  - PSVR processor box is receiving power (blue light)\n"
            "  - another process isn't holding the device exclusively\n");
        return 2;
    }
    return total_ok > 0 ? 0 : 3;
}
