/* PSVR side-by-side display mirror (Objective-C++).
 * Captures the primary (non-PSVR) display via ScreenCaptureKit and
 * renders it twice on the PSVR screen so each eye sees the full image
 * in VR-mode split-screen.
 */
#import "psvr_mirror.h"

#import <AppKit/AppKit.h>
#import <ScreenCaptureKit/ScreenCaptureKit.h>
#import <CoreMedia/CoreMedia.h>
#import <CoreVideo/CoreVideo.h>
#import <IOSurface/IOSurface.h>
#import <QuartzCore/QuartzCore.h>
#import <CoreGraphics/CoreGraphics.h>

static CGDirectDisplayID screen_display_id(NSScreen *s) {
    NSNumber *n = s.deviceDescription[@"NSScreenNumber"];
    return n ? (CGDirectDisplayID)n.unsignedIntValue : kCGNullDirectDisplay;
}

// True if this display is part of ANY mirror set - either because it is
// mirroring another display, or because other displays are mirroring it.
// Rendering a window on a display that's part of a mirror set would also
// show up on the other mirror peers, which we want to avoid.
static BOOL screen_in_mirror_set(NSScreen *s) {
    CGDirectDisplayID did = screen_display_id(s);
    if (did == kCGNullDirectDisplay) return NO;
    if (CGDisplayMirrorsDisplay(did) != kCGNullDirectDisplay) return YES;
    if (CGDisplayIsInMirrorSet(did)) return YES;
    if (CGDisplayIsInHWMirrorSet(did)) return YES;
    return NO;
}

@interface PSVRMirrorImpl : NSObject <SCStreamDelegate, SCStreamOutput>
@property (nonatomic, strong) NSWindow *window;
@property (nonatomic, strong) CALayer  *leftLayer;
@property (nonatomic, strong) CALayer  *rightLayer;
@property (nonatomic, strong) SCStream *stream;
// Dedicated serial queue for SCStream output callbacks. Running them on
// a private queue (rather than the main queue) keeps AppKit responsive
// even if CoreAnimation momentarily blocks while presenting a layer
// update. The handler marshals the IOSurface swap back to the main
// queue inside a CATransaction.
@property (nonatomic, strong) dispatch_queue_t captureQueue;
@property (nonatomic, assign) BOOL enabled;
- (void)enable;
- (void)disable;
- (void)refresh;   // react to display hot-plug
@end

@implementation PSVRMirrorImpl

static BOOL screen_looks_like_psvr(NSScreen *s) {
    if ([s.localizedName containsString:@"SCEI"] ||
        [s.localizedName containsString:@"VRH"] ||
        [s.localizedName containsString:@"PlayStation"])
        return YES;
    return NO;
}

static NSScreen *pick_psvr_screen(void) {
    // Skip any display that's mirroring another - rendering on a mirroring
    // display also shows up on its source, causing duplicated content on the
    // MacBook. The PSVR itself is never identified as "main" unless the
    // MacBook's built-in display is closed, so we can't use main-ness as a
    // filter; rely on the name match instead.
    for (NSScreen *s in NSScreen.screens) {
        if (screen_in_mirror_set(s)) continue;
        if (screen_looks_like_psvr(s)) return s;
    }
    // Fallback: any non-main 1920x1080 external display that isn't mirroring.
    NSScreen *main = NSScreen.mainScreen;
    for (NSScreen *s in NSScreen.screens) {
        if (s == main) continue;
        if (screen_in_mirror_set(s)) continue;
        NSInteger w = (NSInteger)s.frame.size.width;
        NSInteger h = (NSInteger)s.frame.size.height;
        if (w == 1920 && h == 1080) return s;
    }
    return nil;
}

- (instancetype)init {
    if ((self = [super init])) {
        _captureQueue = dispatch_queue_create(
            "opentrack.psvr.mirror.capture", DISPATCH_QUEUE_SERIAL);
        _enabled = NO;
        [NSNotificationCenter.defaultCenter
            addObserver:self
               selector:@selector(screensDidChange:)
                   name:NSApplicationDidChangeScreenParametersNotification
                 object:nil];
    }
    return self;
}

- (void)dealloc {
    [NSNotificationCenter.defaultCenter removeObserver:self];
}

- (void)screensDidChange:(NSNotification *)n { (void)n; [self refresh]; }

- (void)enable {
    self.enabled = YES;
    [self refresh];
}

- (void)disable {
    self.enabled = NO;
    [self teardownWindowAndStream];
}

- (void)refresh {
    if (!self.enabled) return;
    NSScreen *target = pick_psvr_screen();
    if (target) {
        if (self.window) {
            if (!NSEqualRects(self.window.frame, target.frame)) {
                // PSVR moved to a different display slot: rebuild on the new one
                [self teardownWindowAndStream];
                [self startOnScreen:target];
            }
        } else {
            [self startOnScreen:target];
        }
    } else {
        // PSVR display gone - tear down so the mirror doesn't land on MacBook screen
        [self teardownWindowAndStream];
    }
}

- (void)teardownWindowAndStream {
    if (self.stream) {
        [self.stream stopCaptureWithCompletionHandler:^(NSError * _Nullable err) { (void)err; }];
        self.stream = nil;
    }
    if (self.window) {
        [self.window orderOut:nil];
        self.window = nil;
    }
    self.leftLayer = nil;
    self.rightLayer = nil;
}

- (void)startOnScreen:(NSScreen *)screen {
    NSRect frame = screen.frame;
    self.window = [[NSWindow alloc]
        initWithContentRect:NSMakeRect(0, 0, frame.size.width, frame.size.height)
                  styleMask:NSWindowStyleMaskBorderless
                    backing:NSBackingStoreBuffered
                      defer:NO
                     screen:screen];
    [self.window setFrameOrigin:frame.origin];
    [self.window setFrame:frame display:YES];
    self.window.level = NSFloatingWindowLevel;
    self.window.opaque = YES;
    self.window.backgroundColor = NSColor.blackColor;
    self.window.collectionBehavior = NSWindowCollectionBehaviorCanJoinAllSpaces
                                   | NSWindowCollectionBehaviorFullScreenAuxiliary
                                   | NSWindowCollectionBehaviorStationary;
    self.window.ignoresMouseEvents = YES;

    CALayer *root = [CALayer layer];
    root.frame = NSMakeRect(0, 0, frame.size.width, frame.size.height);
    root.backgroundColor = NSColor.blackColor.CGColor;

    CGFloat halfW = frame.size.width / 2.0;
    self.leftLayer  = [CALayer layer];
    self.rightLayer = [CALayer layer];
    self.leftLayer.frame  = NSMakeRect(0,     0, halfW, frame.size.height);
    self.rightLayer.frame = NSMakeRect(halfW, 0, halfW, frame.size.height);
    self.leftLayer.contentsGravity  = kCAGravityResize;
    self.rightLayer.contentsGravity = kCAGravityResize;
    [root addSublayer:self.leftLayer];
    [root addSublayer:self.rightLayer];

    NSView *v = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, frame.size.width, frame.size.height)];
    v.wantsLayer = YES;
    v.layer = root;
    self.window.contentView = v;
    [self.window makeKeyAndOrderFront:nil];

    [self startCapture];
}

- (void)startCapture {
    [SCShareableContent
        getShareableContentExcludingDesktopWindows:NO
                          onScreenWindowsOnly:YES
                            completionHandler:^(SCShareableContent * _Nullable content,
                                                NSError * _Nullable error) {
        if (error || !content) {
            NSLog(@"PSVR mirror: getShareableContent failed: %@", error);
            return;
        }
        // Pick a source display that isn't the PSVR. We identify the
        // PSVR by its exact CGDirectDisplayID (via the NSScreen we
        // already chose as the window target), not by resolution —
        // the resolution heuristic would falsely exclude any OTHER
        // 1920x1080 display attached to the Mac (e.g., a 1080p
        // desktop monitor), leaving us with no source to mirror.
        CGDirectDisplayID psvrId = screen_display_id(self.window.screen);
        SCDisplay *source = nil;
        for (SCDisplay *d in content.displays) {
            if (d.displayID == psvrId) continue;
            source = d;
            break;
        }
        if (!source) {
            NSLog(@"PSVR mirror: only the PSVR is connected - nothing to mirror "
                  @"(open your MacBook lid or attach an external display)");
            return;
        }

        // Exclude our own mirror window from capture to avoid feedback loop
        NSMutableArray<SCWindow *> *excluded = [NSMutableArray array];
        for (SCWindow *w in content.windows) {
            if (w.owningApplication.processID == getpid()) [excluded addObject:w];
        }
        SCContentFilter *filter = [[SCContentFilter alloc]
            initWithDisplay:source excludingWindows:excluded];

        SCStreamConfiguration *cfg = [[SCStreamConfiguration alloc] init];
        cfg.width  = source.width;
        cfg.height = source.height;
        cfg.pixelFormat = kCVPixelFormatType_32BGRA;
        cfg.minimumFrameInterval = CMTimeMake(1, 60);
        cfg.queueDepth = 5;
        cfg.showsCursor = YES;

        NSError *err = nil;
        self.stream = [[SCStream alloc] initWithFilter:filter
                                          configuration:cfg
                                               delegate:self];
        [self.stream addStreamOutput:self
                                type:SCStreamOutputTypeScreen
                  sampleHandlerQueue:self.captureQueue
                               error:&err];
        if (err) { NSLog(@"PSVR mirror: addStreamOutput: %@", err); return; }

        [self.stream startCaptureWithCompletionHandler:^(NSError * _Nullable err2) {
            if (err2) NSLog(@"PSVR mirror: startCapture: %@ (grant Screen Recording permission)", err2);
        }];
    }];
}

- (void)stream:(SCStream *)stream
didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
        ofType:(SCStreamOutputType)type {
    // Invoked on self.captureQueue. ScreenCaptureKit already hands us
    // the captured frame as a CVPixelBuffer backed by an IOSurface; we
    // just hand the IOSurface straight to CoreAnimation as the two
    // layers' `contents`, which presents it to the PSVR screen without
    // any GPU copy or software conversion. The previous pipeline called
    // -[CIContext createCGImage:fromRect:] every frame, which re-rendered
    // the whole 1920x1080 BGRA buffer (~475 MB/s at 60 fps) just to turn
    // around and upload it again on the next CATransaction commit.
    if (type != SCStreamOutputTypeScreen) return;
    CVImageBufferRef px = CMSampleBufferGetImageBuffer(sampleBuffer);
    if (!px) return;
    IOSurfaceRef surface = CVPixelBufferGetIOSurface(px);
    if (!surface) return;

    // The IOSurface lives with the CMSampleBuffer, which Foundation will
    // drop when this callback returns. Retain it so the block below can
    // assign it to a layer on the main queue; balanced by a release
    // there.
    CFRetain(surface);
    dispatch_async(dispatch_get_main_queue(), ^{
        [CATransaction begin];
        [CATransaction setDisableActions:YES];
        self.leftLayer.contents  = (__bridge id)surface;
        self.rightLayer.contents = (__bridge id)surface;
        [CATransaction commit];
        CFRelease(surface);
    });
}

- (void)stream:(SCStream *)stream didStopWithError:(NSError *)error {
    NSLog(@"PSVR mirror: stream stopped: %@", error);
}

@end

// Global instance (one mirror per process)
static PSVRMirrorImpl *g_mirror = nil;

extern "C" void psvr_mirror_start(void) {
    dispatch_async(dispatch_get_main_queue(), ^{
        if (!g_mirror)
            g_mirror = [[PSVRMirrorImpl alloc] init];
        [g_mirror enable];
    });
}

extern "C" void psvr_mirror_stop(void) {
    dispatch_async(dispatch_get_main_queue(), ^{
        if (!g_mirror) return;
        [g_mirror disable];
    });
}
