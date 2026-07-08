if(APPLE)
    otr_module(tracker-psvr)

    # Add the Objective-C++ SBS mirror source manually
    # (otr_glob_sources doesn't pick up .mm files)
    target_sources(${self} PRIVATE psvr_mirror.mm)
    set_source_files_properties(psvr_mirror.mm PROPERTIES
        COMPILE_FLAGS "-fobjc-arc")

    target_link_libraries(${self} PRIVATE
        "-framework IOKit"
        "-framework CoreFoundation"
        "-framework AppKit"
        "-framework ScreenCaptureKit"
        "-framework CoreMedia"
        "-framework CoreImage"
        "-framework CoreVideo"
        "-framework QuartzCore")

    # PS4 Camera (OV580) firmware uploader. Always built into the
    # plugin: this is part of what tracker-psvr is for - users who
    # plug in a PS4 Camera via the PS4-to-PS5 adapter need the
    # firmware blob shipped to the device before AVFoundation can
    # see it as a webcam.
    #
    # libusb is the only external dependency. Located via the repo's
    # opentrack-pkg-config helper (which itself wraps FindPkgConfig);
    # the helper is normally only included on Linux by
    # cmake/opentrack-platform.cmake, but pkg-config works just as
    # well on macOS for finding Homebrew's libusb, so we include it
    # locally here without polluting other Apple-only modules.
    include(opentrack-pkg-config)
    pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
    target_sources(${self} PRIVATE
        ps4cam_firmware.mm
        ps4cam_firmware.h
        ps4cam_firmware_blob.h)
    set_source_files_properties(ps4cam_firmware.mm PROPERTIES
        COMPILE_FLAGS "-fobjc-arc")
    target_include_directories(${self} SYSTEM PRIVATE ${LIBUSB_INCLUDE_DIRS})
    target_link_libraries(${self} PRIVATE ${LIBUSB_LIBRARIES})
    target_link_directories(${self} PRIVATE ${LIBUSB_LIBRARY_DIRS})

    # Optional: LED-constellation position tracking via webcam.
    #
    # The PSVR lights up 9 blue LEDs on its front visor + rear strap when
    # VR mode is enabled (which our 0x23 activation command does). With a
    # regular webcam pointed at the user's face we can extract those blobs
    # and solve PnP against the known 9-LED geometry to recover head
    # position - the same trick the PS4 camera did, but done entirely in
    # our plugin so the user just ticks one checkbox instead of wiring up
    # opentrack's generic PointTracker in Fusion.
    #
    # Gated on OpenCV being available at configure time. Other plugins
    # (tracker-pt, tracker-aruco, tracker-neuralnet) already pull in
    # opencv_core/imgproc/calib3d + the bundled copies land in
    # install/opentrack.app/Contents/Frameworks, so we link against
    # those same copies here. If OpenCV isn't found, the plugin still
    # builds with IMU-only rotation (legacy behavior).
    include(opentrack-opencv)
    find_package(OpenCV QUIET)
    if(OpenCV_FOUND)
        target_sources(${self} PRIVATE
            psvr_camera.mm
            psvr_constellation.cpp)
        set_source_files_properties(psvr_camera.mm PROPERTIES
            COMPILE_FLAGS "-fobjc-arc")
        target_link_libraries(${self} PRIVATE
            opencv_core opencv_imgproc opencv_calib3d
            "-framework AVFoundation")
        target_compile_definitions(${self} PRIVATE PSVR_HAS_CAMERA=1)
    endif()
endif()
