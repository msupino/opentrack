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
