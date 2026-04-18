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
endif()
