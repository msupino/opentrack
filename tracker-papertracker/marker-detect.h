/* Copyright (c) 2026, Adrian Lopez <adrianlopezroche@gmail.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */
#pragma once

#include <opencv2/core.hpp>

namespace papertracker {
    int detect_aruco_mip_36h12(const cv::Mat &in, int &nRotations);
    int detect_apriltag_36h11(const cv::Mat &in, int &nRotations);
}