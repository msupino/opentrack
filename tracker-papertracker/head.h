/* Copyright (c) 2026, Adrian Lopez <adrianlopezroche@gmail.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */
#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include "marker.h"

namespace papertracker {
    class Head {
    public:
        cv::Vec3d rvec;
        cv::Vec3d tvec;

        void set_handle_origin(const cv::Vec3d &origin);
        bool has_handle(int id);
        Marker &get_handle(int id);
        void set_handle(const Marker &handle);
        void update_handle(int id, const cv::Vec3d &rvec, const cv::Vec3d &tvec);
        size_t num_handles();
        std::pair<cv::Vec3d, cv::Vec3d> get_pose_from_handle_transform(int id, cv::Vec3d &rvec, cv::Vec3d &tvec);
        std::vector<int> get_expected_visible_ids(double max_angle);

    private:
        std::unordered_map<int, Marker> handles;
        cv::Vec3d handle_origin;
    };
}