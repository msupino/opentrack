/* Copyright (c) 2026, Adrian Lopez <adrianlopezroche@gmail.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */
#pragma once

#include <unordered_map>

namespace papertracker {
    struct AngleCoverageBin {
        int pitch_index;
        int yaw_index;

        AngleCoverageBin(int pitch_index, int yaw_index);

        bool operator==(const AngleCoverageBin &other) const;
    };
}

namespace std {
    template <>
    struct hash<papertracker::AngleCoverageBin> {
        size_t operator()(const papertracker::AngleCoverageBin& b) const {
            size_t h1 = std::hash<int>{}(b.pitch_index);
            size_t h2 = std::hash<int>{}(b.yaw_index);
            return h1 ^ (h2 << 32 | h2 >> 32);
        }
    };
}

namespace papertracker {
    class AngleCoverageTracker {
    public:
        AngleCoverageTracker(double pitch_step, double yaw_step);

        void add_visit(const AngleCoverageBin &bin);
        void add_visit(double pitch, double yaw);
        void clear_visits(const AngleCoverageBin &bin);
        void clear_visits(double pitch, double yaw);
        int get_visit_count(const AngleCoverageBin &bin) const;
        int get_visit_count(double pitch, double yaw) const;
        AngleCoverageBin get_bin(double pitch, double yaw) const;

    private:
        double pitch_step;
        double yaw_step;

        std::unordered_map<AngleCoverageBin, int> visits;
    };
}