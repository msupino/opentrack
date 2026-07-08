/* Copyright (c) 2026, Adrian Lopez <adrianlopezroche@gmail.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */
#pragma once

#include "ui_papertracker.h"
#include "papertracker-settings.h"
#include "api/plugin-api.hpp"
#include <QTimer>

class PaperTracker;

class PaperTrackerDialog : public ITrackerDialog
{
    Q_OBJECT

public:
    PaperTrackerDialog();
    void register_tracker(ITracker *) override;
    void unregister_tracker() override;
    bool embeddable() noexcept override { return true; }
    void set_buttons_visible(bool x) override;
    void save() override;
    void reload() override;
private:
    enum tracker_status {
        STOPPED,
        STARTING,
        RUNNING,
        RESTART_REQUIRED
    };

    papertracker_settings s;
    Ui::papertracker_dialog ui;
    PaperTracker *tracker;
    QTimer timer;
private slots:
    void doOK();
    void doCancel();
    void doOpenCameraSettings();
    void doShowHelp();
    void doUpdateStatus();
    void setStatusLabel(tracker_status status);
};
