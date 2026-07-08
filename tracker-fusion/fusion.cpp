/* Copyright (c) 2017 Stanislaw Halik <sthalik@misaki.pl>

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#undef NDEBUG

#include "fusion.h"
#include "compat/library-path.hpp"

#include <QDebug>
#include <QMessageBox>
#include <QApplication>
#include <QSplitter>
#include <QVBoxLayout>
#include <cassert>

static const char* own_name = "fusion";

static auto get_modules()
{
    return Modules(OPENTRACK_BASE_PATH + OPENTRACK_LIBRARY_PATH, dylib_load_quiet);
}

fusion_tracker::fusion_tracker() = default;

fusion_tracker::~fusion_tracker()
{
    // CAVEAT order matters
    rot_tracker = nullptr;
    pos_tracker = nullptr;

    rot_dylib = nullptr;
    pos_dylib = nullptr;
}

const QString& fusion_tracker::caption()
{
    static const QString caption = tr("Fusion tracker");
    return caption;
}

module_status fusion_tracker::start_tracker(QFrame* frame)
{
    assert(!rot_tracker && !pos_tracker);
    assert(!rot_dylib && !pos_dylib);

    QString err;
    module_status status;

    fusion_settings s;
    const QString rot_tracker_name = s.rot_tracker_name().toString();
    const QString pos_tracker_name = s.pos_tracker_name().toString();

    assert(rot_tracker_name != own_name);
    assert(pos_tracker_name != own_name);

    if (rot_tracker_name.isEmpty() || pos_tracker_name.isEmpty())
    {
        err = tr("Trackers not selected.");
        goto end;
    }

    if (rot_tracker_name == pos_tracker_name)
    {
        err = tr("Select different trackers for rotation and position.");
        goto end;
    }

    {
        Modules libs = get_modules();

        for (auto& t : libs.trackers())
        {
            if (t->module_name == rot_tracker_name)
            {
                assert(!rot_dylib);
                rot_dylib = t;
            }

            if (t->module_name == pos_tracker_name)
            {
                assert(!pos_dylib);
                pos_dylib = t;
            }
        }
    }

    if (!rot_dylib || !pos_dylib)
        goto end;

    rot_tracker = make_dylib_instance<ITracker>(rot_dylib);
    pos_tracker = make_dylib_instance<ITracker>(pos_dylib);

    // Start the position tracker on a DETACHED probe QFrame first. We
    // don't yet know whether it installs any UI (camera preview, etc.)
    // or is a silent tracker (UDP, joystick, ...). If it's silent we
    // hand the whole `frame` to the rotation tracker, same as the
    // legacy single-slot behavior. If it DOES install UI, we wrap
    // both sub-trackers' frames in a vertical QSplitter so both
    // status widgets - e.g. a camera preview AND PSVR's calibration
    // banner - are visible at once. Previously the rotation tracker's
    // UI was forced onto a permanently-hidden QFrame, which made it
    // impossible to tell what state the rotation tracker was in once
    // Fusion got involved.
    {
        QFrame* pos_probe = new QFrame();
        pos_probe->setVisible(false);
        status = pos_tracker->start_tracker(pos_probe);
        if (!status.is_ok())
        {
            delete pos_probe;
            err = pos_dylib->name + QStringLiteral(":\n    ") + status.error;
            goto end;
        }

        if (pos_probe->layout() == nullptr)
        {
            // position tracker installed no UI; give rotation tracker
            // the full preview frame.
            delete pos_probe;
            status = rot_tracker->start_tracker(frame);
            if (!status.is_ok())
            {
                err = rot_dylib->name + QStringLiteral(":\n    ") + status.error;
                goto end;
            }
        }
        else
        {
            // Wrap position-tracker UI (pos_probe) + a fresh bottom
            // frame into a vertical splitter inside `frame`. Stretch
            // ratio 3:1 gives the camera preview most of the space
            // while leaving the rotation tracker enough room for a
            // calibration banner + re-calibrate button.
            QVBoxLayout* outer   = new QVBoxLayout(frame);
            QSplitter*   split   = new QSplitter(Qt::Vertical, frame);
            QFrame*      bottom  = new QFrame();
            outer->setContentsMargins(0, 0, 0, 0);
            outer->addWidget(split);
            split->addWidget(pos_probe);   // reparents into the splitter
            split->addWidget(bottom);
            split->setStretchFactor(0, 3);
            split->setStretchFactor(1, 1);
            pos_probe->setVisible(true);

            status = rot_tracker->start_tracker(bottom);
            if (!status.is_ok())
            {
                err = rot_dylib->name + QStringLiteral(":\n    ") + status.error;
                goto end;
            }
        }
    }

end:
    if (!err.isEmpty())
        return error(err);
    else
        return status_ok();
}

void fusion_tracker::data(double *data)
{
    if (pos_tracker && rot_tracker)
    {
        rot_tracker->data(rot_tracker_data);
        pos_tracker->data(pos_tracker_data);

        for (unsigned k = 0; k < 3; k++)
            data[k] = pos_tracker_data[k];
        for (unsigned k = 3; k < 6; k++)
            data[k] = rot_tracker_data[k];
    }
}

fusion_dialog::fusion_dialog()
{
    ui.setupUi(this);

    connect(ui.buttonBox, SIGNAL(accepted()), this, SLOT(doOK()));
    connect(ui.buttonBox, SIGNAL(rejected()), this, SLOT(doCancel()));

    ui.rot_tracker->addItem({});
    ui.pos_tracker->addItem({});

    Modules libs = get_modules();

    for (auto& m : libs.trackers())
    {
        if (m->module_name == own_name)
            continue;

        ui.rot_tracker->addItem(m->icon, m->name, QVariant(m->module_name));
        ui.pos_tracker->addItem(m->icon, m->name, QVariant(m->module_name));
    }

    ui.rot_tracker->setCurrentIndex(0);
    ui.pos_tracker->setCurrentIndex(0);

    tie_setting(s.rot_tracker_name, ui.rot_tracker);
    tie_setting(s.pos_tracker_name, ui.pos_tracker);
}

void fusion_dialog::doOK()
{
    const int rot_idx = ui.rot_tracker->currentIndex() - 1;
    const int pos_idx = ui.pos_tracker->currentIndex() - 1;

    if (rot_idx == -1 || pos_idx == -1 || rot_idx == pos_idx)
    {
        QMessageBox::warning(this,
                             fusion_tracker::caption(),
                             tr("Fusion tracker only works when distinct trackers are selected "
                                "for rotation and position."),
                             QMessageBox::Close);
    }

    s.b->save();
    close();
}

void fusion_dialog::doCancel()
{
    close();
}

fusion_settings::fusion_settings() :
    opts("fusion-tracker"),
    rot_tracker_name(b, "rot-tracker", ""),
    pos_tracker_name(b, "pos-tracker", "")
{
}

OPENTRACK_DECLARE_TRACKER(fusion_tracker, fusion_dialog, fusion_metadata)
