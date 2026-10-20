/* Copyright (c) 2026, Sergei 'G0retZ' Mitrofanov

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#pragma once

#include "ui_xr_glasses_dialog.h"
#include "xr_glasses_hid.hpp"
#include "xr_glasses_source.hpp"

#include "api/plugin-api.hpp"
#include "options/options.hpp"

#include <QString>

#include <memory>

using namespace options;

struct xr_glasses_settings : opts
{
    // xr_glasses::device::key() of the glasses to use; 0 = first device found.
    value<int> device_key;
    // IMU rate index (0 = 60 Hz ... 3 = 240 Hz); -1 = highest supported.
    // Honoured by VITURE glasses; others stream at their fixed rate.
    value<int> imu_rate;

    xr_glasses_settings() :
        opts("tracker-xr-glasses"),
        device_key(b, "device", 0),
        imu_rate(b, "imu-rate", -1)
    {}
};

class xr_glasses_tracker : public ITracker
{
    Q_DECLARE_TR_FUNCTIONS(xr_glasses_tracker)

public:
    xr_glasses_tracker();
    ~xr_glasses_tracker() override;

    module_status start_tracker(QFrame*) override;
    void data(double* data) override;

    // For the dialog's status line.
    QString device_label() const { return label; }
    QString describe() const { return source ? source->describe() : QString(); }

private:
    xr_glasses_settings s;
    std::unique_ptr<xr_glasses::pose_source> source;
    QString label;
};

class xr_glasses_dialog : public ITrackerDialog
{
    Q_OBJECT

public:
    xr_glasses_dialog();

    void register_tracker(ITracker* t) override;
    void unregister_tracker() override;

private slots:
    void doOK();
    void doCancel();
    void refresh();

private:
    void set_status(const QString& text);

    Ui::xr_glasses_dialog_ui ui;
    xr_glasses_settings s;
    xr_glasses_tracker* tracker = nullptr;
};

class xr_glasses_metadata : public Metadata
{
    Q_OBJECT

    QString name() override { return tr("XR Glasses (3DoF)"); }
    QIcon icon() override { return QIcon(); }
};
