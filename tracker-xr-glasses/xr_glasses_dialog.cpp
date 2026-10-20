/* Copyright (c) 2026, Sergei 'G0retZ' Mitrofanov

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#include "xr_glasses_tracker.hpp"
#include "xr_glasses_viture.hpp"

#include "api/plugin-api.hpp"

#include <QSignalBlocker>

using namespace xr_glasses;

// Combo boxes store the setting value in itemData, not the row index, so saved
// profiles survive devices being plugged in a different order.
//
// Mirrors options::tie_setting(value<int>&, QComboBox*) but keyed on itemData.
// (opentrack's generic 4-argument tie_setting template can't be instantiated
// for int: its slot takes `int&`, which the by-value signal can't bind.)
static void tie_combo(value<int>& v, QComboBox* cb)
{
    auto index_of = [cb](int x) { int i = cb->findData(x); return i < 0 ? 0 : i; };

    cb->setCurrentIndex(index_of(v));
    v = cb->currentData().toInt();

    QObject::connect(cb, &QComboBox::currentIndexChanged,
                     &v, [&v, cb](int idx) { v = cb->itemData(idx).toInt(); },
                     value<int>::DIRECT_CONNTYPE);
    QObject::connect(&v, value_::value_changed<int>(),
                     cb, [cb, index_of](int x) { cb->setCurrentIndex(index_of(x)); },
                     value<int>::SAFE_CONNTYPE);
}

// Replace a tied combo's items without the intermediate empty state being
// written back into the setting; the saved value is re-selected if still present.
template<typename F>
static void repopulate(QComboBox* cb, int saved, F&& fill)
{
    const QSignalBlocker block(cb);
    cb->clear();
    fill();
    const int i = cb->findData(saved);
    cb->setCurrentIndex(i < 0 ? 0 : i);
}

xr_glasses_dialog::xr_glasses_dialog()
{
    ui.setupUi(this);

    connect(ui.buttonBox, &QDialogButtonBox::accepted, this, &xr_glasses_dialog::doOK);
    connect(ui.buttonBox, &QDialogButtonBox::rejected, this, &xr_glasses_dialog::doCancel);
    connect(ui.refresh_button, &QPushButton::clicked, this, &xr_glasses_dialog::refresh);

    ui.rate_combo->addItem(tr("Auto (highest supported)"), -1);
    for (int r = 0; r <= VITURE_RATE_240; r++)
        ui.rate_combo->addItem(tr("%1 Hz").arg(viture_rate_hz(r)), r);

    refresh();

    tie_combo(s.device_key, ui.device_combo);
    tie_combo(s.imu_rate, ui.rate_combo);
}

void xr_glasses_dialog::refresh()
{
    const int saved = s.device_key;
    const QList<device> devices = enumerate_devices();

    repopulate(ui.device_combo, saved, [&] {
        ui.device_combo->addItem(tr("Auto (first found)"), 0);
        bool saved_present = saved == 0;
        for (const device& d : devices)
        {
            ui.device_combo->addItem(d.label(), d.key());
            saved_present |= d.key() == saved;
        }
        // Keep a previously chosen device selectable while it is unplugged.
        if (!saved_present)
            ui.device_combo->addItem(tr("Saved device (not connected, PID 0x%1)")
                                         .arg(QString::number(saved & 0xffff, 16).toUpper()),
                                     saved);
    });

    if (!tracker)
        set_status(devices.isEmpty() ? tr("No supported glasses found.") : tr("Not running."));
}

void xr_glasses_dialog::set_status(const QString& text)
{
    ui.status_label->setText(text);
}

void xr_glasses_dialog::register_tracker(ITracker* t)
{
    tracker = static_cast<xr_glasses_tracker*>(t);
    set_status(tr("Running: %1\n%2").arg(tracker->device_label(), tracker->describe()));
}

void xr_glasses_dialog::unregister_tracker()
{
    tracker = nullptr;
    set_status(tr("Not running."));
}

void xr_glasses_dialog::doOK()
{
    s.b->save();
    close();
}

void xr_glasses_dialog::doCancel()
{
    close();
}
