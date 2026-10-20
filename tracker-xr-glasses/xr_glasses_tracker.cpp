/* Copyright (c) 2026, Sergei 'G0retZ' Mitrofanov

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#include "xr_glasses_tracker.hpp"
#include "xr_glasses_log.hpp"

#include "api/plugin-api.hpp"

#include <QDir>
#include <QElapsedTimer>
#include <QStringList>
#include <QThread>

using namespace xr_glasses;

xr_glasses_tracker::xr_glasses_tracker() = default;

xr_glasses_tracker::~xr_glasses_tracker()
{
    if (source)
    {
        source->stop();
        source.reset();
    }
}

module_status xr_glasses_tracker::start_tracker(QFrame*)
{
    log_begin_session();
    const QString see_log = tr("\n\nLog: %1").arg(QDir::toNativeSeparators(log_path()));

    const QList<device> devices = enumerate_devices();
    if (devices.isEmpty())
        return error(tr("No supported glasses found.\n"
                        "Plug them in directly (not through an unpowered hub) and make sure "
                        "no other application is using them."));

    const device* chosen = nullptr;
    if (const int key = s.device_key; key != 0)
    {
        for (const device& d : devices)
            if (d.key() == key)
                chosen = &d;
        if (!chosen)
            return error(tr("The selected glasses are not connected."));
    }
    else
        chosen = &devices.front();

    label = chosen->label();
    QStringList found;
    for (const device& d : devices)
        found << d.label();
    xr_glasses::log(QStringLiteral("devices: %1; using %2").arg(found.join(QStringLiteral(", ")), label));

    QString err;
    source = make_source(*chosen, err);
    if (!source)
        return error(tr("%1: %2").arg(label, err));

    if (err = source->start(*chosen, s.imu_rate); !err.isEmpty())
    {
        xr_glasses::log(QStringLiteral("start failed: %1").arg(err));
        source->stop();
        source.reset();
        return error(tr("%1: %2").arg(label, err) + see_log);
    }

    // Wait for the first pose so a silent device becomes an error instead of
    // a frozen zero pose in opentrack.
    const int timeout_ms = source->startup_timeout_ms();
    QElapsedTimer t;
    t.start();
    double tmp[3];
    while (!source->latest(tmp))
    {
        if (t.elapsed() > timeout_ms)
        {
            xr_glasses::log(QStringLiteral("no pose within %1 ms").arg(timeout_ms));
            source->stop();
            source.reset();
            return error(tr("%1 opened but delivered no pose data within %2 s.\n\n"
                            "Make sure no other application is using the glasses, "
                            "then unplug and reconnect them and try again.")
                             .arg(label).arg(timeout_ms / 1000) + see_log);
        }
        QThread::msleep(10);
    }

    xr_glasses::log(QStringLiteral("streaming: %1, %2").arg(label, source->describe()));
    return status_ok();
}

void xr_glasses_tracker::data(double* d)
{
    double e[3] = { 0, 0, 0 };
    if (source)
        source->latest(e);

    d[TX] = 0;
    d[TY] = 0;
    d[TZ] = 0;
    // Sources publish NWU: +yaw = turn left, +pitch = look down, +roll = tilt
    // right. opentrack wants +pitch = look up, so pitch flips.
    d[Yaw] = e[2];
    d[Pitch] = -e[1];
    d[Roll] = e[0];
}

OPENTRACK_DECLARE_TRACKER(xr_glasses_tracker, xr_glasses_dialog, xr_glasses_metadata)
