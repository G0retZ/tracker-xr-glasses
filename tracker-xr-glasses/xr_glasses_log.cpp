/* Copyright (c) 2026, Sergei 'G0retZ' Mitrofanov

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#include "xr_glasses_log.hpp"

#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QMutex>
#include <QStandardPaths>

namespace xr_glasses {

static QMutex g_mutex;
static QFile g_file;

QString log_path()
{
    return QDir(QStandardPaths::writableLocation(QStandardPaths::TempLocation))
        .filePath(QStringLiteral("xr-glasses.log"));
}

void log_begin_session()
{
    QMutexLocker l(&g_mutex);
    if (g_file.isOpen())
        g_file.close();
    g_file.setFileName(log_path());
    g_file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
}

void log(const QString& line)
{
    qDebug().noquote() << "xr-glasses:" << line;
    QMutexLocker l(&g_mutex);
    if (!g_file.isOpen())
        return;
    g_file.write(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz ")).toUtf8());
    g_file.write(line.toUtf8());
    g_file.write("\n");
    g_file.flush();
}

} // namespace xr_glasses
