/* Copyright (c) 2026, Sergei 'G0retZ' Mitrofanov

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#pragma once

#include <QString>

namespace xr_glasses {

// Append a line to the plugin's log file (and qDebug). The file is truncated
// on each tracker start so it tells the story of one session.
void log(const QString& line);

// Path of that log file, for error messages.
QString log_path();

// Truncate the log file and start a new session.
void log_begin_session();

} // namespace xr_glasses
