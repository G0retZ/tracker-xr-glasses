/* Copyright (c) 2026, Sergei 'G0retZ' Mitrofanov

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#pragma once

// One pose source per glasses protocol. Every source ends up publishing the
// same thing: roll/pitch/yaw in degrees in the NWU frame (+roll = tilt right,
// +pitch = look down, +yaw = turn left), which the tracker maps to opentrack.

#include "xr_glasses_hid.hpp"
#include "xr_glasses_math.hpp"

#include <QMutex>
#include <QString>

#include <atomic>
#include <cstdint>
#include <memory>

namespace xr_glasses {

// Thread-safe holder for the latest orientation.
class pose_store
{
public:
    void set_nwu(const quat& q)
    {
        double e[3];
        nwu_to_euler_deg(q, e);
        set_euler(e);
    }
    void set_euler(const double e[3])
    {
        QMutexLocker l(&mtx);
        euler[0] = e[0];
        euler[1] = e[1];
        euler[2] = e[2];
        have = true;
        count.fetch_add(1, std::memory_order_relaxed);
    }
    bool get(double out[3]) const
    {
        QMutexLocker l(&mtx);
        if (!have)
            return false;
        out[0] = euler[0];
        out[1] = euler[1];
        out[2] = euler[2];
        return true;
    }
    void reset()
    {
        QMutexLocker l(&mtx);
        have = false;
    }
    uint64_t samples() const { return count.load(std::memory_order_relaxed); }

private:
    mutable QMutex mtx;
    double euler[3] = {};
    bool have = false;
    std::atomic<uint64_t> count { 0 };
};

struct pose_source
{
    virtual ~pose_source() = default;

    // Open the device and start streaming. `rate` is the IMU rate index
    // (0 = 60 Hz ... 5 = 1000 Hz) or -1 for the source's default; only some
    // protocols honour it. Returns an empty string on success.
    virtual QString start(const device& dev, int rate) = 0;
    virtual void stop() = 0;

    // Latest roll/pitch/yaw (degrees, NWU). False until the first pose.
    virtual bool latest(double euler[3]) = 0;

    // How long the tracker waits for the first pose before giving up.
    virtual int startup_timeout_ms() const { return 2000; }

    // Short human-readable description for the status line and log.
    virtual QString describe() const = 0;
};

// Pick the protocol for a device. Returns null with `error` set when the
// device is recognised but unsupported.
std::unique_ptr<pose_source> make_source(const device& dev, QString& error);

} // namespace xr_glasses
