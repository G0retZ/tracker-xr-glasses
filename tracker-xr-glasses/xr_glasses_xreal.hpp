/* Copyright (c) 2026, Sergei 'G0retZ' Mitrofanov

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#pragma once

// XREAL One-series (One, One Pro, 1S) over the glasses' USB Ethernet link.
//
// The glasses expose a network interface; IMU reports stream on TCP port
// 52998 of 169.254.2.1 (some firmware: 169.254.1.1) without any request.
// Each frame is a 6-byte big-endian header (u16 command, i32 size) and a
// body; command 0x2836 with a 128-byte body is a sensor report whose
// little-endian layout holds i64 device time at 8, i32 type at 24 (11 = IMU),
// gyro rad/s at 28 and acceleration m/s^2 at 40. Orientation is fused on
// the host. Protocol details as documented by DroidDeck (GPL-3.0; facts only)
// and community readers; unverified on hardware here.

#include "xr_glasses_source.hpp"

#include <atomic>
#include <thread>

namespace xr_glasses {

class xreal_source : public pose_source
{
public:
    QString start(const device& dev, int rate) override;
    void stop() override;
    bool latest(double euler[3]) override { return store.get(euler); }
    int startup_timeout_ms() const override { return 5000; }
    QString describe() const override;

private:
    void reader_loop();
    bool read_fully(uint8_t* buf, size_t len);

    uintptr_t sock = ~uintptr_t(0);  // SOCKET
    std::thread reader;
    std::atomic<bool> stop_flag { false };
    pose_store store;
    QString host;
    std::atomic<uint64_t> reports { 0 };
};

} // namespace xr_glasses
