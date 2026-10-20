/* Copyright (c) 2026, Sergei 'G0retZ' Mitrofanov

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#pragma once

// RayNeo Air series (Air 2/2s/3s/3s Pro/4 Pro) and GT series over HID.
//
// Commands are 64-byte reports [0x66, cmd, arg]; the glasses answer with
// packets starting 0x99: [0x99, type, length, ...] where type 0x65 is a
// sensor sample (little-endian floats: accel m/s^2 at 4, gyro deg/s at 16,
// tick in 100 us units at 40) and 0xC8 a command response (cmd at 8; the
// device-info response carries the board id at 21). Packets may be split
// across or packed within reports, so a byte stream is reassembled.
// Orientation is fused on the host, after a per-board IMU mounting
// correction. Protocol details as documented by DroidDeck (GPL-3.0; facts
// only), the RayNeo-Air-Series-3DoF-Scaffold (Apache-2.0) and Monado's
// RayNeo driver (BSL-1.0); unverified on hardware here.

#include "xr_glasses_source.hpp"

#include <atomic>
#include <mutex>
#include <vector>

namespace xr_glasses {

class rayneo_source : public pose_source
{
public:
    QString start(const device& dev, int rate) override;
    void stop() override;
    bool latest(double euler[3]) override { return store.get(euler); }
    int startup_timeout_ms() const override { return 8000; }
    QString describe() const override;

private:
    void on_report(const uint8_t* data, size_t len);
    void on_packet(const uint8_t* pkt, size_t len);
    bool command(uint8_t cmd);
    bool wait_response(uint8_t cmd, int timeout_ms, std::vector<uint8_t>* out = nullptr);
    vec3 map_axes(const vec3& v) const;

    hid_device hid;
    pose_store store;
    bool gt = false;
    int board_id = -1;
    float mount_deg = 0;      // IMU tilt about X, degrees
    bool gt_max_axes = false; // GT Max: permute axes before the tilt

    std::vector<uint8_t> stash;  // reassembly buffer, reader thread only

    std::mutex resp_mtx;
    std::vector<uint8_t> last_response;
    std::atomic<int> last_response_cmd { -1 };

    imu_filter filter;
    uint32_t last_tick = 0;
    bool have_tick = false;
    int64_t last_wall_ns = 0;
    int64_t warmup_until_ns = 0;
    std::atomic<uint64_t> samples { 0 };
};

} // namespace xr_glasses
