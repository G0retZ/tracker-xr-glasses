/* Copyright (c) 2026, Sergei 'G0retZ' Mitrofanov

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#pragma once

// VITURE glasses over HID. Two wire protocols exist:
//
//  * V2 (Gen2: Pro 2, Beast): one vendor HID interface, 64-byte reports,
//    8-byte header (preamble 0x0010, msg id, length, byte-sum checksum).
//    Pose arrives as a quaternion in event 0x7308. Documented by
//    https://github.com/elasticjava/viture-v2 (MIT); verified here on a Pro 2.
//
//  * V1 (Gen1: One, One Lite, Pro, Luma, Luma Pro, Luma Cyber): two HID
//    interfaces (command + IMU), 64-byte packets with a 0xFF/magic header
//    and CRC16-XMODEM, big-endian floats. Protocol details as documented by
//    DroidDeck's VITURE adapter (GPL-3.0; facts only, no code reused).
//    Unverified on hardware.
//
// Luma Ultra (Carina) uses USB bulk transfers and on-device 6DoF; not supported.

#include "xr_glasses_source.hpp"

#include <QElapsedTimer>

#include <atomic>
#include <vector>

namespace xr_glasses {

enum class viture_family { one, pro, luma, ultra, beast, pro2, unknown };
viture_family viture_family_for_pid(int pid);
const char* viture_family_name(viture_family f);

// Rate index -> Hz (the same index is used on the wire by both protocols).
int viture_rate_hz(int rate);
constexpr int VITURE_RATE_COUNT = 6;
constexpr int VITURE_RATE_240 = 3;

class viture_v2_source : public pose_source
{
public:
    QString start(const device& dev, int rate) override;
    void stop() override;
    bool latest(double euler[3]) override { return store.get(euler); }
    QString describe() const override;

private:
    void on_report(const uint8_t* data, size_t len);
    int set_imu(uint8_t streams, int rate, int timeout_ms);
    bool send(uint16_t msg, const uint8_t* payload, size_t len);

    hid_device hid;
    pose_store store;
    int product_id = 0;
    int rate_used = -1;
    QString firmware;
    std::atomic<int> ack_status { -1 };
    std::atomic<int> unknown_logged { 0 };
};

class viture_v1_source : public pose_source
{
public:
    QString start(const device& dev, int rate) override;
    void stop() override;
    bool latest(double euler[3]) override { return store.get(euler); }
    QString describe() const override;
    int startup_timeout_ms() const override { return 3000; }

private:
    void on_report(const uint8_t* data, size_t len);
    bool send(uint16_t msg, const uint8_t* payload, size_t len);

    hid_device mcu;   // command interface (has output reports)
    hid_device imu;   // separate IMU interface, if the device has one
    pose_store store;
    int rate_used = -1;
    std::atomic<int> replies_logged { 0 };
    QElapsedTimer last_send;
};

} // namespace xr_glasses
