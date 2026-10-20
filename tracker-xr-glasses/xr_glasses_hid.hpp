/* Copyright (c) 2026, Sergei 'G0retZ' Mitrofanov

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#pragma once

// Device discovery for every supported vendor, plus a small Win32 HID
// transport (overlapped ReadFile/WriteFile on a reader thread).

#include <QList>
#include <QString>
#include <QStringList>

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

namespace xr_glasses {

enum class vendor { viture, xreal, rayneo };

constexpr int VID_VITURE = 0x35CA;
constexpr int VID_XREAL = 0x3318;
constexpr int VID_RAYNEO = 0x1BBB;     // Air series
constexpr int VID_RAYNEO_GT = 0x3941;  // GT series
constexpr int VID_RAYNEO_ALT = 0x22D9;

struct device
{
    vendor vend = vendor::viture;
    int vendor_id = 0;
    int product_id = 0;
    QString name;        // USB product string, or a generic vendor label
    QStringList paths;   // HID interface paths (empty for XREAL, which is reached over TCP)

    // Stable identity for the settings file.
    int key() const { return (vendor_id << 16) | product_id; }
    QString label() const;  // "<name> (PID 0x1301)"
};

// Every connected device of a supported vendor. Does not open any device for
// data access, so it never conflicts with a running session.
QList<device> enumerate_devices();

// Report sizes (including the report-id byte) of a HID interface.
bool hid_report_sizes(const QString& path, size_t& in_len, size_t& out_len);

// Interface number from a composite device path ("&mi_XX"), or -1.
int hid_interface_number(const QString& path);

class hid_device
{
public:
    // Called on the reader thread with one input report, report-id byte stripped.
    using handler_t = std::function<void(const uint8_t* data, size_t len)>;

    hid_device() = default;
    ~hid_device();
    hid_device(const hid_device&) = delete;
    hid_device& operator=(const hid_device&) = delete;

    // Open the interface and start the reader thread. Empty string on success.
    QString open(const QString& path, handler_t on_report);
    void close();
    bool is_open() const;

    size_t input_len() const { return in_len; }    // incl. report id
    size_t output_len() const { return out_len; }  // incl. report id

    // Write one output report: `data` follows the (zero) report-id byte and is
    // zero-padded to the report length. Fails if it doesn't fit.
    bool write(const uint8_t* data, size_t len);

private:
    void reader_loop();

    void* handle = nullptr;
    size_t in_len = 0, out_len = 0;
    std::thread reader;
    std::atomic<bool> stop_flag { false };
    std::mutex write_mtx;
    handler_t handler;
};

} // namespace xr_glasses
