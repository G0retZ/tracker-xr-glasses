/* Copyright (c) 2026, Sergei 'G0retZ' Mitrofanov

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#include "xr_glasses_rayneo.hpp"
#include "xr_glasses_log.hpp"

#include <QElapsedTimer>
#include <QThread>

#include <chrono>
#include <cstring>

namespace xr_glasses {

namespace {
constexpr uint8_t PKT_MAGIC = 0x99;
constexpr uint8_t PKT_SENSOR = 0x65;
constexpr uint8_t PKT_RESPONSE = 0xC8;
constexpr uint8_t CMD_HEAD = 0x66;

constexpr uint8_t CMD_DEVICE_INFO = 0;
constexpr uint8_t CMD_OPEN_IMU = 1;
constexpr uint8_t CMD_CLOSE_IMU = 2;

constexpr int RESPONSE_CMD_OFFSET = 8;
constexpr int DEVINFO_BOARD_OFFSET = 21;
constexpr int ACCEL_OFFSET = 4;
constexpr int GYRO_OFFSET = 16;
constexpr int TICK_OFFSET = 40;
constexpr int SENSOR_MIN = 56;

constexpr int BOARD_AIR_4_PRO = 0x3A;
constexpr int BOARD_GT = 0x40;
constexpr int GT_CONTROL_INTERFACE = 5;
constexpr int PID_RAYNEO = 0xAF50;

constexpr int STASH_MAX = 4096;

float rdf32le(const uint8_t* p)
{
    float f;
    std::memcpy(&f, p, sizeof(f));
    return f;
}
uint32_t rd32le(const uint8_t* p) { return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24); }

int64_t now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
} // namespace

vec3 rayneo_source::map_axes(const vec3& v) const
{
    const vec3 in = gt_max_axes ? vec3{ -v.y, v.z, -v.x } : v;
    const float deg = gt_max_axes ? 35.0f : mount_deg;
    return quat::from_axis_angle({ 1, 0, 0 }, deg * 3.14159265f / 180.0f).rotate(in);
}

bool rayneo_source::command(uint8_t cmd)
{
    uint8_t pkt[64] = {};
    pkt[0] = CMD_HEAD;
    pkt[1] = cmd;
    pkt[2] = 0;
    const bool ok = hid.write(pkt, sizeof(pkt));
    log(QStringLiteral("rayneo: command %1 -> %2").arg(cmd).arg(ok ? QStringLiteral("sent") : QStringLiteral("failed")));
    return ok;
}

bool rayneo_source::wait_response(uint8_t cmd, int timeout_ms, std::vector<uint8_t>* out)
{
    QElapsedTimer t;
    t.start();
    while (t.elapsed() < timeout_ms)
    {
        if (last_response_cmd.load() == cmd)
        {
            if (out)
            {
                std::lock_guard<std::mutex> l(resp_mtx);
                *out = last_response;
            }
            return true;
        }
        QThread::msleep(5);
    }
    return false;
}

void rayneo_source::on_report(const uint8_t* data, size_t len)
{
    // Append to the stream and pull out every complete packet.
    if (len == 0)
        return;
    if (stash.size() + len > size_t(STASH_MAX))
        stash.clear();
    stash.insert(stash.end(), data, data + len);

    size_t off = 0;
    while (off + 4 <= stash.size())
    {
        if (stash[off] != PKT_MAGIC)
        {
            off++;
            continue;
        }
        const size_t plen = stash[off + 2];
        const uint8_t type = stash[off + 1];
        if (plen < 4 || plen > 64 || (type != PKT_SENSOR && type != PKT_RESPONSE))
        {
            off++;
            continue;
        }
        if (off + plen > stash.size())
            break;
        on_packet(stash.data() + off, plen);
        off += plen;
    }
    if (off > 0)
        stash.erase(stash.begin(), stash.begin() + std::ptrdiff_t(off));
}

void rayneo_source::on_packet(const uint8_t* pkt, size_t len)
{
    if (pkt[1] == PKT_RESPONSE)
    {
        if (len <= size_t(RESPONSE_CMD_OFFSET))
            return;
        {
            std::lock_guard<std::mutex> l(resp_mtx);
            last_response.assign(pkt, pkt + len);
        }
        last_response_cmd.store(pkt[RESPONSE_CMD_OFFSET]);
        return;
    }
    if (pkt[1] != PKT_SENSOR || len < size_t(SENSOR_MIN))
        return;

    const vec3 accel_raw { rdf32le(pkt + ACCEL_OFFSET), rdf32le(pkt + ACCEL_OFFSET + 4), rdf32le(pkt + ACCEL_OFFSET + 8) };
    const vec3 gyro_dps { rdf32le(pkt + GYRO_OFFSET), rdf32le(pkt + GYRO_OFFSET + 4), rdf32le(pkt + GYRO_OFFSET + 8) };
    if (!accel_raw.finite() || !gyro_dps.finite())
        return;
    const uint32_t tick = rd32le(pkt + TICK_OFFSET);
    const int64_t wall = now_ns();

    float dt = 0.01f;
    if (have_tick && tick != last_tick)
        dt = float(double(uint32_t(tick - last_tick)) * 1e-4);
    else if (last_wall_ns > 0 && wall > last_wall_ns)
        dt = float(double(wall - last_wall_ns) * 1e-9);
    last_tick = tick;
    have_tick = true;
    last_wall_ns = wall;

    const vec3 accel = map_axes(accel_raw);
    const vec3 gyro = map_axes(gyro_dps) * (3.14159265f / 180.0f);

    quat q;
    if (!filter.update(accel, gyro, dt, q))
        return;
    samples.fetch_add(1, std::memory_order_relaxed);
    // Let the gravity alignment and bias estimate settle before publishing.
    if (wall < warmup_until_ns)
        return;
    store.set_nwu(rub_to_nwu(q));
}

QString rayneo_source::start(const device& dev, int)
{
    gt = dev.vendor_id == VID_RAYNEO_GT && dev.product_id == PID_RAYNEO;
    store.reset();
    stash.clear();
    last_response_cmd.store(-1);
    have_tick = false;
    last_wall_ns = 0;

    // Pick the interface that carries the protocol: 64-byte reports both
    // ways; on GT, interface 5 when present.
    QString chosen;
    QStringList candidates;
    for (const QString& path : dev.paths)
    {
        size_t in = 0, out = 0;
        if (!hid_report_sizes(path, in, out))
            continue;
        log(QStringLiteral("rayneo: interface %1 (mi %2) in %3 out %4").arg(path).arg(hid_interface_number(path)).arg(in).arg(out));
        if (in < 65 || out < 65)
            continue;
        if (gt && hid_interface_number(path) == GT_CONTROL_INTERFACE)
        {
            chosen = path;
            break;
        }
        candidates << path;
    }
    if (chosen.isEmpty())
        chosen = gt ? (candidates.size() == 1 ? candidates.first() : QString()) : candidates.value(0);
    if (chosen.isEmpty())
        return QStringLiteral("no HID interface with 64-byte reports found on the glasses");

    if (const QString err = hid.open(chosen, [this](const uint8_t* d, size_t n) { on_report(d, n); }); !err.isEmpty())
        return err;

    std::vector<uint8_t> info;
    if (!command(CMD_DEVICE_INFO) || !wait_response(CMD_DEVICE_INFO, 2500, &info))
    {
        stop();
        return QStringLiteral("the glasses did not answer the device-info request");
    }
    board_id = info.size() > size_t(DEVINFO_BOARD_OFFSET) ? info[DEVINFO_BOARD_OFFSET] : -1;

    // IMU mounting correction per board, from DroidDeck's captures.
    if (gt && board_id == BOARD_GT)      { mount_deg = -25.0f; gt_max_axes = false; }
    else if (gt)                         { mount_deg = 35.0f;  gt_max_axes = true; }
    else if (board_id == BOARD_AIR_4_PRO){ mount_deg = -20.0f; gt_max_axes = false; }
    else                                 { mount_deg = 0.0f;   gt_max_axes = false; }
    log(QStringLiteral("rayneo: board 0x%1, gt=%2, mount %3 deg, gt-max axes=%4")
            .arg(board_id, 2, 16, QLatin1Char('0')).arg(gt).arg(mount_deg).arg(gt_max_axes));

    imu_filter::params p;
    p.accel_gain = 1.5f;
    p.gravity_tolerance = 1.25f;
    p.stationary_gyro = 0.18f;
    p.bias_rate = 0.5f;
    p.default_dt = 0.01f;
    p.reset_on_gap = false;
    filter = imu_filter(p);

    last_response_cmd.store(-1);
    if (!command(CMD_OPEN_IMU) || !wait_response(CMD_OPEN_IMU, 1500))
    {
        stop();
        return QStringLiteral("the glasses did not acknowledge the IMU start command");
    }
    warmup_until_ns = now_ns() + 1500 * 1000000LL;
    return {};
}

void rayneo_source::stop()
{
    if (hid.is_open())
    {
        command(CMD_CLOSE_IMU);
        hid.close();
    }
}

QString rayneo_source::describe() const
{
    return QStringLiteral("RayNeo %1 (board 0x%2, host fusion, unverified)")
        .arg(gt ? QStringLiteral("GT") : QStringLiteral("Air"))
        .arg(board_id, 2, 16, QLatin1Char('0'));
}

} // namespace xr_glasses
