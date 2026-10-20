/* Copyright (c) 2026, Sergei 'G0retZ' Mitrofanov

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#include "xr_glasses_viture.hpp"
#include "xr_glasses_log.hpp"

#include <QThread>

#include <algorithm>
#include <cstring>

namespace xr_glasses {

// ---- shared -----------------------------------------------------------------

viture_family viture_family_for_pid(int pid)
{
    switch (pid)
    {
    case 0x1011: case 0x1013: case 0x1015: case 0x1017: case 0x101b: return viture_family::one;
    case 0x1019: case 0x101d:                                         return viture_family::pro;
    case 0x1121: case 0x1131: case 0x1141: case 0x1151:               return viture_family::luma;
    case 0x1101: case 0x1104:                                         return viture_family::ultra;
    case 0x1201: case 0x1211:                                         return viture_family::beast;
    case 0x1301:                                                      return viture_family::pro2;
    default:                                                          return viture_family::unknown;
    }
}

const char* viture_family_name(viture_family f)
{
    switch (f)
    {
    case viture_family::one:   return "One";
    case viture_family::pro:   return "Pro";
    case viture_family::luma:  return "Luma";
    case viture_family::ultra: return "Luma Ultra";
    case viture_family::beast: return "Beast";
    case viture_family::pro2:  return "Pro 2";
    default:                   return "unknown model";
    }
}

int viture_rate_hz(int rate)
{
    static constexpr int hz[VITURE_RATE_COUNT] = { 60, 90, 120, 240, 500, 1000 };
    return (rate >= 0 && rate < VITURE_RATE_COUNT) ? hz[rate] : 0;
}

static uint16_t rd16le(const uint8_t* p) { return uint16_t(p[0] | (p[1] << 8)); }
static float rdf32le(const uint8_t* p)
{
    float f;
    std::memcpy(&f, p, sizeof(f));
    return f;
}
static float rdf32be(const uint8_t* p)
{
    const uint32_t u = (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
    float f;
    std::memcpy(&f, &u, sizeof(f));
    return f;
}

// ---- protocol V2 ------------------------------------------------------------

namespace v2 {
constexpr uint16_t PREAMBLE = 0x0010;
constexpr uint16_t MSG_IMU_CONTROL = 0x0301;  // payload [streams, rate]
constexpr uint16_t MSG_IMU_ACK = 0x2301;      // payload [status]
constexpr uint16_t MSG_FIRMWARE = 0x3003;     // reply 0x5003: [status, ASCII]
constexpr uint16_t MSG_FIRMWARE_REPLY = 0x5003;
constexpr uint16_t EVT_POSE = 0x7308;         // u32, u32 tick, f32 qw qx qy qz
constexpr uint16_t EVT_RAW = 0x7309;
constexpr uint8_t STREAM_OFF = 0x00;
constexpr uint8_t STREAM_POSE = 0x01;
constexpr int HEADER = 8;
constexpr int FRAME_MAX = 64;

static std::vector<uint8_t> build(uint16_t msg, const uint8_t* payload, size_t len)
{
    std::vector<uint8_t> f(HEADER + len);
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++)
        sum = uint16_t(sum + payload[i]);
    f[0] = uint8_t(PREAMBLE & 0xff); f[1] = uint8_t(PREAMBLE >> 8);
    f[2] = uint8_t(msg & 0xff);      f[3] = uint8_t(msg >> 8);
    f[4] = uint8_t(len & 0xff);      f[5] = uint8_t(len >> 8);
    f[6] = uint8_t(sum & 0xff);      f[7] = uint8_t(sum >> 8);
    if (len)
        std::memcpy(f.data() + HEADER, payload, len);
    return f;
}
} // namespace v2

// Axis frame of the Gen2 pose quaternion. Observed on a Pro 2: component x
// is rotation about the glasses' LEFT axis, y about UP, z about FORWARD (a
// right-handed cyclic permutation of NWU). Other Gen2 models are assumed to
// match until someone verifies them.
static quat v2_to_nwu(int /*pid*/, float w, float x, float y, float z)
{
    return quat{ w, z, x, y };  // forward <- z, left <- x, up <- y
}

bool viture_v2_source::send(uint16_t msg, const uint8_t* payload, size_t len)
{
    const std::vector<uint8_t> frame = v2::build(msg, payload, len);
    return hid.write(frame.data(), frame.size());
}

int viture_v2_source::set_imu(uint8_t streams, int rate, int timeout_ms)
{
    ack_status.store(-1);
    const uint8_t payload[2] = { streams, uint8_t(rate) };
    log(QStringLiteral("v2: imu control streams=0x%1 rate=%2")
            .arg(streams, 2, 16, QLatin1Char('0')).arg(rate));
    if (!send(v2::MSG_IMU_CONTROL, payload, sizeof(payload)))
        return -1;
    QElapsedTimer t;
    t.start();
    while (t.elapsed() < timeout_ms)
    {
        const int st = ack_status.load();
        if (st >= 0)
            return st;
        QThread::msleep(5);
    }
    return -1;
}

void viture_v2_source::on_report(const uint8_t* d, size_t n)
{
    if (n < size_t(v2::HEADER) || rd16le(d) != v2::PREAMBLE)
        return;
    const uint16_t msg = rd16le(d + 2);
    const size_t len = rd16le(d + 4);
    if (v2::HEADER + len > n)
        return;
    const uint8_t* p = d + v2::HEADER;
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++)
        sum = uint16_t(sum + p[i]);
    if (sum != rd16le(d + 6))
        return;

    switch (msg)
    {
    case v2::EVT_POSE:
        if (len >= 24)
            store.set_nwu(v2_to_nwu(product_id, rdf32le(p + 8), rdf32le(p + 12), rdf32le(p + 16), rdf32le(p + 20)));
        break;
    case v2::MSG_IMU_ACK:
        if (len >= 1)
        {
            log(QStringLiteral("v2: imu control ack status=%1").arg(p[0]));
            ack_status.store(p[0]);
        }
        break;
    case v2::MSG_FIRMWARE_REPLY:
        if (len >= 2 && p[0] == 0)
        {
            firmware = QString::fromLatin1(reinterpret_cast<const char*>(p + 1), int(len - 1)).trimmed();
            log(QStringLiteral("v2: firmware %1").arg(firmware));
        }
        break;
    case v2::EVT_RAW:
        break;
    default:
        if (unknown_logged.fetch_add(1) < 20)
            log(QStringLiteral("v2: msg 0x%1 len %2").arg(msg, 4, 16, QLatin1Char('0')).arg(len));
        break;
    }
}

QString viture_v2_source::start(const device& dev, int rate)
{
    if (dev.paths.isEmpty())
        return QStringLiteral("no HID interface");
    product_id = dev.product_id;
    store.reset();

    if (const QString err = hid.open(dev.paths.first(), [this](const uint8_t* d, size_t n) { on_report(d, n); });
        !err.isEmpty())
        return err;
    if (hid.output_len() < size_t(v2::HEADER) + 1)
    {
        hid.close();
        return QStringLiteral("HID interface has no output reports; not a protocol V2 device");
    }

    send(v2::MSG_FIRMWARE, nullptr, 0);

    // Pose streaming on Gen2 tops out at 240 Hz; unsupported rates are either
    // rejected in the ACK or stay silent, so walk down until data flows.
    const int top = (rate >= 0 && rate < VITURE_RATE_COUNT) ? std::min(rate, VITURE_RATE_240) : VITURE_RATE_240;
    QString last;
    for (int r = top; r >= 0; r--)
    {
        const int st = set_imu(v2::STREAM_POSE, r, 500);
        if (st > 0)
        {
            last = QStringLiteral("%1 Hz rejected (status %2)").arg(viture_rate_hz(r)).arg(st);
            log(last);
            continue;
        }
        if (st < 0)
            log(QStringLiteral("v2: no acknowledgement for %1 Hz, waiting for data anyway").arg(viture_rate_hz(r)));

        QElapsedTimer t;
        t.start();
        double tmp[3];
        while (t.elapsed() < 1500)
        {
            if (store.get(tmp))
            {
                rate_used = r;
                log(QStringLiteral("v2: streaming at %1 Hz").arg(viture_rate_hz(r)));
                return {};
            }
            QThread::msleep(10);
        }
        last = QStringLiteral("%1 Hz accepted but no pose data arrived").arg(viture_rate_hz(r));
        log(last);
    }

    stop();
    return QStringLiteral("the glasses did not start the pose stream (%1)").arg(last);
}

void viture_v2_source::stop()
{
    if (hid.is_open())
    {
        const uint8_t off[2] = { v2::STREAM_OFF, 0 };
        send(v2::MSG_IMU_CONTROL, off, sizeof(off));
        hid.close();
    }
}

QString viture_v2_source::describe() const
{
    return QStringLiteral("protocol V2, %1 Hz%2")
        .arg(viture_rate_hz(rate_used))
        .arg(firmware.isEmpty() ? QString() : QStringLiteral(", firmware %1").arg(firmware));
}

// ---- protocol V1 ------------------------------------------------------------

namespace v1 {
constexpr int PACKET = 64;
constexpr int HEADER = 0x12;          // payload offset
constexpr int LEN_BASE = 0x0C;        // declared length = payload + LEN_BASE
constexpr uint8_t START = 0xFF;
constexpr uint8_t MAGIC_COMMAND = 0xFE;
constexpr uint8_t MAGIC_REPORT = 0xFD;
constexpr uint8_t MAGIC_IMU = 0xFC;
constexpr uint16_t MSG_IMU = 0x0015;       // payload [on]
constexpr uint16_t MSG_IMU_RATE = 0x0018;  // payload [rate index]
constexpr int COMMAND_GAP_MS = 60;

// CRC16-XMODEM: poly 0x1021, init 0, no reflection ("123456789" -> 0x31C3).
static uint16_t crc16(const uint8_t* data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= uint16_t(data[i]) << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
    }
    return crc;
}

// [0]=0xFF [1]=magic [2..3]=crc16 LE over [4, 4+len+2) [4..5]=len LE
// [14..15]=msg id LE [18..]=payload. len = payload size + 0x0C.
static std::vector<uint8_t> build(uint16_t msg, const uint8_t* payload, size_t n)
{
    std::vector<uint8_t> p(PACKET, 0);
    const size_t len = n + LEN_BASE;
    p[0] = START;
    p[1] = MAGIC_COMMAND;
    p[4] = uint8_t(len & 0xff);
    p[5] = uint8_t(len >> 8);
    p[14] = uint8_t(msg & 0xff);
    p[15] = uint8_t(msg >> 8);
    if (n)
        std::memcpy(p.data() + HEADER, payload, n);
    const uint16_t crc = crc16(p.data() + 4, len + 2);
    p[2] = uint8_t(crc & 0xff);
    p[3] = uint8_t(crc >> 8);
    return p;
}

struct frame
{
    uint8_t magic = 0;
    uint16_t msg = 0;
    const uint8_t* payload = nullptr;
    size_t len = 0;
    bool crc_ok = false;
};

static bool parse(const uint8_t* d, size_t n, frame& f)
{
    if (n < size_t(HEADER) || d[0] != START)
        return false;
    f.magic = d[1];
    if (f.magic != MAGIC_COMMAND && f.magic != MAGIC_REPORT && f.magic != MAGIC_IMU)
        return false;
    const size_t len = rd16le(d + 4);
    if (len < size_t(LEN_BASE) || len + 6 > n)
        return false;
    f.msg = rd16le(d + 14);
    f.payload = d + HEADER;
    f.len = len - LEN_BASE;
    f.crc_ok = crc16(d + 4, len + 2) == rd16le(d + 2);
    return true;
}
} // namespace v1

bool viture_v1_source::send(uint16_t msg, const uint8_t* payload, size_t len)
{
    // The firmware drops commands sent back to back; keep a gap between them.
    if (last_send.isValid())
    {
        const qint64 wait = v1::COMMAND_GAP_MS - last_send.elapsed();
        if (wait > 0)
            QThread::msleep(static_cast<unsigned long>(wait));
    }
    const std::vector<uint8_t> p = v1::build(msg, payload, len);
    const bool ok = mcu.write(p.data(), p.size());
    last_send.start();
    log(QStringLiteral("v1: send 0x%1 -> %2").arg(msg, 4, 16, QLatin1Char('0')).arg(ok ? QStringLiteral("ok") : QStringLiteral("failed")));
    return ok;
}

void viture_v1_source::on_report(const uint8_t* d, size_t n)
{
    v1::frame f;
    if (!v1::parse(d, n, f) || !f.crc_ok)
        return;

    if (f.magic != v1::MAGIC_IMU)
    {
        if (replies_logged.fetch_add(1) < 20)
            log(QStringLiteral("v1: reply magic 0x%1 msg 0x%2 len %3")
                    .arg(f.magic, 2, 16, QLatin1Char('0')).arg(f.msg, 4, 16, QLatin1Char('0')).arg(f.len));
        return;
    }

    // IMU payload (big-endian floats): roll, pitch, yaw in degrees at 0/4/8;
    // newer firmware adds a unit quaternion w,x,y,z at 20..32. Prefer the
    // quaternion. Frame assumed to be NWU (unverified on hardware).
    if (f.len >= 36)
    {
        const quat q { rdf32be(f.payload + 20), rdf32be(f.payload + 24), rdf32be(f.payload + 28), rdf32be(f.payload + 32) };
        const float nrm = q.norm();
        if (q.finite() && nrm > 0.9f && nrm < 1.1f)
        {
            store.set_nwu(q.normalized());
            return;
        }
    }
    if (f.len >= 12)
    {
        const double e[3] = { rdf32be(f.payload), rdf32be(f.payload + 4), rdf32be(f.payload + 8) };
        if (std::isfinite(e[0]) && std::isfinite(e[1]) && std::isfinite(e[2]))
            store.set_euler(e);
    }
}

QString viture_v1_source::start(const device& dev, int rate)
{
    store.reset();

    // The command interface is the one with output reports; a second
    // interface, if present, carries the IMU stream.
    QString mcu_path, imu_path;
    for (const QString& path : dev.paths)
    {
        size_t in = 0, out = 0;
        if (!hid_report_sizes(path, in, out))
            continue;
        log(QStringLiteral("v1: interface %1 in %2 out %3").arg(path).arg(in).arg(out));
        if (out > 1)
            mcu_path = path;
    }
    for (const QString& path : dev.paths)
        if (path != mcu_path && imu_path.isEmpty())
            imu_path = path;
    if (mcu_path.isEmpty())
        return QStringLiteral("no HID interface with output reports; not a protocol V1 device");

    auto handler = [this](const uint8_t* d, size_t n) { on_report(d, n); };
    if (const QString err = mcu.open(mcu_path, handler); !err.isEmpty())
        return err;
    if (mcu.output_len() < size_t(v1::PACKET) + 1)
    {
        mcu.close();
        return QStringLiteral("unexpected output report size %1; not a protocol V1 device").arg(mcu.output_len());
    }
    if (!imu_path.isEmpty())
        if (const QString err = imu.open(imu_path, handler); !err.isEmpty())
            log(QStringLiteral("v1: IMU interface not opened: %1").arg(err));

    const uint8_t on = 1;
    if (!send(v1::MSG_IMU, &on, 1))
    {
        stop();
        return QStringLiteral("could not send the IMU enable command");
    }
    rate_used = (rate >= 0 && rate < VITURE_RATE_COUNT) ? std::min(rate, VITURE_RATE_240) : VITURE_RATE_240;
    const uint8_t r = uint8_t(rate_used);
    send(v1::MSG_IMU_RATE, &r, 1);
    return {};
}

void viture_v1_source::stop()
{
    if (mcu.is_open())
    {
        const uint8_t off = 0;
        send(v1::MSG_IMU, &off, 1);
    }
    imu.close();
    mcu.close();
}

QString viture_v1_source::describe() const
{
    return QStringLiteral("protocol V1 (unverified), %1 Hz requested").arg(viture_rate_hz(rate_used));
}

} // namespace xr_glasses
