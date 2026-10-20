/* Copyright (c) 2026, Sergei 'G0retZ' Mitrofanov

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#include "xr_glasses_xreal.hpp"
#include "xr_glasses_log.hpp"

#ifndef WIN32_LEAN_AND_MEAN
#   define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#   define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include <cstring>
#include <vector>

namespace xr_glasses {

namespace {
constexpr uint16_t REPORT_PORT = 52998;
constexpr uint16_t CMD_REPORT = 0x2836;
constexpr int REPORT_BODY = 128;
constexpr int TYPE_IMU = 11;
constexpr int MAX_BODY = 64 * 1024;
const char* const HOSTS[] = { "169.254.2.1", "169.254.1.1" };

int32_t rd32le(const uint8_t* p) { return int32_t(uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24)); }
int64_t rd64le(const uint8_t* p) { return int64_t(uint64_t(uint32_t(rd32le(p))) | (uint64_t(uint32_t(rd32le(p + 4))) << 32)); }
float rdf32le(const uint8_t* p)
{
    float f;
    std::memcpy(&f, p, sizeof(f));
    return f;
}

struct wsa_init
{
    bool ok = false;
    wsa_init()
    {
        WSADATA d;
        ok = WSAStartup(MAKEWORD(2, 2), &d) == 0;
    }
};

SOCKET connect_with_timeout(const char* host, uint16_t port, int timeout_ms)
{
    SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == INVALID_SOCKET)
        return INVALID_SOCKET;
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, host, &addr.sin_addr);

    u_long nonblocking = 1;
    ioctlsocket(s, FIONBIO, &nonblocking);
    connect(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));

    fd_set wset;
    FD_ZERO(&wset);
    FD_SET(s, &wset);
    timeval tv = { timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
    int err = 0;
    int errlen = sizeof(err);
    if (select(0, nullptr, &wset, nullptr, &tv) != 1 ||
        getsockopt(s, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&err), &errlen) != 0 || err != 0)
    {
        closesocket(s);
        return INVALID_SOCKET;
    }
    nonblocking = 0;
    ioctlsocket(s, FIONBIO, &nonblocking);
    DWORD rcv = 3000;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&rcv), sizeof(rcv));
    BOOL nodelay = TRUE;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&nodelay), sizeof(nodelay));
    return s;
}
} // namespace

QString xreal_source::start(const device&, int)
{
    static wsa_init wsa;
    if (!wsa.ok)
        return QStringLiteral("Winsock initialisation failed");

    store.reset();
    reports.store(0);

    SOCKET s = INVALID_SOCKET;
    for (const char* h : HOSTS)
    {
        s = connect_with_timeout(h, REPORT_PORT, 1500);
        if (s != INVALID_SOCKET)
        {
            host = QLatin1String(h);
            break;
        }
    }
    if (s == INVALID_SOCKET)
        return QStringLiteral("could not reach the glasses' IMU service at 169.254.2.1:%1 (or 169.254.1.1). "
                              "Check that Windows shows a USB Ethernet adapter for the glasses and that "
                              "Ethernet is not disabled in the glasses' developer menu. XREAL Air models "
                              "are not supported.")
            .arg(REPORT_PORT);

    sock = uintptr_t(s);
    stop_flag.store(false);
    reader = std::thread([this] { reader_loop(); });
    log(QStringLiteral("xreal: connected to %1:%2").arg(host).arg(REPORT_PORT));
    return {};
}

void xreal_source::stop()
{
    if (sock == ~uintptr_t(0))
        return;
    stop_flag.store(true);
    closesocket(SOCKET(sock));  // unblocks recv()
    if (reader.joinable())
        reader.join();
    sock = ~uintptr_t(0);
    log(QStringLiteral("xreal: disconnected after %1 reports").arg(reports.load()));
}

bool xreal_source::read_fully(uint8_t* buf, size_t len)
{
    size_t at = 0;
    while (at < len)
    {
        if (stop_flag.load())
            return false;
        const int n = recv(SOCKET(sock), reinterpret_cast<char*>(buf + at), int(len - at), 0);
        if (n <= 0)
            return false;
        at += size_t(n);
    }
    return true;
}

void xreal_source::reader_loop()
{
    imu_filter::params p;
    p.accel_gain = 1.5f;
    p.gravity_tolerance = 1.5f;
    p.stationary_gyro = 0.04f;
    p.bias_rate = 0.5f;
    p.min_dt = 0.0001f;
    p.max_dt = 0.05f;
    p.reset_on_gap = true;
    imu_filter filter(p);

    int64_t previous = 0;
    uint8_t header[6];
    std::vector<uint8_t> body;

    while (!stop_flag.load())
    {
        if (!read_fully(header, sizeof(header)))
        {
            if (!stop_flag.load())
                log(QStringLiteral("xreal: stream ended"));
            break;
        }
        const uint16_t cmd = uint16_t((header[0] << 8) | header[1]);
        const int32_t size = int32_t((uint32_t(header[2]) << 24) | (uint32_t(header[3]) << 16) | (uint32_t(header[4]) << 8) | header[5]);
        if (size < 0 || size > MAX_BODY)
        {
            log(QStringLiteral("xreal: bad frame length %1").arg(size));
            break;
        }
        body.resize(size_t(size));
        if (size > 0 && !read_fully(body.data(), body.size()))
            break;
        if (cmd != CMD_REPORT || size != REPORT_BODY || rd32le(body.data() + 24) != TYPE_IMU)
            continue;

        const int64_t t = rd64le(body.data() + 8);
        const vec3 gyro_raw { rdf32le(body.data() + 28), rdf32le(body.data() + 32), rdf32le(body.data() + 36) };
        const vec3 accel_raw { rdf32le(body.data() + 40), rdf32le(body.data() + 44), rdf32le(body.data() + 48) };
        if (t <= 0 || !gyro_raw.finite() || !accel_raw.finite())
            continue;
        if (t <= previous)
            continue;
        const float dt = previous == 0 ? 0.0f : float(double(t - previous) * 1e-9);
        previous = t;
        reports.fetch_add(1, std::memory_order_relaxed);

        // The 1S reports in a right/down/forward body frame; rotate 180 degrees
        // about X into right/up/back for the filter.
        const vec3 accel { accel_raw.x, -accel_raw.y, -accel_raw.z };
        const vec3 gyro { gyro_raw.x, -gyro_raw.y, -gyro_raw.z };

        quat q;
        if (filter.update(accel, gyro, dt, q))
            store.set_nwu(rub_to_nwu(q));
    }
}

QString xreal_source::describe() const
{
    return QStringLiteral("XREAL One-series over USB network (%1, unverified)").arg(host);
}

} // namespace xr_glasses
