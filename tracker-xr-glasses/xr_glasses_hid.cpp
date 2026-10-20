/* Copyright (c) 2026, Sergei 'G0retZ' Mitrofanov

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#include "xr_glasses_hid.hpp"
#include "xr_glasses_log.hpp"

#include <QRegularExpression>

#ifndef WIN32_LEAN_AND_MEAN
#   define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#   define NOMINMAX
#endif
#include <windows.h>
#include <setupapi.h>
#include <hidsdi.h>

#include <cstring>
#include <vector>

namespace xr_glasses {

QString device::label() const
{
    return QStringLiteral("%1 (PID 0x%2)")
        .arg(name, QString::number(product_id, 16).rightJustified(4, QLatin1Char('0')).toUpper());
}

// ---- helpers ----------------------------------------------------------------

static QString last_error_text()
{
    const DWORD e = GetLastError();
    wchar_t* buf = nullptr;
    FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                   nullptr, e, 0, reinterpret_cast<wchar_t*>(&buf), 0, nullptr);
    QString s = buf ? QString::fromWCharArray(buf).trimmed() : QString();
    if (buf)
        LocalFree(buf);
    return QStringLiteral("%1 (error %2)").arg(s).arg(e);
}

static bool parse_ids(const QString& path, int& vid, int& pid)
{
    static const QRegularExpression re(QStringLiteral("vid_([0-9a-f]{4})&pid_([0-9a-f]{4})"),
                                       QRegularExpression::CaseInsensitiveOption);
    const auto m = re.match(path);
    if (!m.hasMatch())
        return false;
    vid = m.captured(1).toInt(nullptr, 16);
    pid = m.captured(2).toInt(nullptr, 16);
    return true;
}

int hid_interface_number(const QString& path)
{
    static const QRegularExpression re(QStringLiteral("&mi_([0-9a-f]{2})"), QRegularExpression::CaseInsensitiveOption);
    const auto m = re.match(path);
    return m.hasMatch() ? m.captured(1).toInt(nullptr, 16) : -1;
}

static HANDLE open_query_handle(const QString& path)
{
    return CreateFileW(reinterpret_cast<const wchar_t*>(path.utf16()), 0,
                       FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
}

static QString product_string(const QString& path)
{
    HANDLE h = open_query_handle(path);
    if (h == INVALID_HANDLE_VALUE)
        return {};
    wchar_t buf[128] = {};
    QString s;
    if (HidD_GetProductString(h, buf, sizeof(buf)))
        s = QString::fromWCharArray(buf).trimmed();
    CloseHandle(h);
    return s;
}

bool hid_report_sizes(const QString& path, size_t& in_len, size_t& out_len)
{
    HANDLE h = open_query_handle(path);
    if (h == INVALID_HANDLE_VALUE)
        return false;
    PHIDP_PREPARSED_DATA pp = nullptr;
    HIDP_CAPS caps = {};
    bool ok = HidD_GetPreparsedData(h, &pp) && HidP_GetCaps(pp, &caps) == HIDP_STATUS_SUCCESS;
    if (pp)
        HidD_FreePreparsedData(pp);
    CloseHandle(h);
    if (!ok)
        return false;
    in_len = caps.InputReportByteLength;
    out_len = caps.OutputReportByteLength;
    return true;
}

// Interface paths for one device-interface GUID.
static QStringList interface_paths(const GUID& guid)
{
    QStringList out;
    HDEVINFO set = SetupDiGetClassDevsW(&guid, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (set == INVALID_HANDLE_VALUE)
        return out;

    SP_DEVICE_INTERFACE_DATA iface = {};
    iface.cbSize = sizeof(iface);
    for (DWORD i = 0; SetupDiEnumDeviceInterfaces(set, nullptr, &guid, i, &iface); i++)
    {
        DWORD needed = 0;
        SetupDiGetDeviceInterfaceDetailW(set, &iface, nullptr, 0, &needed, nullptr);
        if (needed == 0)
            continue;
        std::vector<BYTE> buf(needed);
        auto* detail = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W*>(buf.data());
        detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
        if (SetupDiGetDeviceInterfaceDetailW(set, &iface, detail, needed, nullptr, nullptr))
            out << QString::fromWCharArray(detail->DevicePath);
    }
    SetupDiDestroyDeviceInfoList(set);
    return out;
}

// ---- enumeration ------------------------------------------------------------

QList<device> enumerate_devices()
{
    QList<device> out;
    auto find = [&](int vid, int pid) -> device* {
        for (device& d : out)
            if (d.vendor_id == vid && d.product_id == pid)
                return &d;
        return nullptr;
    };

    // HID vendors: VITURE and RayNeo. Interfaces of one composite device are
    // grouped by vendor/product id.
    GUID hid_guid;
    HidD_GetHidGuid(&hid_guid);
    for (const QString& path : interface_paths(hid_guid))
    {
        int vid = 0, pid = 0;
        if (!parse_ids(path, vid, pid))
            continue;
        vendor v;
        if (vid == VID_VITURE)
            v = vendor::viture;
        else if (vid == VID_RAYNEO || vid == VID_RAYNEO_GT || vid == VID_RAYNEO_ALT)
            v = vendor::rayneo;
        else
            continue;

        device* d = find(vid, pid);
        if (!d)
        {
            out.push_back(device{ v, vid, pid, QString(), {} });
            d = &out.last();
        }
        d->paths << path;
        if (d->name.isEmpty())
            d->name = product_string(path);
    }
    for (device& d : out)
        if (d.name.isEmpty())
            d.name = d.vend == vendor::viture ? QStringLiteral("VITURE glasses") : QStringLiteral("RayNeo glasses");

    // XREAL One-series: reached over the glasses' USB Ethernet link, so only
    // USB presence is checked here.
    static const GUID usb_device_guid =
        { 0xA5DCBF10, 0x6530, 0x11D2, { 0x90, 0x1F, 0x00, 0xC0, 0x4F, 0xB9, 0x51, 0xED } };
    for (const QString& path : interface_paths(usb_device_guid))
    {
        int vid = 0, pid = 0;
        if (!parse_ids(path, vid, pid) || vid != VID_XREAL || find(vid, pid))
            continue;
        out.push_back(device{ vendor::xreal, vid, pid, QStringLiteral("XREAL glasses (USB network)"), {} });
    }

    return out;
}

// ---- transport --------------------------------------------------------------

hid_device::~hid_device()
{
    close();
}

bool hid_device::is_open() const
{
    return handle != nullptr && handle != INVALID_HANDLE_VALUE;
}

QString hid_device::open(const QString& path, handler_t on_report)
{
    close();

    HANDLE h = CreateFileW(reinterpret_cast<const wchar_t*>(path.utf16()),
                           GENERIC_READ | GENERIC_WRITE,
                           FILE_SHARE_READ | FILE_SHARE_WRITE,
                           nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
    if (h == INVALID_HANDLE_VALUE)
        return QStringLiteral("Could not open the HID interface: %1").arg(last_error_text());

    PHIDP_PREPARSED_DATA pp = nullptr;
    HIDP_CAPS caps = {};
    if (!HidD_GetPreparsedData(h, &pp) || HidP_GetCaps(pp, &caps) != HIDP_STATUS_SUCCESS)
    {
        const QString err = QStringLiteral("HID capabilities unavailable: %1").arg(last_error_text());
        if (pp)
            HidD_FreePreparsedData(pp);
        CloseHandle(h);
        return err;
    }
    HidD_FreePreparsedData(pp);

    in_len = caps.InputReportByteLength;
    out_len = caps.OutputReportByteLength;
    if (in_len < 2)
    {
        CloseHandle(h);
        return QStringLiteral("HID interface has no input reports");
    }

    handle = h;
    handler = std::move(on_report);
    stop_flag.store(false);
    reader = std::thread([this] { reader_loop(); });
    log(QStringLiteral("hid: opened %1 (reports in %2 / out %3 bytes)").arg(path).arg(in_len).arg(out_len));
    return {};
}

void hid_device::close()
{
    if (!is_open())
        return;
    stop_flag.store(true);
    CancelIoEx(handle, nullptr);
    if (reader.joinable())
        reader.join();
    CloseHandle(handle);
    handle = nullptr;
    handler = nullptr;
    log(QStringLiteral("hid: closed"));
}

bool hid_device::write(const uint8_t* data, size_t len)
{
    if (!is_open() || out_len < 2 || len + 1 > out_len)
        return false;

    std::vector<uint8_t> report(out_len, 0);
    std::memcpy(report.data() + 1, data, len);

    std::lock_guard<std::mutex> l(write_mtx);
    OVERLAPPED ov = {};
    ov.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    DWORD written = 0;
    BOOL ok = WriteFile(handle, report.data(), DWORD(report.size()), &written, &ov);
    if (!ok && GetLastError() == ERROR_IO_PENDING)
    {
        if (WaitForSingleObject(ov.hEvent, 1000) == WAIT_OBJECT_0)
            ok = GetOverlappedResult(handle, &ov, &written, FALSE);
        else
        {
            CancelIoEx(handle, &ov);
            GetOverlappedResult(handle, &ov, &written, TRUE);
            ok = FALSE;
        }
    }
    CloseHandle(ov.hEvent);
    if (!ok)
        log(QStringLiteral("hid: write failed: %1").arg(last_error_text()));
    return ok;
}

void hid_device::reader_loop()
{
    std::vector<uint8_t> buf(in_len);
    OVERLAPPED ov = {};
    ov.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);

    while (!stop_flag.load())
    {
        ResetEvent(ov.hEvent);
        DWORD n = 0;
        BOOL ok = ReadFile(handle, buf.data(), DWORD(buf.size()), &n, &ov);
        if (!ok)
        {
            if (GetLastError() != ERROR_IO_PENDING)
            {
                if (!stop_flag.load())
                    log(QStringLiteral("hid: read failed: %1").arg(last_error_text()));
                break;
            }
            bool done = false;
            while (!done)
            {
                if (WaitForSingleObject(ov.hEvent, 100) == WAIT_OBJECT_0)
                    done = true;
                else if (stop_flag.load())
                {
                    CancelIoEx(handle, &ov);
                    GetOverlappedResult(handle, &ov, &n, TRUE);
                    CloseHandle(ov.hEvent);
                    return;
                }
            }
            if (!GetOverlappedResult(handle, &ov, &n, FALSE))
            {
                if (!stop_flag.load())
                    log(QStringLiteral("hid: read completion failed: %1").arg(last_error_text()));
                break;
            }
        }
        if (n > 1 && handler)
            handler(buf.data() + 1, n - 1);  // skip the report-id byte
    }
    CloseHandle(ov.hEvent);
}

} // namespace xr_glasses
