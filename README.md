# tracker-xr-glasses

An [opentrack](https://github.com/opentrack/opentrack) **input tracker** for 3DoF XR
glasses. The glasses show up in opentrack's *Input* list as **"XR Glasses (3DoF)"**, and
opentrack's own Start/Stop, centering, filtering and axis mapping apply to them like to
any other tracker. No vendor SDK, driver or app is needed: the plugin speaks the glasses'
USB protocols directly.

```
glasses ──USB──▶ opentrack.exe ──▶ game (freetrack / TrackIR / UDP / SteamVR add-ons…)
                   │
                   └── modules/opentrack-tracker-xr-glasses.dll (this plugin)
```

Translation is always zero: these glasses have no positional tracking.

## Supported glasses

| Glasses | How | Orientation | Status |
|---|---|---|---|
| VITURE Pro 2 | HID, protocol V2 | fused on the glasses | **verified** |
| VITURE Beast / Phantom Beast | HID, protocol V2 | fused on the glasses | untested |
| VITURE One, One Lite, Pro, Luma, Luma Pro, Luma Cyber | HID, protocol V1 | fused on the glasses | untested |
| XREAL One, One Pro, 1S | TCP over the glasses' USB Ethernet link | gyro+accel fused on the PC | untested |
| RayNeo Air 2 / 2s / 3s / 3s Pro / 4 Pro, GT / GT Max | HID | gyro+accel fused on the PC | untested |

Not supported: VITURE Luma Ultra (6DoF over USB bulk), XREAL Air series (different
transport), anything not listed. "Untested" means the protocol is implemented from
documented captures (see *Acknowledgements*) but nobody has run this plugin on that
hardware yet; reports welcome, especially about axis directions. Windows x64 only.

## Install

1. **Match versions.** A tracker DLL only works with the exact opentrack release it was
   built for (opentrack has no plugin ABI versioning). Each release of this plugin states
   the opentrack version in its name and in `BUILD-INFO.txt`:

   | plugin release | opentrack | Qt |
   |---|---|---|
   | v0.x | 2026.1.0 | 6.10.1 |

2. Copy `opentrack-tracker-xr-glasses.dll` into opentrack's `modules\` folder (next to
   `opentrack-tracker-udp.dll` and friends).

3. In opentrack: *Input* → **XR Glasses (3DoF)** → *Start*. Put the glasses on, look
   straight ahead and press the *Center* hotkey.

XREAL One-series only: Windows must show a USB Ethernet adapter for the glasses (it uses
the inbox NCM driver) and *Ethernet* must not be disabled in the glasses' developer menu.

## Settings

Open the tracker's settings (gear icon next to the Input selector):

- **Glasses** — *Auto (first found)*, or a specific connected device. Use this when more
  than one pair is attached.
- **IMU rate** — *Auto* or 60–240 Hz. Honoured by VITURE glasses; XREAL and RayNeo stream
  at their fixed rate. Higher is not better beyond opentrack's own ~250 Hz pipeline.
- **Status** — the running device and protocol, or why nothing is running.

Axis direction: the plugin outputs opentrack's conventions (yaw + = left, pitch + = up,
roll + = right). If a game expects the opposite, invert the axis in opentrack's *Mapping*
or *Options → Output*. If an axis is *swapped* with another one (e.g. turning your head
pitches the view), that's a per-model axis-frame issue in the plugin: report it with the
model name.

## Troubleshooting

Every *Start* writes a fresh log to `%TEMP%\xr-glasses.log` with device discovery, every
command and reply, and the reason for a failure. Look there first; include it when
reporting a problem.

**"No supported glasses found"** — check *Device Manager → Human Interface Devices* (VITURE,
RayNeo) or *Network adapters* (XREAL) with the glasses plugged in; plug them directly into
the PC rather than through an unpowered hub.

**"did not start the pose stream" / "no pose data"** — another process may hold the
glasses (the vendor's desktop app, VertoXR, …); only one can use them at a time. Unplug,
reconnect and *Start* again.

**"could not reach the glasses' IMU service"** (XREAL) — no USB Ethernet link to the
glasses. Check the adapter in Device Manager and the developer-menu Ethernet setting.

**opentrack says "library … failed" at startup / the tracker is missing from the list** —
plugin/opentrack version mismatch. Use the plugin build made for your opentrack release.

**Related tools.** [VRto3D](https://github.com/oneup03/VRto3D) takes head pose *from*
opentrack over UDP, so `glasses → this plugin → opentrack (UDP output) → VRto3D` works.
Do not run the vendor's desktop app or VertoXR at the same time; they claim the glasses.

## Building

The plugin is built *inside* opentrack's source tree (its CMake discovers any
`tracker-*/` directory).

```powershell
git clone --branch opentrack-2026.1.0 https://github.com/opentrack/opentrack otr
Copy-Item -Recurse tracker-xr-glasses otr/tracker-xr-glasses
Copy-Item -Force ci/opentrack-opencv.cmake otr/cmake/opentrack-opencv.cmake   # build without OpenCV
cmake -G Ninja -S otr -B build -DCMAKE_BUILD_TYPE=Release `
      -DQt6_DIR="<Qt>/6.10.1/msvc2022_64/lib/cmake/Qt6" `
      -DOPENTRACK_USERCONFIG="$PWD/ci/opentrack-userconfig.cmake"
cmake --build build --target opentrack-tracker-xr-glasses
# → build/tracker-xr-glasses/opentrack-tracker-xr-glasses.dll
```

Requirements: Visual Studio 2022+ (MSVC x64), Qt 6.10.1 (msvc2022_64), CMake, Ninja.
`.github/workflows/build.yml` does exactly this on every push and attaches the DLL as an
artifact; tags `v*` also publish a GitHub release.

## Acknowledgements

All protocol knowledge comes from other people's reverse-engineering work; this plugin
contains original implementations of it. See `NOTICE.md` for licenses.

- [DroidDeck](https://github.com/Droid-Deck/DroidDeck) — the VITURE V1, XREAL One-series
  and RayNeo adapters (framing, command ids, packet layouts, per-model IMU mounting and
  axis conventions) and the fusion-filter tuning.
- [elasticjava/viture-v2](https://github.com/elasticjava/viture-v2) — the VITURE Gen2
  protocol.
- [RayNeo-Air-Series-3DoF-Scaffold](https://github.com/informalTechCode/RayNeo-Air-Series-3DoF-Scaffold)
  and the [Monado](https://gitlab.freedesktop.org/monado/monado) RayNeo driver — the RayNeo
  protocol and filter design.

## License

ISC, like opentrack — see `LICENSE`. VITURE, XREAL and RayNeo are trademarks of their
respective owners, used here only to describe compatibility; this project is not
affiliated with or endorsed by any of them.
