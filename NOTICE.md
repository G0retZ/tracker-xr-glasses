# Notices

## This plugin

`opentrack-tracker-xr-glasses` is licensed under the ISC license (see `LICENSE`), the same
license opentrack uses. It contains no vendor code and loads no vendor libraries.

## Protocol sources

The device protocols were reverse-engineered by others. This plugin reimplements them;
the facts below (framing, command ids, packet layouts, axis conventions, filter tuning)
were taken from these projects, not their code.

| Project | License | Used for |
|---|---|---|
| [DroidDeck](https://github.com/Droid-Deck/DroidDeck) | GPL-3.0 | VITURE protocol V1 (packet layout, CRC16-XMODEM, IMU commands, payload format), XREAL One-series TCP report format and 1S axis frame, RayNeo command/response layout, board-specific IMU mounting corrections, complementary-filter parameters, VITURE product-id table |
| [elasticjava/viture-v2](https://github.com/elasticjava/viture-v2) | MIT | VITURE protocol V2 (frame format, IMU control, pose event layout) |
| [RayNeo-Air-Series-3DoF-Scaffold](https://github.com/informalTechCode/RayNeo-Air-Series-3DoF-Scaffold) | Apache-2.0 | RayNeo packet format and the complementary-filter design |
| [Monado](https://gitlab.freedesktop.org/monado/monado) (`src/xrt/drivers/rayneo/`) | BSL-1.0 | RayNeo protocol (via the scaffold) |

Protocol descriptions are facts and are not subject to those projects' copyrights; no
source code from any of them is included in this repository. DroidDeck's GPL-3.0 license
therefore does not extend to this plugin.

## opentrack

opentrack is Copyright (c) Stanislaw Halik and contributors, ISC license —
https://github.com/opentrack/opentrack. This plugin is built against opentrack's public
plugin interface and links `opentrack-api`, `opentrack-options` and `opentrack-compat`.

## Trademarks

VITURE is a trademark of VITURE Inc. XREAL is a trademark of XREAL Inc. RayNeo is a
trademark of TCL/RayNeo. They are used here only to state device compatibility. This
project is not affiliated with, endorsed by, or sponsored by any of them.
