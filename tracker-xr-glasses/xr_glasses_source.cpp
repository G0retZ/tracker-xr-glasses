/* Copyright (c) 2026, Sergei 'G0retZ' Mitrofanov

 * Permission to use, copy, modify, and/or distribute this
 * software for any purpose with or without fee is hereby granted,
 * provided that the above copyright notice and this permission
 * notice appear in all copies.
 */

#include "xr_glasses_source.hpp"
#include "xr_glasses_log.hpp"
#include "xr_glasses_rayneo.hpp"
#include "xr_glasses_viture.hpp"
#include "xr_glasses_xreal.hpp"

namespace xr_glasses {

std::unique_ptr<pose_source> make_source(const device& dev, QString& error)
{
    switch (dev.vend)
    {
    case vendor::viture:
    {
        const viture_family f = viture_family_for_pid(dev.product_id);
        log(QStringLiteral("viture family: %1").arg(QLatin1String(viture_family_name(f))));
        switch (f)
        {
        case viture_family::pro2:
        case viture_family::beast:
        case viture_family::unknown:  // newer models are expected to speak V2
            return std::make_unique<viture_v2_source>();
        case viture_family::one:
        case viture_family::pro:
        case viture_family::luma:
            return std::make_unique<viture_v1_source>();
        case viture_family::ultra:
            error = QStringLiteral("Luma Ultra uses a different (6DoF, USB bulk) interface and is not supported.");
            return nullptr;
        }
        return nullptr;
    }
    case vendor::xreal:
        return std::make_unique<xreal_source>();
    case vendor::rayneo:
        return std::make_unique<rayneo_source>();
    }
    error = QStringLiteral("unsupported device");
    return nullptr;
}

} // namespace xr_glasses
