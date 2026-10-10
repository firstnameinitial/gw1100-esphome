// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "fineoffset.h"

namespace esphome { namespace ecowitt_rf {

// Explicit, stable reservations in Ecowitt's eight extra T/H slots. Unmapped
// barometers remain available by RF ID over MQTT, including their pressure.
struct RemoteIndoorChannels {
    uint32_t ids[8] = {UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX,
                       UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
    void set(uint32_t id, uint8_t channel) {
        if (channel >= 1 && channel <= 8) ids[channel - 1] = id;
    }
    bool reserved(uint8_t channel) const {
        return channel >= 1 && channel <= 8 && ids[channel - 1] != UINT32_MAX;
    }
    bool map(const Reading &source, Reading &out) const {
        if (source.family != Family::WH25 || source.model == 0) return false;
        for (uint8_t i = 0; i < 8; ++i) {
            if (ids[i] != source.id) continue;
            out = source;
            out.family = Family::WH31;
            out.channel = i + 1;
            // WH31 formatting carries only T/H/battery, never station pressure.
            return true;
        }
        return false;
    }
};

}}  // namespace esphome::ecowitt_rf
