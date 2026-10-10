// SPDX-License-Identifier: GPL-3.0-only
#include <cassert>
#include <cstdio>
#include "../components/ecowitt_rf/sensor/remote_indoor.h"
using namespace esphome::ecowitt_rf;
int main() {
    RemoteIndoorChannels channels;
    Reading source, mapped;
    source.family = Family::WH25; source.model = 0xe; source.id = 0x12;
    source.temp_c = 21.6f; source.humidity = 50; source.pressure_hpa = 1013.5f;
    assert(!channels.map(source, mapped));
    channels.set(0x12, 1); channels.set(0x2a, 8);
    assert(channels.map(source, mapped));
    assert(mapped.family == Family::WH31 && mapped.channel == 1);
    assert(mapped.temp_c == source.temp_c && mapped.humidity == source.humidity);
    assert(source.family == Family::WH25 && source.channel == 0);
    source.id = 0x2a;
    assert(channels.map(source, mapped) && mapped.channel == 8);
    source.id = 0x13; assert(!channels.map(source, mapped));
    source.id = 0x12; source.model = 0; assert(!channels.map(source, mapped));
    source.model = 0xe; source.family = Family::WH31;
    assert(!channels.map(source, mapped));
    assert(channels.reserved(1) && channels.reserved(8));
    assert(!channels.reserved(0) && !channels.reserved(9) && !channels.reserved(2));
    channels.set(0x123, 0); channels.set(0x456, 9);
    assert(channels.ids[0] == 0x12 && channels.ids[7] == 0x2a);
    std::puts("Remote indoor identity, extra-channel mapping and collision checks passed");
}
