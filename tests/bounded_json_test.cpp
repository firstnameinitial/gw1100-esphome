// SPDX-License-Identifier: GPL-3.0-only
#include <cassert>
#include <climits>
#include <string>
#include "../components/ecowitt_rf/sensor/bounded_json.h"
using namespace esphome::ecowitt_rf;
static void fixture(BoundedJson &j) {
    j.object(); j.field("s", "\"\\\n\t\001"); j.field("bool", true);
    j.field("max", UINT_MAX); j.field("min", INT_MIN); j.field("v", 139.5f);
    j.field("missing", NAN); j.array("roster"); j.object(); j.field("id", "003F0C");
    j.end_object(); j.end_array(); j.end_object();
}
int main() {
    BoundedJson measure; fixture(measure);
    char bytes[256] = {}; BoundedJson writer(bytes, measure.size()); fixture(writer);
    assert(writer.fits() && writer.size() == measure.size());
    assert(std::string(bytes, writer.size()) ==
        "{\"s\":\"\\\"\\\\\\u000a\\u0009\\u0001\",\"bool\":true,\"max\":4294967295,\"min\":-2147483648,\"v\":139.5,\"missing\":null,\"roster\":[{\"id\":\"003F0C\"}]}");
    char short_buffer[4] = {'a', 'b', 'c', 'd'};
    BoundedJson short_writer(short_buffer, 3); fixture(short_writer);
    assert(!short_writer.fits() && short_buffer[3] == 'd');
    assert(state_response_fits(2500, 9000, 6000));
    assert(!state_response_fits(4000, 9000, 6000));
    assert(!state_response_fits(2500, 9000, 2000));
    assert(!state_response_fits(9000, 20000, 20000));
    assert(!state_response_fits(1, 4000, 4000));
    std::puts("Bounded JSON escaping, exact size, overflow and heap guards passed");
}
