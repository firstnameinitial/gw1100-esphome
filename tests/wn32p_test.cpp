// SPDX-License-Identifier: GPL-3.0-only
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include "../components/ecowitt_rf/sensor/fineoffset.h"

using namespace esphome::ecowitt_rf;
int main() {
    // Synthetic WN32P frame: ID 0x12, 20 C, 45%, 1000 hPa.
    const uint8_t sample[] = {0xe1,0x22,0x58,0x2d,0x27,0x10,0xbf,0xa5,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    Reading r;
    assert(decode(sample, 7, r));
    assert(r.family == Family::WH25 && r.id == 0x12 && r.model == 0xe);
    assert(std::fabs(r.temp_c - 20.0f) < .01f && r.humidity == 45);
    assert(std::fabs(r.pressure_hpa - 1000.0f) < .01f && !r.battery_low);
    assert(decode(sample, sizeof(sample), r)); // trailing FIFO bytes are not WH25 XOR
    assert(!decode(sample, 6, r));
    uint8_t changed[sizeof(sample)]; std::memcpy(changed,sample,sizeof(sample));
    changed[6] ^= 1; assert(!decode(changed,sizeof(changed),r));
    std::memcpy(changed,sample,sizeof(sample)); changed[3]=101;
    changed[6]=add_sum(changed,6); assert(!decode(changed,sizeof(changed),r));
    std::memcpy(changed,sample,sizeof(sample)); changed[4]=0xff;changed[5]=0;
    changed[6]=add_sum(changed,6); assert(!decode(changed,sizeof(changed),r));
    std::memcpy(changed,sample,sizeof(sample)); changed[1]|=4;
    changed[6]=add_sum(changed,6); assert(!decode(changed,sizeof(changed),r));
    std::memcpy(changed,sample,sizeof(sample)); changed[1]|=8;
    changed[6]=add_sum(changed,6); assert(decode(changed,sizeof(changed),r));
    assert(r.battery_low && std::fabs(r.temp_c - 20.0f)<.01f);
    // A second transmitter must retain its own identity even if T/H/P are equal.
    changed[0]=0xe2;changed[1]=0xa2;changed[6]=add_sum(changed,6);
    assert(decode(changed,sizeof(changed),r) && r.id==0x2a);
    // Legacy WH25 with its optional XOR remains accepted.
    changed[7]=xor_sum_swapped(changed,6);
    assert(decode(changed,8,r));
    std::puts("WN32P synthetic-frame, checksum, bounds, identity and legacy checks passed");
}
