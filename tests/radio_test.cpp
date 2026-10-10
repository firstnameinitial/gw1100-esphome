// SPDX-License-Identifier: GPL-3.0-only
#include "cmt2300a.h"
#include <cassert>
#include <cstdio>
using namespace esphome::ecowitt_rf;
void reset(uint8_t part=0x66) { mock::reset(PIN_SCLK,PIN_SDIO,PIN_CS,PIN_FCSB,part); }
int main() {
    for(uint8_t band=0;band<4;++band) {
        reset(); init(band);
        assert(g_is_cmt2300a && fifo_packet_len()==69);
        assert((mock::registers[0x61]&15)==5);
        assert(mock::registers[0x46]==69);
        assert(mock::registers[0x16]==0x32); // valid RSSI averaging
        assert(mock::registers[0x43]==0xd4 && mock::registers[0x44]==0x2d);
        assert((mock::registers[0x69]&7)==2); // merged RX, no TX FIFO selection
        assert((mock::registers[0x28]&0x10)==(band==1?0x10:0));
        for(auto w:mock::writes) assert(w.first!=0x60 || w.second!=0x40);
        assert(mock::level[PIN_CS] && mock::level[PIN_FCSB]);
        assert(mock::direction[PIN_SDIO]==INPUT);
        rearm_rx(); assert((mock::registers[0x61]&15)==5);
        mock::fifo={0x96,0x42,0xff,0}; uint8_t got[4]={}; fifo_read(got,4);
        assert(mock::fifo_bit==32);
        for(unsigned i=0;i<4;++i) assert(got[i]==mock::fifo[i]);
        assert(mock::level[PIN_CS] && mock::level[PIN_FCSB]);
    }
    for(uint8_t band=0;band<4;++band) {
        reset(0xb3); init(band);
        assert(!g_is_cmt2300a && fifo_packet_len()==32);
        assert((mock::registers[0x47]&31)==8);
        assert(mock::registers[0x40]==0x0a); // latched packet-ready
        assert(mock::registers[0x17]==0xd4 && mock::registers[0x18]==0x2d);
        auto before=mock::registers[0x44]; rearm_rx();
        assert(mock::registers[0x44]==uint8_t(before+4));
    }
    reset(); mock::stuck=true; mock::registers[0x6a]=8; // stale TX_DONE must not pass
    assert(!mode_cmd(8)); assert(mock::mode_polls==40*8);
    auto n=mock::writes.size(); assert(!mode_cmd(0x40)); assert(mock::writes.size()==n);
    puts("radio: both chip families, four bands, RX-only, FIFO, mode timeout passed");
}
