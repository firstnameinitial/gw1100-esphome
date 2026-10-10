// SPDX-License-Identifier: GPL-3.0-only
#pragma once
// GPIO-level host model. Serial bits reach the actual driver; no mocked driver calls.
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <array>
#include <vector>
#include <utility>
using byte = uint8_t;
constexpr uint8_t LOW=0, HIGH=1, INPUT=0, OUTPUT=1, INPUT_PULLUP=2, INPUT_PULLDOWN=3;
#define HEX 16
#define DEC 10
#define OCT 8
#define BIN 2
namespace mock {
inline std::array<uint8_t,32> level{}, direction{};
inline std::array<uint8_t,128> registers{};
inline std::array<uint8_t,0x4000> lcd{};
inline std::vector<std::pair<uint16_t,uint8_t>> writes, lcd_writes;
inline std::vector<uint8_t> fifo;
inline size_t fifo_bit=0;
inline uint8_t sclk=0, sdio=0, cs=0, fcs=0, part=0x66;
inline unsigned bits=0, byte_count=0, rx_bit=0;
inline uint8_t shift=0, address=0;
inline unsigned lcd_bits=0, lcd_count=0, lcd_rx_bit=0;
inline uint8_t lcd_shift=0;
inline uint16_t lcd_address=0;
inline bool active=false, lcd_active=false, stuck=false;
inline unsigned long now=0;
inline unsigned long microseconds=0;
inline unsigned mode_polls=0;
inline void reset(uint8_t clock, uint8_t data, uint8_t select, uint8_t fifo_select,
                  uint8_t chip=0x66) {
    sclk=clock; sdio=data; cs=select; fcs=fifo_select; part=chip;
    level.fill(HIGH); direction.fill(INPUT); registers.fill(0); lcd.fill(0);
    writes.clear(); lcd_writes.clear(); fifo.clear(); fifo_bit=0;
    active=lcd_active=stuck=false; bits=byte_count=rx_bit=0;
    lcd_bits=lcd_count=lcd_rx_bit=0; shift=lcd_shift=0; lcd_address=0;
    now=microseconds=mode_polls=0;
}
inline uint8_t read_radio() {
    if(address==1) return part;
    if(address==0x61) ++mode_polls;
    return registers[address];
}
inline void radio_byte(uint8_t value) {
    if(byte_count++==0) { address=value&0x7f; rx_bit=0; return; }
    registers[address]=value; writes.emplace_back(address,value);
    if(address==0x60 && !stuck) {
        uint8_t state=value==0x10?1:value==2?2:value==8?5:0;
        registers[0x61]=(registers[0x61]&0xf0)|state;
    }
}
inline void lcd_byte(uint8_t value) {
    if(lcd_count==0) lcd_address=uint16_t(value)<<8;
    else if(lcd_count==1) lcd_address|=value;
    else {
        assert(lcd_address<lcd.size());
        lcd[lcd_address]=value;
        lcd_writes.emplace_back(lcd_address++,value);
    }
    ++lcd_count;
}
}
inline void pinMode(uint8_t pin,uint8_t mode) { mock::direction[pin]=mode; }
inline void digitalWrite(uint8_t pin,uint8_t value) {
    using namespace mock;
    bool rising=!level[pin] && value;
    level[pin]=value;
    if(pin==cs) {
        active=!value;
        if(active) { bits=byte_count=rx_bit=0; shift=0; }
    }
    if(pin==18 && cs!=18) {
        lcd_active=!value;
        if(lcd_active) { lcd_bits=lcd_count=lcd_rx_bit=0; lcd_shift=0; }
    }
    if(pin==sclk && rising && active && direction[sdio]==OUTPUT) {
        shift=uint8_t((shift<<1)|(level[sdio]!=0));
        if(++bits==8) { radio_byte(shift); shift=0; bits=0; }
    }
    if(pin==6 && rising && lcd_active && lcd_count<2) {
        lcd_shift=uint8_t((lcd_shift<<1)|(level[7]!=0));
        if(++lcd_bits==8) { lcd_byte(lcd_shift); lcd_shift=0; lcd_bits=0; }
    } else if(pin==6 && rising && lcd_active && !(lcd_address&0x8000)) {
        lcd_shift=uint8_t((lcd_shift<<1)|(level[7]!=0));
        if(++lcd_bits==8) { lcd_byte(lcd_shift); lcd_shift=0; lcd_bits=0; }
    }
}
inline uint8_t digitalRead(uint8_t pin) {
    using namespace mock;
    if(pin==sdio && active) {
        return (read_radio()>>(7-(rx_bit++%8)))&1;
    }
    if(pin==sdio && !level[fcs]) {
        assert(fifo_bit<8*fifo.size());
        unsigned pos=fifo_bit++;
        return (fifo[pos/8]>>(7-pos%8))&1;
    }
    if(pin==1 && lcd_active) {
        return (lcd[lcd_address&0x7fff]>>(7-(lcd_rx_bit++%8)))&1;
    }
    return level[pin];
}
inline void delay(unsigned long ms) { mock::now+=ms; }
inline void delayMicroseconds(unsigned int us) { mock::microseconds+=us; }
inline unsigned long millis() { return mock::now; }
