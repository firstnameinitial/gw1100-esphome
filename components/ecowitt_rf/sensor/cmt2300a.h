// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <Arduino.h>
#include <initializer_list>
#include "radio_profile.h"

// Receive-only CMT2300A / CMT2219A transport. The register/FIFO transaction framing
// follows CMOSTEK's published serial interface and the MIT anisyanka/cmt2300a driver
// (attribution in THIRD_PARTY_NOTICES.md). Board timings are conservative minima.
// SDIO is released after every byte; this is essential on the console's shared bus.
namespace esphome { namespace ecowitt_rf {
constexpr uint8_t PIN_SCLK = 5;
constexpr uint8_t PIN_SDIO = 2;
constexpr uint8_t PIN_CS = 13;
constexpr uint8_t PIN_FCSB = 14;
constexpr uint8_t PIN_DOUT = 15;
static bool g_is_cmt2300a = true;

inline void send_byte(uint8_t value) {
    pinMode(PIN_SDIO, OUTPUT);
    for (uint8_t bit = 0x80; bit; bit >>= 1) {
        digitalWrite(PIN_SCLK, LOW);
        digitalWrite(PIN_SDIO, (value & bit) != 0);
        digitalWrite(PIN_SCLK, HIGH);
    }
    pinMode(PIN_SDIO, INPUT);
    digitalWrite(PIN_SCLK, LOW);
}
inline uint8_t recv_byte() {
    uint8_t value = 0;
    pinMode(PIN_SDIO, INPUT);
    for (unsigned count = 0; count != 8; ++count) {
        digitalWrite(PIN_SCLK, HIGH);
        value = uint8_t((value << 1) | (digitalRead(PIN_SDIO) != 0));
        digitalWrite(PIN_SCLK, LOW);
    }
    return value;
}

// A scoped transaction always releases its select line, including early returns.
class RadioTransaction {
 public:
    RadioTransaction() {
        digitalWrite(PIN_FCSB, HIGH);
        delayMicroseconds(4);
        digitalWrite(PIN_CS, LOW);
        delayMicroseconds(16);
    }
    ~RadioTransaction() {
        delayMicroseconds(4);
        digitalWrite(PIN_CS, HIGH);
        pinMode(PIN_SDIO, INPUT);
    }
};
inline void write_reg(uint8_t reg, uint8_t value) {
    RadioTransaction transaction;
    send_byte(reg & 0x7f);
    send_byte(value);
}
inline uint8_t read_reg(uint8_t reg) {
    RadioTransaction transaction;
    send_byte(reg | 0x80);
    return recv_byte();
}
inline void radio_field(uint8_t reg, uint8_t mask, uint8_t value) {
    write_reg(reg, uint8_t((read_reg(reg) & ~mask) | (value & mask)));
}
inline void radio_bank(uint8_t first, const uint8_t *values, size_t count) {
    for (size_t offset = 0; offset != count; ++offset)
        write_reg(uint8_t(first + offset), values[offset]);
}

// Only receiver states are accepted. No TX command or FIFO writer is exposed.
// The status register, not TX_DONE, determines when configuration can proceed.
inline bool mode_cmd(uint8_t command) {
    uint8_t state;
    switch (command) {
        case 0x10: state = 1; break;  // sleep
        case 0x02: state = 2; break;  // standby
        case 0x08: state = 5; break;  // receive
        default: return false;
    }
    write_reg(0x60, command);
    for (unsigned attempt = 0; attempt != 40; ++attempt) {
        delayMicroseconds(100);
        if ((read_reg(0x61) & 15) == state) return true;
    }
    return false;
}

inline void radio_2300_profile(uint8_t band, bool acquire) {
    write_reg(0x7f, 0xff);  // software reset
    delay(2);
    mode_cmd(0x02);
    radio_field(0x61, 0x30, 0x10); // retain config, disable reset pin
    radio_field(0x62, 0x20, 0x20);
    radio_field(0x0d, 0xe0, 0);
    write_reg(0x6a, 7);
    write_reg(0x6b, 0x3f);
    radio_bank(0, RADIO_CAL, sizeof(RADIO_CAL));
    radio_bank(8, kMainTable, sizeof(kMainTable));
    radio_field(0x28, 0x10, band == 1 ? 0x10 : 0);
    if (band >= 1 && band <= 3) radio_bank(0x18, kBandTable[band - 1], 8);
    if (acquire) {
        write_reg(4, 0x30);
        radio_bank(0x20, RADIO_ACQUIRE, sizeof(RADIO_ACQUIRE));
        radio_bank(0x28, kModeTable, sizeof(kModeTable));
        write_reg(0x51, 1);
        write_reg(0x55, 0x41);
        write_reg(0x56, band ? 0x9a : 0x4d);
        write_reg(0x57, band ? 0x0c : 6);
        write_reg(0x59, 1);
    } else {
        if (band >= 2) write_reg(0x27, 0x0b);
        write_reg(0x56, 7);
        write_reg(0x57, 0x0b);
    }
    radio_field(9, 7, 2);
    write_reg(0x65, 0x17); // IO selection
    write_reg(0x68, 0x21); // packet interrupt enable
    radio_field(0x0d, 0xe0, 0);
    radio_field(0x69, 2, 2); // merged receive FIFO
    radio_field(0x54, 0x7f, 0x30);
    radio_field(0x45, 0x70, 0);
    write_reg(0x46, 0x45); // drain 69 raw bytes
    mode_cmd(0x10);
}

inline void init_2219a(uint8_t band) {
    for (uint8_t reg : {uint8_t(0x1f), uint8_t(0x23), uint8_t(0x25)})
        radio_field(reg, 0x80, 0);
    for (uint8_t state : {uint8_t(4), uint8_t(0x10)}) {
        radio_field(0x47, 0x1f, state);
        delay(1);
    }
    radio_field(0x1f, 0x60, 0);
    radio_field(0x2d, 0x20, 0);
    radio_field(0x47, 0x1f, 2);
    radio_bank(0, k2219Table[band <= 2 ? band : 1], 62);
    radio_field(0x3d, 0x20, 0x20);
    write_reg(0x42, 0xff);
    write_reg(0x40, 0x0a); // latched packet-ready on both GPOs
    write_reg(0x3f, 0xff);
    write_reg(0x41, 0x6c);
    radio_field(0x47, 0x1f, 2);
    radio_field(0x47, 0x1f, 8);
}
inline void init(uint8_t band) {
    pinMode(PIN_SCLK, OUTPUT);
    pinMode(PIN_CS, OUTPUT);
    pinMode(PIN_FCSB, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    digitalWrite(PIN_FCSB, HIGH);
    digitalWrite(PIN_SCLK, LOW);
    radio_2300_profile(band, true);
    g_is_cmt2300a = read_reg(1) == 0x66;
    if (!g_is_cmt2300a) { init_2219a(band); return; }
    mode_cmd(2);
    radio_field(0x69, 7, 7);
    write_reg(0x39, 8);
    write_reg(0x3a, 0);
    write_reg(0x6c, 1);
    write_reg(0x6a, 4);
    radio_2300_profile(band, false);
    mode_cmd(2);
    radio_field(0x69, 7, 2);
    radio_field(0x66, 0x1f, 0x10);
    radio_field(0x67, 0x1f, 0x19);
    write_reg(0x6b, 1);
    mode_cmd(8);
}
inline size_t fifo_packet_len() { return g_is_cmt2300a ? 69 : 32; }
inline bool rx_ready() { return digitalRead(PIN_DOUT) != 0; }
inline void fifo_read(uint8_t *out, size_t count) {
    digitalWrite(PIN_CS, HIGH);
    pinMode(PIN_SDIO, INPUT);
    for (size_t pos = 0; pos != count; ++pos) {
        digitalWrite(PIN_FCSB, LOW);
        delayMicroseconds(4);
        out[pos] = recv_byte();
        delayMicroseconds(4);
        digitalWrite(PIN_FCSB, HIGH);
        delayMicroseconds(8);
    }
}
inline void rearm_rx() {
    if (!g_is_cmt2300a) {
        write_reg(0x42, 0xff);
        write_reg(0x44, uint8_t(read_reg(0x44) + 4));
        return;
    }
    mode_cmd(2);
    write_reg(0x6a, 7);
    write_reg(0x6b, 0x3f);
    write_reg(0x6c, 2);
    mode_cmd(8);
}
} }  // namespace esphome::ecowitt_rf
