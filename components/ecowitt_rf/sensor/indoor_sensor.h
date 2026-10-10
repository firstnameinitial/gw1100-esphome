// SPDX-License-Identifier: GPL-3.0-only
// GW1100 built-in indoor T/H/P — reverse-engineered from the stock firmware.
// Pressure = Hoperf HP203B barometer @ I2C 0x76. Temperature/humidity = a digital sensor on
// the same bus, auto-probed at boot: SHT3x/SHT4x @0x44, SI7006 @0x40, SHTC3 @0x70.
// (NOT a thermistor/analog humidity — that was a wrong earlier assumption.)
#pragma once
#include "esphome.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace ecowitt_rf {

// CRC-8, poly 0x31 (SHT3x/SHT4x/SHTC3).
inline uint8_t sht_crc8(const uint8_t *d, size_t n) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

class IndoorSensor {
 public:
    void set_bus(i2c::I2CBus *bus) { bus_ = bus; }

    // Probe the T/H sensor (SHT3x/SHT4x/SI7006/SHTC3, in the stock's order).
    uint8_t probe() {
        uint8_t c[2];
        c[0] = 0x30; c[1] = 0xA2;                       // SHT3x soft reset
        if (bus_->write(0x44, c, 2) == i2c::ERROR_OK) { th_type_ = 1; return 1; }
        c[0] = 0x94;                                     // SHT4x reset
        if (bus_->write(0x44, c, 1) == i2c::ERROR_OK) { th_type_ = 2; return 2; }
        c[0] = 0xFE;                                     // SI7006 reset
        if (bus_->write(0x40, c, 1) == i2c::ERROR_OK) { th_type_ = 3; return 3; }
        c[0] = 0x35; c[1] = 0x17;                        // SHTC3 wakeup
        if (bus_->write(0x70, c, 2) == i2c::ERROR_OK) { th_type_ = 4; return 4; }
        th_type_ = 0;
        return 0;
    }

    bool read_th(float &temp_c, float &humidity) {
        if (th_type_ == 0 || bus_ == nullptr)
            return false;
        uint8_t c[2], d[6];
        if (th_type_ == 1) {               // SHT3x
            c[0] = 0x24; c[1] = 0x00;
            if (bus_->write(0x44, c, 2) != i2c::ERROR_OK) return false;
            delay(20);
            if (bus_->read(0x44, d, 6) != i2c::ERROR_OK) return false;
            if (sht_crc8(d, 2) != d[2] || sht_crc8(d + 3, 2) != d[5]) return false;
            uint16_t st = ((uint16_t)d[0] << 8) | d[1];
            uint16_t sr = ((uint16_t)d[3] << 8) | d[4];
            temp_c = -45.0f + 175.0f * st / 65535.0f;
            humidity = 100.0f * sr / 65535.0f;
        } else if (th_type_ == 2) {        // SHT4x
            c[0] = 0xFD;
            if (bus_->write(0x44, c, 1) != i2c::ERROR_OK) return false;
            delay(10);
            if (bus_->read(0x44, d, 6) != i2c::ERROR_OK) return false;
            if (sht_crc8(d, 2) != d[2] || sht_crc8(d + 3, 2) != d[5]) return false;
            uint16_t st = ((uint16_t)d[0] << 8) | d[1];
            uint16_t sr = ((uint16_t)d[3] << 8) | d[4];
            temp_c = -45.0f + 175.0f * st / 65535.0f;
            humidity = -6.0f + 125.0f * sr / 65535.0f;
        } else if (th_type_ == 3) {        // SI7006
            c[0] = 0xF5;
            if (bus_->write(0x40, c, 1) != i2c::ERROR_OK) return false;
            delay(25);
            uint8_t rh[2];
            if (bus_->read(0x40, rh, 2) != i2c::ERROR_OK) return false;
            uint16_t sr = ((uint16_t)rh[0] << 8) | rh[1];
            humidity = 125.0f * sr / 65536.0f - 6.0f;
            c[0] = 0xF3;
            if (bus_->write(0x40, c, 1) != i2c::ERROR_OK) return false;
            delay(25);
            uint8_t t[2];
            if (bus_->read(0x40, t, 2) != i2c::ERROR_OK) return false;
            uint16_t st = ((uint16_t)t[0] << 8) | t[1];
            temp_c = -46.85f + 175.72f * st / 65536.0f;
        } else if (th_type_ == 4) {        // SHTC3
            c[0] = 0x78; c[1] = 0x66;
            if (bus_->write(0x70, c, 2) != i2c::ERROR_OK) return false;
            delay(15);
            if (bus_->read(0x70, d, 6) != i2c::ERROR_OK) return false;
            if (sht_crc8(d, 2) != d[2] || sht_crc8(d + 3, 2) != d[5]) return false;
            uint16_t st = ((uint16_t)d[0] << 8) | d[1];
            uint16_t sr = ((uint16_t)d[3] << 8) | d[4];
            temp_c = -45.0f + 175.0f * st / 65535.0f;
            humidity = 100.0f * sr / 65535.0f;
        }
        if (humidity < 0) humidity = 0;
        if (humidity > 100) humidity = 100;
        return true;
    }

    bool read_pressure(float &pressure_hpa) {
        if (bus_ == nullptr)
            return false;
        uint8_t c = 0x06;                              // HP203B soft reset
        bus_->write(0x76, &c, 1);
        delay(2);
        c = 0x48;                                       // ADC_CVT: P + T, OSR=2
        bus_->write(0x76, &c, 1);
        delay(50);
        c = 0x30;                                       // READ_P
        bus_->write(0x76, &c, 1);
        uint8_t p[3];
        if (bus_->read(0x76, p, 3) != i2c::ERROR_OK)
            return false;
        if (p[0] >= 2 || (p[0] | p[1]) == 0)            // invalid / zero
            return false;
        uint32_t raw = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
        pressure_hpa = (float)(raw + 5) / 100.0f;
        return pressure_hpa > 300.0f && pressure_hpa < 1200.0f;
    }

    uint8_t th_type() const { return th_type_; }

 private:
    i2c::I2CBus *bus_ = nullptr;
    uint8_t th_type_ = 0;
};

}  // namespace ecowitt_rf
}  // namespace esphome
