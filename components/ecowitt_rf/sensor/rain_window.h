// SPDX-License-Identifier: GPL-3.0-only
// Bounded rolling rainfall, independent of packet cadence and counter resets.
#pragma once
#include <cstdint>
#include <cmath>

namespace esphome {
namespace ecowitt_rf {

template<uint8_t N, uint32_t INTERVAL_MS> class RainWindow {
 public:
    static constexpr uint32_t SLOT_MS = INTERVAL_MS;
    static constexpr uint8_t SLOTS = N;

    struct State {
        uint16_t amounts[N] = {};
        uint32_t previous_counter = 0, phase_ms = 0;
        uint8_t head = 0;
        bool initialized = false;
    };
    State state() const {
        State s;
        for (uint8_t i = 0; i < N; ++i) s.amounts[i] = amounts_[i];
        s.previous_counter = previous_counter_; s.phase_ms = phase_ms_;
        s.head = head_; s.initialized = initialized_;
        return s;
    }
    bool restore(const State &s, uint32_t elapsed_ms, uint32_t now_ms) {
        if (s.head >= N || s.phase_ms >= SLOT_MS) return false;
        for (uint8_t i = 0; i < N; ++i) amounts_[i] = s.amounts[i];
        previous_counter_ = s.previous_counter; phase_ms_ = s.phase_ms;
        head_ = s.head; initialized_ = s.initialized;
        last_ms_ = now_ms - elapsed_ms;
        sample(previous_counter_ * 0.1f, now_ms);
        return true;
    }

    float sample(float counter_mm, uint32_t now_ms) {
        if (!std::isfinite(counter_mm) || counter_mm < 0) return total_mm();
        const uint32_t counter = (uint32_t)std::lround(counter_mm * 10.0f);
        if (!initialized_) {
            initialized_ = true;
            previous_counter_ = counter;
            last_ms_ = now_ms;
            return 0;
        }
        const uint32_t elapsed = now_ms - last_ms_;  // millis() wrap-safe
        last_ms_ = now_ms;
        if (elapsed >= SLOTS * SLOT_MS) {
            for (auto &amount : amounts_) amount = 0;
            phase_ms_ = 0;
            // Cannot date rain accumulated during a gap longer than this window.
            previous_counter_ = counter;
            return 0;
        } else {
            phase_ms_ += elapsed;
            while (phase_ms_ >= SLOT_MS) {
                phase_ms_ -= SLOT_MS;
                head_ = (uint8_t)((head_ + 1) % SLOTS);
                amounts_[head_] = 0;
            }
        }
        // Reset/wrap changes the baseline but does not erase rain already observed.
        if (counter >= previous_counter_) {
            uint32_t amount = amounts_[head_] + counter - previous_counter_;
            amounts_[head_] = (uint16_t)(amount > 65535 ? 65535 : amount);
        }
        previous_counter_ = counter;
        return total_mm();
    }

 private:
    float total_mm() const {
        uint32_t total = 0;
        for (auto amount : amounts_) total += amount;
        return total * 0.1f;
    }
    uint16_t amounts_[SLOTS] = {};
    uint32_t previous_counter_ = 0, last_ms_ = 0, phase_ms_ = 0;
    uint8_t head_ = 0;
    bool initialized_ = false;
};

// Keep the boundary bucket: rainfall expires 24h to 24h30m after receipt.
// 98 bytes of amounts per sensor, with no heap or flash allocations.
using RainWindow24h = RainWindow<49, 1800000UL>;
// Ten one-minute buckets: an increment remains visible for 9–10 minutes.
using RainWindow10m = RainWindow<10, 60000UL>;

}  // namespace ecowitt_rf
}  // namespace esphome
