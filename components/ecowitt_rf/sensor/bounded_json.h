// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cmath>
#include <cstdio>
#include <cstring>
#include <type_traits>

namespace esphome::ecowitt_rf {

// Counting pass + bounded write pass: no JSON DOM or growing strings on the ESP8266.
class BoundedJson {
 public:
    BoundedJson(char *buffer = nullptr, size_t capacity = 0) : buffer_(buffer), capacity_(capacity) {}
    void object(const char *key = nullptr) { prefix(key); put('{'); first_[depth_++] = true; }
    void array(const char *key) { prefix(key); put('['); first_[depth_++] = true; }
    void end_object() { --depth_; put('}'); }
    void end_array() { --depth_; put(']'); }
    void field(const char *key, const char *value) { prefix(key); quoted(value ? value : ""); }
    void field(const char *key, bool value) { prefix(key); raw(value ? "true" : "false"); }
    template<typename T, typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, bool>::value, int>::type = 0>
    void field(const char *key, T value) {
        prefix(key); char text[24];
        if constexpr (std::is_signed<T>::value) std::snprintf(text, sizeof(text), "%lld", (long long)value);
        else std::snprintf(text, sizeof(text), "%llu", (unsigned long long)value);
        raw(text);
    }
    void field(const char *key, float value) {
        prefix(key); char text[32];
        if (!std::isfinite(value)) { raw("null"); return; }
        std::snprintf(text, sizeof(text), "%.7g", (double)value); raw(text);
    }
    size_t size() const { return size_; }
    bool fits() const { return !buffer_ || size_ <= capacity_; }
 private:
    void put(char c) { if (buffer_ && size_ < capacity_) buffer_[size_] = c; ++size_; }
    void raw(const char *s) { while (*s) put(*s++); }
    void quoted(const char *s) {
        put('"');
        for (const unsigned char *p = (const unsigned char *)s; *p; ++p) {
            if (*p == '"' || *p == '\\') { put('\\'); put((char)*p); }
            else if (*p < 0x20) {
                static constexpr char hex[] = "0123456789abcdef";
                raw("\\u00"); put(hex[*p >> 4]); put(hex[*p & 15]);
            } else put((char)*p);
        }
        put('"');
    }
    void prefix(const char *key) {
        if (depth_) { if (!first_[depth_ - 1]) put(','); first_[depth_ - 1] = false; }
        if (key) { quoted(key); put(':'); }
    }
    char *buffer_;
    size_t capacity_, size_ = 0;
    bool first_[8] = {};
    unsigned depth_ = 0;
};

// Retain space for the asynchronous response/TCP bookkeeping and normal RF uploads.
inline bool state_response_fits(size_t bytes, size_t free_heap, size_t largest_block) {
    constexpr size_t reserve = 5500;
    return bytes <= 8192 && free_heap >= reserve && bytes <= free_heap - reserve &&
           bytes + 32 <= largest_block;
}

}  // namespace esphome::ecowitt_rf
