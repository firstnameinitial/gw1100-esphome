// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>

class String {
 public:
    String() = default;
    String(const char *s) : text_(s) {}
    const char *c_str() const { return text_.c_str(); }
    size_t length() const { return text_.size(); }
 private: std::string text_;
};
class AsyncClient {
 public:
    size_t budget = 0;
    std::string bytes;
    size_t write(const char *p, size_t size) {
        size_t n = std::min(size, budget); bytes.append(p, n); budget -= n; return n;
    }
};
class AsyncWebServerRequest {
 public:
    AsyncClient socket;
    AsyncClient *client() { return &socket; }
    uint8_t version() { return 1; }
};
enum { RESPONSE_SETUP, RESPONSE_HEADERS, RESPONSE_CONTENT, RESPONSE_WAIT_ACK, RESPONSE_END };
class AsyncWebServerResponse {
 public:
    virtual ~AsyncWebServerResponse() = default;
    virtual void _respond(AsyncWebServerRequest *) = 0;
    virtual size_t _ack(AsyncWebServerRequest *, size_t, uint32_t) = 0;
    virtual bool _sourceValid() const = 0;
    bool finished() const { return _state == RESPONSE_END; }
    void addHeader(const char *, const char *) {}
 protected:
    void _assembleHead(String &head, uint8_t) { head = String("HTTP/1.1 200 OK\r\n\r\n"); }
    int _code = 0, _state = RESPONSE_SETUP;
    String _contentType;
    size_t _contentLength = 0, _writtenLength = 0, _sentLength = 0, _ackedLength = 0;
};
