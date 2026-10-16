// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <cstdlib>
#include <ESPAsyncWebServer.h>

namespace esphome::ecowitt_rf {

// One owned payload; no intermediate response-sized or two-MSS staging buffer.
// A short/zero socket write retains the unsent suffix for the next ACK/poll.
class OwnedJsonResponse : public AsyncWebServerResponse {
 public:
    OwnedJsonResponse(char *body, size_t length, bool *active) : body_(body), active_(active) {
        _code = 200; _contentType = "application/json"; _contentLength = length;
        addHeader("Cache-Control", "no-store"); addHeader("Connection", "close");
    }
    ~OwnedJsonResponse() override { std::free(body_); *active_ = false; }
    bool _sourceValid() const override { return body_ != nullptr; }
    void _respond(AsyncWebServerRequest *request) override {
        _assembleHead(headers_, request->version());
        _state = RESPONSE_HEADERS;
        flush_(request);
    }
    size_t _ack(AsyncWebServerRequest *request, size_t len, uint32_t) override {
        _ackedLength += len;
        return flush_(request);
    }
 private:
    size_t flush_(AsyncWebServerRequest *request) {
        size_t accepted = 0;
        if (_state == RESPONSE_HEADERS) {
            size_t n = request->client()->write(headers_.c_str() + header_offset_, headers_.length() - header_offset_);
            header_offset_ += n; _writtenLength += n; accepted += n;
            if (header_offset_ != headers_.length()) return accepted;
            headers_ = String(); _state = RESPONSE_CONTENT;
        }
        if (_state == RESPONSE_CONTENT) {
            size_t n = request->client()->write(body_ + _sentLength, _contentLength - _sentLength);
            _sentLength += n; _writtenLength += n; accepted += n;
            if (_sentLength == _contentLength) _state = RESPONSE_END;
        }
        return accepted;
    }
    char *body_;
    bool *active_;
    String headers_;
    size_t header_offset_ = 0;
};

}  // namespace esphome::ecowitt_rf
