// SPDX-License-Identifier: GPL-3.0-only
#include <cassert>
#include <cstring>
#include <string>
#include "../components/ecowitt_rf/sensor/owned_response.h"
using namespace esphome::ecowitt_rf;
int main() {
    const std::string body(2400, 'x');
    auto *payload = static_cast<char *>(std::malloc(body.size()));
    std::memcpy(payload, body.data(), body.size());
    bool active = true;
    auto *response = new OwnedJsonResponse(payload, body.size(), &active);
    AsyncWebServerRequest request;
    response->_respond(&request); // initial zero-byte write; no premature completion
    assert(!response->finished());
    for (size_t i = 0; !response->finished() && i < 1000; ++i) {
        request.socket.budget = i % 3 == 0 ? 0 : 7; // fragmented headers/body + backpressure
        response->_ack(&request, i ? 7 : 0, 0);
        if (response->finished()) assert(request.socket.bytes.size() == body.size() + 19);
    }
    assert(response->finished());
    assert(request.socket.bytes == "HTTP/1.1 200 OK\r\n\r\n" + body);
    assert(active); delete response; assert(!active);
    std::puts("Owned response survives short/zero writes and releases its request gate");
}
