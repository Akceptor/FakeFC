#include "crsf.h"
#include "web.h"
#include <Arduino.h>
#include <AlfredoCRSF.h>

static AlfredoCRSF crsf;
static uint32_t    frame_count = 0;
static bool        last_link = false;
static uint32_t    last_log_ms = 0;
static bool        crsf_enabled = false;

void crsf_init() {
    Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    crsf.begin(Serial1);
    web_logf("crsf: init rx=%d tx=%d baud=%d", CRSF_RX_PIN, CRSF_TX_PIN, CRSF_BAUDRATE);
    crsf_enabled = true;
}

void crsf_update() {
    if (!crsf_enabled) return;
    crsf.update();
    bool link = crsf.isLinkUp();
    if (link) frame_count++;

    if (link != last_link) {
        web_logf("crsf: link %s", link ? "up" : "down");
        last_link = link;
    }

    uint32_t now = millis();
    if (now - last_log_ms >= 5000) {
        web_logf("crsf: frames=%lu", (unsigned long)frame_count);
        last_log_ms = now;
    }
}

const uint16_t* crsf_get_channels() {
    // getChannel() returns µs directly (1000–2000), cast pointer via temp array
    static uint16_t channels[CRSF_CHANNEL_COUNT];
    for (uint8_t i = 0; i < CRSF_CHANNEL_COUNT; i++) {
        int v = crsf.getChannel(i + 1);
        channels[i] = (v > 0) ? (uint16_t)constrain(v, 1000, 2000) : 1500;
    }
    return channels;
}

uint32_t crsf_get_frame_count()     { return frame_count; }
uint32_t crsf_get_bytes_received()  { return frame_count * 26; }

void crsf_set_enabled(bool enable) {
    if (enable == crsf_enabled) return;
    crsf_enabled = enable;
    if (enable) {
        Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
        crsf.begin(Serial1);
        web_log("crsf: enabled");
    } else {
        Serial1.end();
        web_log("crsf: disabled");
    }
}

bool crsf_is_enabled() { return crsf_enabled; }
