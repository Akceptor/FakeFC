#include "crsf.h"
#include <Arduino.h>
#include <AlfredoCRSF.h>

static AlfredoCRSF crsf;
static uint32_t    frame_count = 0;

void crsf_init() {
    Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    crsf.begin(Serial1);
}

void crsf_update() {
    crsf.update();
    if (crsf.isLinkUp()) frame_count++;
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
