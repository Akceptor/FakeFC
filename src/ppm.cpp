#include "ppm.h"
#include "crsf.h"  // for CRSF_RX_PIN (shared pin)
#include <Arduino.h>

#define PPM_CHANNEL_COUNT 16
#define PPM_SYNC_US 3000
#define PPM_MIN_US 700
#define PPM_MAX_US 2300

static volatile uint16_t ppm_channels[PPM_CHANNEL_COUNT];
static volatile uint8_t  ppm_index = 0;
static volatile uint32_t ppm_last_us = 0;
static bool ppm_enabled = false;

static portMUX_TYPE ppm_mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR ppm_isr() {
    uint32_t now = micros();
    uint32_t dt = now - ppm_last_us;
    ppm_last_us = now;

    if (dt > PPM_SYNC_US) {
        ppm_index = 0;
        return;
    }
    if (dt < PPM_MIN_US || dt > PPM_MAX_US) return;

    if (ppm_index < PPM_CHANNEL_COUNT) {
        portENTER_CRITICAL_ISR(&ppm_mux);
        ppm_channels[ppm_index] = (uint16_t)dt;
        portEXIT_CRITICAL_ISR(&ppm_mux);
        ppm_index++;
    }
}

void ppm_init() {
    for (uint8_t i = 0; i < PPM_CHANNEL_COUNT; i++) ppm_channels[i] = 1500;
}

void ppm_set_enabled(bool enable) {
    if (enable == ppm_enabled) return;
    ppm_enabled = enable;

    if (enable) {
        pinMode(CRSF_RX_PIN, INPUT);
        ppm_index = 0;
        ppm_last_us = micros();
        attachInterrupt(digitalPinToInterrupt(CRSF_RX_PIN), ppm_isr, RISING);
    } else {
        detachInterrupt(digitalPinToInterrupt(CRSF_RX_PIN));
    }
}

bool ppm_is_enabled() { return ppm_enabled; }

const uint16_t* ppm_get_channels() {
    static uint16_t out[PPM_CHANNEL_COUNT];
    portENTER_CRITICAL(&ppm_mux);
    for (uint8_t i = 0; i < PPM_CHANNEL_COUNT; i++) {
        uint16_t v = ppm_channels[i];
        out[i] = (v >= 900 && v <= 2100) ? v : 1500;
    }
    portEXIT_CRITICAL(&ppm_mux);
    return out;
}
