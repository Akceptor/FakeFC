#include "rx.h"
#include "crsf.h"
#include "ppm.h"
#include <Arduino.h>
#include <Preferences.h>

// Serial RX provider values (Betaflight)
#define SERIALRX_PROVIDER_CRSF 9

static Preferences prefs;
static uint32_t feature_mask = FEATURE_RX_SERIAL;
static uint8_t  serial_provider = SERIALRX_PROVIDER_CRSF;

static void rx_apply_features() {
    bool want_ppm = (feature_mask & FEATURE_RX_PPM) != 0;
    bool want_serial = (feature_mask & FEATURE_RX_SERIAL) != 0;

    if (!want_ppm && !want_serial) {
        want_serial = true;
        feature_mask |= FEATURE_RX_SERIAL;
    }
    if (want_ppm && want_serial) {
        // Prefer serial if both set
        want_ppm = false;
        feature_mask &= ~FEATURE_RX_PPM;
    }

    crsf_set_enabled(want_serial);
    ppm_set_enabled(want_ppm);
}

void rx_init() {
    prefs.begin("fakefc", false);
    feature_mask = prefs.getUInt("feat", FEATURE_RX_SERIAL);
    serial_provider = prefs.getUChar("srxprov", SERIALRX_PROVIDER_CRSF);

    crsf_init();
    ppm_init();
    rx_apply_features();
}

void rx_update() {
    crsf_update();
}

const uint16_t* rx_get_channels() {
    return rx_ppm_enabled() ? ppm_get_channels() : crsf_get_channels();
}

uint32_t rx_get_feature_mask() { return feature_mask; }

void rx_set_feature_mask(uint32_t mask) {
    feature_mask = mask;
    rx_apply_features();
    prefs.putUInt("feat", feature_mask);
}

uint8_t rx_get_serial_provider() { return serial_provider; }

void rx_set_serial_provider(uint8_t provider) {
    serial_provider = provider;
    prefs.putUChar("srxprov", serial_provider);
}

bool rx_serial_enabled() { return (feature_mask & FEATURE_RX_SERIAL) != 0; }
bool rx_ppm_enabled() { return (feature_mask & FEATURE_RX_PPM) != 0; }

uint32_t rx_get_bytes_received() {
    return rx_serial_enabled() ? crsf_get_bytes_received() : 0;
}

uint32_t rx_get_frame_count() {
    return rx_serial_enabled() ? crsf_get_frame_count() : 0;
}
