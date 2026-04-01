#pragma once
#include <stdint.h>

// ── Wiring — change these to match your board layout ─────────────────────────
#define CRSF_RX_PIN  25   // GPIO connected to ELRS receiver TX
#define CRSF_TX_PIN  26   // GPIO connected to ELRS receiver RX (also reset line)
// ─────────────────────────────────────────────────────────────────────────────

#define CRSF_CHANNEL_COUNT 16

#define CRSF_BAUDRATE 420000

// Initialize Serial1 at 420000 baud on CRSF_RX_PIN / CRSF_TX_PIN
void crsf_init();

// Call every loop iteration — reads and parses incoming CRSF frames non-blockingly
void crsf_update();

// Returns pointer to 16-element array of channel values in 1000–2000 µs range
const uint16_t* crsf_get_channels();

// Returns total raw bytes received on Serial1 (never resets).
// Use to distinguish "no bytes" (wiring/power) from "bytes but no valid frames" (protocol).
uint32_t crsf_get_bytes_received();

// Returns number of successfully decoded RC frames since boot.
uint32_t crsf_get_frame_count();
