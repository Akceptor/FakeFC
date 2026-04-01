#include <Arduino.h>
#include "crsf.h"
#include "msp.h"
#include "passthrough.h"
#include "web.h"

// ── Pin / baud summary ────────────────────────────────────────────────────────
// Serial  (UART0)  GPIO1/3  115200   USB ↔ Betaflight Configurator (MSP + CLI)
// Serial1 (UART1)  GPIO16/17 420000  ELRS receiver (CRSF RC channels)
//
// To flash the ELRS receiver:
//   1. Open ELRS Configurator → select "Betaflight Passthrough" method
//   2. ELRS Configurator finds UART1 via MSP_CF_SERIAL_CONFIG (function 64)
//   3. It sends:  serialpassthrough 1 420000
//   4. Firmware enters a permanent byte-bridge loop — power-cycle ESP32 to exit
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);  // USB serial: MSP + CLI
    crsf_init();           // Serial1 at 420000 on GPIO25/26
    msp_init();
    passthrough_init();
    web_init();            // WiFi AP "FakeFC" → http://192.168.4.1
}

void loop() {
    // Read every incoming USB byte and feed it to both parsers.
    // The MSP parser ignores printable/text bytes; the passthrough parser
    // ignores binary MSP bytes.  Both see the full stream so neither misses
    // its own protocol.
    while (Serial.available()) {
        uint8_t b = (uint8_t)Serial.read();
        msp_process_byte(b);       // MSP v1 state machine → sends responses
        passthrough_process_byte(b); // CLI scanner → may enter bridge (blocking)
    }

    // Parse any CRSF frames that arrived from the ELRS receiver since last loop.
    // Freshly decoded channels are immediately available to msp_process_byte
    // via crsf_get_channels() the next time MSP_RC is requested.
    crsf_update();
    web_update();
}
