#include "passthrough.h"
#include "crsf.h"       // for CRSF_TX_PIN
#include "web.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

// ── DTR state tracking ────────────────────────────────────────────────────────
// On USB-CDC boards (ESP32-S2 with TinyUSB) DTR is readable via a USBCDC
// event callback.  On UART-bridge boards (WROOM 32) the bridge chip owns the
// DTR pin; we can't read it in software, so forwarding is skipped and the user
// must reset the ELRS receiver manually before flashing.
static volatile bool g_dtr = false;

#if ARDUINO_USB_CDC_ON_BOOT
#include <USBCDC.h>
static void on_cdc_line_state(void* /*arg*/, esp_event_base_t /*base*/,
                              int32_t /*id*/, void* event_data) {
    auto* ev = static_cast<arduino_usb_cdc_event_data_t*>(event_data);
    g_dtr = ev->line_state.dtr;
}
#endif

// ── CLI line buffer ───────────────────────────────────────────────────────────
// We accumulate printable characters into a ring buffer and scan for the
// "serialpassthrough" command after each newline.
#define CLI_BUF_SIZE 128
static char  cli_buf[CLI_BUF_SIZE];
static uint8_t cli_len = 0;

// ── DTR / reset forwarding ────────────────────────────────────────────────────
// The S2 Mini uses native USB CDC (no external USB-UART chip).
// The Arduino HWCDC class exposes Serial.dtr() directly, so we can read the
// DTR state that esptool asserts to trigger bootloader entry on the ELRS RX.
// When DTR falls (asserted), we briefly pull GPIO17 LOW to reset the ELRS RX.

// CRSF_TX_PIN comes from crsf.h — it is also the ELRS-RX reset line

// ── Passthrough bridge (blocking, never returns) ──────────────────────────────
// Reconfigures Serial1 to the requested baud rate, then enters a tight
// bidirectional byte-copy loop identical to real Betaflight passthrough.
// esptool (embedded in ELRS Configurator) drives firmware flashing through this
// bridge.
static bool bootloader_probe(uint32_t baud, uint32_t window_ms) {
    // Sample any receiver output for a short window to see if it reacts to reset
    const uint32_t start = millis();
    const uint32_t inter_byte_ms = 5;

    uint8_t buf[24];
    uint8_t n = 0;
    uint32_t last_byte_ms = millis();

    while (millis() - start < window_ms) {
        while (Serial1.available() && n < sizeof(buf)) {
            buf[n++] = (uint8_t)Serial1.read();
            last_byte_ms = millis();
        }
        // Exit early if we got some bytes and the line went idle briefly
        if (n > 0 && (millis() - last_byte_ms) > inter_byte_ms) break;
        delay(1);
    }

    if (n == 0) {
    web_logf("bootprobe: no bytes @ %lu", (unsigned long)baud);
    return false;
    }

    char line[96];
    char* p = line;
    p += snprintf(p, sizeof(line), "bootprobe: bytes=%u @ %lu data=",
                  (unsigned)n, (unsigned long)baud);
    for (uint8_t i = 0; i < n && (p - line) < (int)sizeof(line) - 4; i++) {
        p += snprintf(p, (size_t)(line + sizeof(line) - p), "%02X ", buf[i]);
    }
    web_log(line);
    return true;
}

static bool bootloader_probe_multi(uint32_t baud) {
    // Probe at passthrough baud first
    bootloader_probe(baud, 300);
    // Then probe at 115200 (ESP ROM boot chatter is commonly 115200)
    Serial1.end();
    Serial1.begin(115200, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    delay(5);
    bool ok_115200 = bootloader_probe(115200, 600);
    // Restore passthrough baud
    Serial1.end();
    Serial1.begin(baud, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    delay(5);
    return ok_115200;
}

static void pulse_rx_reset(uint32_t low_ms, uint32_t settle_ms) {
    Serial1.end();
    pinMode(CRSF_TX_PIN, OUTPUT);
    digitalWrite(CRSF_TX_PIN, LOW);
    delay(low_ms);
    Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    delay(settle_ms);
}

static void enter_passthrough(uint8_t uart_index, uint32_t baud) {
    // Only UART index 1 is wired to the ELRS receiver
    if (uart_index != 1) return;

    // Flush any pending data, then reconfigure Serial1 at the target baud
    Serial1.flush();
    Serial1.end();
    Serial1.begin(baud, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    // Small settling delay is acceptable — we are already in blocking mode
    delay(10);

    // Snapshot initial DTR state from native USB CDC
    bool last_dtr = g_dtr;

#if !ARDUINO_USB_CDC_ON_BOOT
    // UART-bridge boards can't read DTR. Pulse reset once on entry so the
    // ELRS receiver can enter bootloader mode for flashing.
    pulse_rx_reset(10, 20);
    web_log("passthrough: reset pulse 10ms");
#else
    web_logf("passthrough: DTR=%s", g_dtr ? "HIGH" : "LOW");
#endif
    web_logf("passthrough: entered uart=%u baud=%lu", uart_index, (unsigned long)baud);
    bool boot_115200 = bootloader_probe_multi(baud);
    if (boot_115200 && baud != 115200) {
        web_log("passthrough: bootloader detected @115200, switching baud");
        Serial1.end();
        Serial1.begin(115200, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
        baud = 115200;
        delay(5);
    }
    web_log("passthrough: web updates enabled");

    // ── Permanent bridge loop ─────────────────────────────────────────────────
    // Every byte received from USB goes to the ELRS receiver, and vice versa.
    // DTR changes reported by the USB CDC host are forwarded as a brief reset
    // pulse on GPIO17 so that esptool can trigger bootloader entry on the ELRS
    // receiver automatically — exactly as real Betaflight does.
    uint32_t last_web_ms = millis();
    while (true) {
        // USB → ELRS RX
        while (Serial.available()) {
            Serial1.write((uint8_t)Serial.read());
        }
        // ELRS RX → USB
        while (Serial1.available()) {
            Serial.write((uint8_t)Serial1.read());
        }

        // DTR falling edge (HIGH→LOW) = esptool asserted DTR = reset ELRS RX.
        // Briefly reclaim GPIO17 as a plain output, pull it LOW for ~1 ms,
        // then restore Serial1.  The ELRS receiver sees its RX pin go low and
        // enters bootloader mode.
        bool dtr_now = g_dtr;
        if (last_dtr && !dtr_now) {
            Serial1.end();
            pinMode(CRSF_TX_PIN, OUTPUT);
            digitalWrite(CRSF_TX_PIN, LOW);
            delay(1);
            Serial1.begin(baud, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
        }
        last_dtr = dtr_now;

        // Keep WiFi/WebServer alive so logs remain visible during passthrough.
        uint32_t now = millis();
        if (now - last_web_ms >= 5) {
            web_update();
            last_web_ms = now;
        }
        delay(0);  // yield to WiFi/RTOS
    }
    // Never reached — matches real Betaflight behaviour.
    // Power-cycle the S2 Mini to exit passthrough.
}

// ── CLI line parser ───────────────────────────────────────────────────────────
// Betaflight Configurator / ELRS Configurator enters CLI mode first ('#'),
// then issues: serialpassthrough <uart_index> <baud>
// We scan every completed line for that pattern.
// Send a CLI response line followed by the '# ' prompt.
// The ELRS flasher uses '# ' as a read delimiter, so every response must end with it.
static void cli_reply(const char* msg) {
    Serial.print(msg);
    Serial.print("\r\n# ");
    Serial.flush();
}

static void process_cli_line(const char* line) {
    // Trim leading whitespace
    while (*line == ' ' || *line == '\t') line++;
    if (*line) web_logf("cli: line '%s'", line);

    // ── get <param> ───────────────────────────────────────────────────────────
    // ELRS flasher queries these three params to validate CRSF configuration
    // before attempting passthrough.  Each response must contain ' = VALUE'
    // and end with '# ' (used as a read delimiter).
    if (strncmp(line, "get ", 4) == 0) {
        const char* param = line + 4;
        if (strcmp(param, "serialrx_provider") == 0) {
            cli_reply("serialrx_provider = CRSF");
        } else if (strcmp(param, "serialrx_inverted") == 0) {
            cli_reply("serialrx_inverted = OFF");
        } else if (strcmp(param, "serialrx_halfduplex") == 0) {
            cli_reply("serialrx_halfduplex = OFF");
        } else if (strcmp(param, "rx_spi_protocol") == 0) {
            cli_reply("rx_spi_protocol = OFF");
        } else {
            cli_reply("");  // unknown param — send empty reply + prompt
        }
        web_logf("cli: get %s", param);
        return;
    }

    // ── serial ────────────────────────────────────────────────────────────────
    // ELRS flasher parses the output looking for a UART with function mask
    // bit 6 set (64 = RX_SERIAL).  Format: serial <id> <mask> 0 0 0 0
    // A line containing '#' terminates the read loop in the flasher.
    if (strcmp(line, "serial") == 0) {
        Serial.print("serial 1 64 0 0 0 0\r\n# ");
        Serial.flush();
        web_log("cli: serial");
        return;
    }

    // ── serialpassthrough <index> <baud> ──────────────────────────────────────
    if (strncmp(line, "serialpassthrough", 17) == 0) {
        const char* args = line + 17;
        while (*args == ' ' || *args == '\t') args++;

        char* end;
        long uart_index = strtol(args, &end, 10);
        if (end == args) {
            web_log("cli: passthrough parse fail (uart)");
            return;
        }

        while (*end == ' ' || *end == '\t') end++;
        long baud = strtol(end, NULL, 10);
        if (baud <= 0) baud = 420000;

        Serial.printf("\r\nEntering serial passthrough on UART%ld @ %ld baud\r\n",
                      uart_index, baud);
        Serial.flush();
        web_logf("cli: passthrough uart=%ld baud=%ld", uart_index, baud);

        enter_passthrough((uint8_t)uart_index, (uint32_t)baud);
        // Does not return
    }

    // ── elrsboot ─────────────────────────────────────────────────────────────
    if (strcmp(line, "elrsboot") == 0) {
        web_log("cli: elrsboot");
        cli_reply("ok");
        pulse_rx_reset(50, 50);
        web_log("elrsboot: pulsed 50ms");
        // Probe immediately at both baud rates
        Serial1.end();
        Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
        delay(5);
        bootloader_probe_multi(CRSF_BAUDRATE);
        return;
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

void passthrough_init() {
    cli_len = 0;
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.onEvent(ARDUINO_USB_CDC_LINE_STATE_EVENT, on_cdc_line_state);
#endif
}

void passthrough_process_byte(uint8_t b) {
    // '#' triggers CLI mode entry in real Betaflight — respond immediately with
    // the welcome banner and the '# ' prompt.  ELRS Configurator sends '#\n'
    // and waits to see '# ' before it sends the serialpassthrough command.
    // Without this reply it prints "No CLI available" and aborts.
    if (b == '#') {
        Serial.print("\r\nEntering CLI Mode, type 'exit' to return, or 'help'\r\n\r\n# ");
        Serial.flush();
        cli_len = 0;
        web_log("cli: enter");
        return;
    }

    // Ignore non-printable bytes below space, except CR and LF which end the line.
    // This naturally ignores binary MSP frame bytes without disrupting parsing.
    if (b == '\n' || b == '\r') {
        if (cli_len > 0) {
            cli_buf[cli_len] = '\0';
            process_cli_line(cli_buf);
            cli_len = 0;
        }
        return;
    }
    if (b < 0x20 || b > 0x7E) return;  // skip non-printable

    if (cli_len < CLI_BUF_SIZE - 1) {
        cli_buf[cli_len++] = (char)b;
    } else {
        // Buffer overflow — reset (prevents memory issues from garbage data)
        cli_len = 0;
    }
}
