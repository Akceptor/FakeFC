#include "msp.h"
#include "rx.h"
#include <Arduino.h>
#include <string.h>

// ── MSP v1 command IDs ────────────────────────────────────────────────────────
#define MSP_API_VERSION          1
#define MSP_FC_VARIANT           2
#define MSP_FC_VERSION           3
#define MSP_BOARD_INFO           4
#define MSP_BUILD_INFO           5
#define MSP_NAME                 10
#define MSP_FEATURE_CONFIG       36
#define MSP_SET_FEATURE_CONFIG   37
#define MSP_RX_MAP               64
#define MSP_CF_SERIAL_CONFIG     54
#define MSP_SET_CF_SERIAL_CONFIG 55
#define MSP_STATUS               101
#define MSP_RAW_IMU              102
#define MSP_RC                   105
#define MSP_ATTITUDE             108
#define MSP_DATAFLASH_SUMMARY    70
#define MSP_MIXER_CONFIG         42
#define MSP_SENSOR_ALIGNMENT     126
#define MSP_ANALOG               110
#define MSP_BOXNAMES             116
#define MSP_ACC_TRIM             240
#define MSP_RX_CONFIG            144
#define MSP_RX_CONFIG_V1         44
#define MSP_SET_RX_CONFIG        145
#define MSP_SET_RX_CONFIG_V1     45
#define MSP_STATUS_EX            150
#define MSP_UID                  160

// ── Parser state machine (MSP v1 + v2) ───────────────────────────────────────
static enum {
    // shared
    MSP_IDLE,           // waiting for '$'
    MSP_HEADER_M,       // got '$', waiting for 'M' (v1) or 'X' (v2)
    // MSP v1
    MSP_HEADER_DIR,     // got '$M', waiting for '<'
    MSP_HEADER_SIZE,    // waiting for payload size byte
    MSP_HEADER_CMD,     // waiting for command byte
    MSP_PAYLOAD,        // accumulating payload bytes
    MSP_CHECKSUM,       // waiting for XOR checksum
    // MSP v2
    MSP2_HEADER_DIR,    // got '$X', waiting for '<'
    MSP2_HEADER_FLAG,   // waiting for flag byte
    MSP2_HEADER_CODE0,  // waiting for command low byte
    MSP2_HEADER_CODE1,  // waiting for command high byte
    MSP2_HEADER_SIZE0,  // waiting for payload size low byte
    MSP2_HEADER_SIZE1,  // waiting for payload size high byte
    MSP2_PAYLOAD,       // accumulating payload bytes
    MSP2_CHECKSUM,      // waiting for CRC8/DVB-S2
} msp_state = MSP_IDLE;

#define MSP_MAX_PAYLOAD 64
// v1 state
static uint8_t  msp_size;
static uint8_t  msp_cmd;
static uint8_t  msp_payload[MSP_MAX_PAYLOAD];
static uint8_t  msp_payload_pos;
static uint8_t  msp_checksum_acc;
// v2 state
static uint16_t msp2_code;
static uint16_t msp2_size;
static uint16_t msp2_payload_pos;
static uint8_t  msp2_crc;

// ── Response helpers ──────────────────────────────────────────────────────────

// Build and send an MSP v1 response frame.
// cmd: the command being responded to
// payload / payload_len: response data (may be NULL / 0 for empty ACK)
static void msp_send(uint8_t cmd, const uint8_t* payload, uint8_t payload_len) {
    uint8_t csum = payload_len ^ cmd;
    for (uint8_t i = 0; i < payload_len; i++) csum ^= payload[i];

    Serial.write('$');
    Serial.write('M');
    Serial.write('>');
    Serial.write(payload_len);
    Serial.write(cmd);
    if (payload_len > 0) Serial.write(payload, payload_len);
    Serial.write(csum);
}

// CRC8/DVB-S2 single-byte update (same poly used by CRSF)
static inline uint8_t crc8_byte(uint8_t crc, uint8_t b) {
    crc ^= b;
    for (uint8_t i = 0; i < 8; i++)
        crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
    return crc;
}

// Build and send an MSP v2 response frame.
// CRC covers: flag(0) + code_lo + code_hi + size_lo + size_hi + payload
static void msp2_send(uint16_t cmd, const uint8_t* payload, uint16_t payload_len) {
    uint8_t crc = 0;
    crc = crc8_byte(crc, 0);                              // flag
    crc = crc8_byte(crc, (uint8_t)(cmd & 0xFF));
    crc = crc8_byte(crc, (uint8_t)(cmd >> 8));
    crc = crc8_byte(crc, (uint8_t)(payload_len & 0xFF));
    crc = crc8_byte(crc, (uint8_t)(payload_len >> 8));
    for (uint16_t i = 0; i < payload_len; i++) crc = crc8_byte(crc, payload[i]);

    Serial.write('$');
    Serial.write('X');
    Serial.write('>');
    Serial.write((uint8_t)0);                             // flag
    Serial.write((uint8_t)(cmd & 0xFF));
    Serial.write((uint8_t)(cmd >> 8));
    Serial.write((uint8_t)(payload_len & 0xFF));
    Serial.write((uint8_t)(payload_len >> 8));
    if (payload_len > 0) Serial.write(payload, payload_len);
    Serial.write(crc);
}

// Convenience: write a uint16 little-endian into a buffer and advance the pointer
static inline void w16(uint8_t*& p, uint16_t v) {
    *p++ = (uint8_t)(v & 0xFF);
    *p++ = (uint8_t)(v >> 8);
}

// Convenience: write a uint32 little-endian
static inline void w32(uint8_t*& p, uint32_t v) {
    *p++ = (uint8_t)(v & 0xFF);
    *p++ = (uint8_t)((v >> 8)  & 0xFF);
    *p++ = (uint8_t)((v >> 16) & 0xFF);
    *p++ = (uint8_t)((v >> 24) & 0xFF);
}

// ── Command dispatch ──────────────────────────────────────────────────────────

static void msp_handle(uint8_t cmd) {
    uint8_t buf[64];
    uint8_t* p = buf;

    switch (cmd) {

        // ── MSP_API_VERSION (1) ───────────────────────────────────────────────
        // Configurator 10.x requires API >= 1.44.0 to proceed past the
        // version check; 1.43.0 triggers showVersionMismatchAndCli() and
        // the 10-second timeout fires unconditionally.
        case MSP_API_VERSION:
            *p++ = 0;   // MSP protocol version
            *p++ = 1;   // API major
            *p++ = 44;  // API minor (matches BF 4.3, minimum accepted by configurator 10.x)
            msp_send(cmd, buf, p - buf);
            break;

        // ── MSP_FC_VARIANT (2) ────────────────────────────────────────────────
        // Four-character FC identifier. "BTFL" makes the configurator treat
        // this board as genuine Betaflight.
        case MSP_FC_VARIANT:
            memcpy(buf, "BTFL", 4);
            msp_send(cmd, buf, 4);
            break;

        // ── MSP_FC_VERSION (3) ────────────────────────────────────────────────
        // Firmware version reported to the configurator: 4.3.0
        // (API 1.44 was introduced in BF 4.3)
        case MSP_FC_VERSION:
            *p++ = 4;  // major
            *p++ = 3;  // minor
            *p++ = 0;  // patch
            msp_send(cmd, buf, p - buf);
            break;

        // ── MSP_BOARD_INFO (4) ────────────────────────────────────────────────
        // Board identifier + hardware metadata. We emit the minimum set of
        // fields that BF 4.2 configurator expects (identifier, rev, type,
        // capabilities, then length-prefixed name strings, then signature).
        case MSP_BOARD_INFO: {
            // 4-char board identifier
            memcpy(p, "ESP3", 4); p += 4;
            w16(p, 0);             // hardwareRevision
            *p++ = 0;              // boardType: 0 = FC
            *p++ = 0;              // targetCapabilities
            // targetName (length-prefixed string)
            *p++ = 5; memcpy(p, "ESP32", 5); p += 5;
            // boardName (length-prefixed string)
            *p++ = 5; memcpy(p, "ESP32", 5); p += 5;
            // manufacturerId (empty)
            *p++ = 0;
            // Signature: 32 zero bytes (unsigned firmware)
            memset(p, 0, 32); p += 32;
            *p++ = 0xFF;  // mcuTypeId: unknown
            *p++ = 0;     // configurationState
            // Fields added in API 1.44 (BF 4.3) — configurator reads these
            // when apiVersion >= 1.44; zero-reads beyond payload are safe but
            // providing them avoids any parser quirks.
            w16(p, 0);    // sampleRateHz (0 = not reported)
            w32(p, 0);    // configurationProblems (0 = none)
            msp_send(cmd, buf, p - buf);
            break;
        }

        // ── MSP_BUILD_INFO (5) ────────────────────────────────────────────────
        // Build date, time and git short revision — configurator displays these
        // in the status bar. Lengths match BF source: 11 + 8 + 7 = 26 bytes.
        case MSP_BUILD_INFO:
            memcpy(p, "Jan  1 2024", 11); p += 11;  // BUILD_DATE_LENGTH = 11
            memcpy(p, "00:00:00",     8); p +=  8;  // BUILD_TIME_LENGTH = 8
            memcpy(p, "0000000",      7); p +=  7;  // GIT_SHORT_REVISION_LENGTH = 7
            msp_send(cmd, buf, p - buf);
            break;

        // ── MSP_STATUS (101) ──────────────────────────────────────────────────
        // Basic FC status.  BF 4.3 returns 13 bytes; the configurator reads
        // cycleTime → i2cError → activeSensors → flightModeFlags → profile
        // → averageSystemLoadPercent and stops (extra bytes are harmless).
        case MSP_STATUS:
            w16(p, 1000);  // cycleTime µs
            w16(p, 0);     // i2cErrorCount
            w16(p, 0);     // activeSensors bitmask (none)
            w32(p, 0);     // flightModeFlags (disarmed)
            *p++ = 0;      // pidProfileIndex
            w16(p, 0);     // averageSystemLoadPercent
            msp_send(cmd, buf, p - buf);  // 13 bytes
            break;

        // ── MSP_STATUS_EX (150) ───────────────────────────────────────────────
        // Extended status including arming-disabled flags.
        // Layout (BF 4.3 / API 1.44): cycleTime(u16) + i2cError(u16) +
        // activeSensors(u16) + flightModeFlags(u32) + pidProfile(u8) +
        // averageLoad(u16) + numProfiles(u8) + armingDisabledFlags(u32) +
        // accelerometerAxis(u8)  → 19 bytes
        case MSP_STATUS_EX:
            w16(p, 1000);  // cycleTime µs
            w16(p, 0);     // i2cErrorCount
            w16(p, 0);     // activeSensors bitmask (none)
            w32(p, 0);     // flightModeFlags (disarmed)
            *p++ = 0;      // pidProfileIndex
            w16(p, 0);     // averageSystemLoadPercent (cpuload)
            *p++ = 3;      // numProfiles
            *p++ = 0;      // rateProfile
            *p++ = 0;      // flightModeByteCount (0 = no extra mode bytes follow)
            *p++ = 1;      // armingDisableCount (1 entry follows)
            w32(p, 0);     // armingDisableFlags (0 = not disabled)
            *p++ = 0;      // configStateFlag (no reboot needed)
            msp_send(cmd, buf, p - buf);  // 22 bytes
            break;

        // ── MSP_UID (160) ─────────────────────────────────────────────────────
        // 96-bit unique FC identifier (3 × u32 LE).  Required by the
        // configurator's processBuildConfiguration chain after MSP_STATUS;
        // missing response stalls the connection sequence past the FC info
        // step and causes the 10-second timeout to fire.
        case MSP_UID:
            w32(p, 0xDEADBEEF);
            w32(p, 0x12345678);
            w32(p, 0xCAFEBABE);
            msp_send(cmd, buf, p - buf);  // 12 bytes
            break;

        // ── MSP_NAME (10) ─────────────────────────────────────────────────────
        // Craft name string (no null terminator; length inferred from payload).
        // Configurator requests this during processCraftName() for API < 1.45.
        // An empty payload is valid (unnamed craft).
        case MSP_NAME:
            msp_send(cmd, NULL, 0);
            break;

        // ── MSP_RAW_IMU (102) ─────────────────────────────────────────────────
        // Raw sensor readings: 3× accel (i16, raw ADC), 3× gyro (i16, raw
        // ADC), 3× mag (i16, raw ADC).  Static zeros = sensor at rest, level.
        case MSP_RAW_IMU:
            for (uint8_t i = 0; i < 9; i++) w16(p, 0);  // acc+gyro+mag all zero
            msp_send(cmd, buf, p - buf);  // 18 bytes
            break;

        // ── MSP_RC (105) ──────────────────────────────────────────────────────
        // 16 RC channel values as uint16 LE.  The configurator polls this at
        // ~10 Hz to draw the live bar graph in the Receiver tab.
        case MSP_RC: {
            const uint16_t* ch = rx_get_channels();
            for (uint8_t i = 0; i < RX_CHANNEL_COUNT; i++) w16(p, ch[i]);
            msp_send(cmd, buf, p - buf);  // 32 bytes
            break;
        }

        // ── MSP_ATTITUDE (108) ────────────────────────────────────────────────
        // Computed attitude: roll i16 (tenths of °), pitch i16 (tenths of °),
        // heading i16 (°, 0–360).  Zeros = level, facing north.
        case MSP_ATTITUDE:
            w16(p, 0);  // roll
            w16(p, 0);  // pitch
            w16(p, 0);  // heading
            msp_send(cmd, buf, p - buf);  // 6 bytes
            break;

        // ── MSP_RX_CONFIG (44 / 144) ──────────────────────────────────────────
        // 32-byte layout for API 1.44.  serialrx_provider reflects saved config.
        case MSP_RX_CONFIG:
        case MSP_RX_CONFIG_V1:
            *p++ = rx_get_serial_provider();  // [0] serialrx_provider
            w16(p, 1900);   // [1-2]  stick_max
            w16(p, 1500);   // [3-4]  stick_center
            w16(p, 1100);   // [5-6]  stick_min
            *p++ = 0;       // [7]    spektrum_sat_bind
            w16(p, 885);    // [8-9]  rx_min_usec
            w16(p, 2115);   // [10-11] rx_max_usec
            *p++ = 0;       // [12]   deprecated rcInterpolation
            *p++ = 0;       // [13]   deprecated rcInterpolationInterval
            w16(p, 1300);   // [14-15] airModeActivateThreshold
            *p++ = 0;       // [16]   rxSpiProtocol
            w32(p, 0);      // [17-20] rxSpiId
            *p++ = 0;       // [21]   rxSpiRfChannelCount
            *p++ = 0;       // [22]   fpvCamAngleDegrees
            *p++ = 0;       // [23]   deprecated rcInterpolationChannels
            *p++ = 0;       // [24]   deprecated rcSmoothingType
            *p++ = 0;       // [25]   rcSmoothingSetpointCutoff
            *p++ = 0;       // [26]   rcSmoothingFeedforwardCutoff (< 1.47)
            *p++ = 0;       // [27]   deprecated field
            *p++ = 0;       // [28]   deprecated rcSmoothingDerivativeType
            *p++ = 0;       // [29]   usbCdcHidType
            *p++ = 0;       // [30]   rcSmoothingAutoFactor
            *p++ = 1;       // [31]   rcSmoothing: enabled
            msp_send(cmd, buf, p - buf);  // 32 bytes
            break;

        // ── MSP_FEATURE_CONFIG (36) ───────────────────────────────────────────
        // u32 bitmask of enabled features (includes receiver mode selection).
        case MSP_FEATURE_CONFIG:
            w32(p, rx_get_feature_mask());
            msp_send(cmd, buf, p - buf);  // 4 bytes
            break;

        // ── MSP_RX_MAP (64) ───────────────────────────────────────────────────
        // 8-byte channel mapping: value = source RC channel (0-based) for
        // each BF function slot (A/E/T/R/AUX1-4).  Default: identity map.
        case MSP_RX_MAP:
            for (uint8_t i = 0; i < 8; i++) *p++ = i;
            msp_send(cmd, buf, p - buf);  // 8 bytes
            break;

        // ── MSP_CF_SERIAL_CONFIG (54) ─────────────────────────────────────────
        // Reports UART port assignments.  Each record: uint8 identifier,
        // uint16 functionMask, uint8 mspBaud, uint8 gpsBaud,
        // uint8 telemetryBaud, uint8 blackboxBaud  (7 bytes per port).
        //
        // UART identifiers match Betaflight's serialPortIdentifier_e:
        //   0 = UART1, 1 = UART2, 2 = UART3, 20 = USB VCP
        //
        // Function bitmask bits (serialPortFunction_e):
        //   bit 0 (1)  = MSP
        //   bit 6 (64) = RX_SERIAL  ← ELRS Configurator looks for this
        //
        // Baud rate indices: 5 = 115200, 8 = 400000
        case MSP_CF_SERIAL_CONFIG:
            // Port 0 (UART1): MSP on USB/UART0 — function 1
            *p++ = 0;   w16(p, 1);   *p++ = 5; *p++ = 0; *p++ = 0; *p++ = 0;
            // Port 1 (UART2 / ESP32 Serial1): SerialRX + CRSF — function 64
            // ELRS Configurator will see this and send: serialpassthrough 1 <baud>
            *p++ = 1;   w16(p, rx_serial_enabled() ? 64 : 0);
            *p++ = 0; *p++ = 0; *p++ = 0; *p++ = 0;
            // Port 2 (UART3): unused
            *p++ = 2;   w16(p, 0);   *p++ = 0; *p++ = 0; *p++ = 0; *p++ = 0;
            msp_send(cmd, buf, p - buf);  // 21 bytes
            break;

        // ── MSP_SET_CF_SERIAL_CONFIG (55) ─────────────────────────────────────
        // The configurator sends this when the user applies serial port changes.
        // We accept and ACK it (no-op: UART1 is always SerialRX in this firmware).
        case MSP_SET_CF_SERIAL_CONFIG:
            msp_send(cmd, NULL, 0);
            break;

        // ── MSP_SET_FEATURE_CONFIG (37) ───────────────────────────────────────
        case MSP_SET_FEATURE_CONFIG:
            if (msp_size >= 4) {
                uint32_t mask = (uint32_t)msp_payload[0]
                              | ((uint32_t)msp_payload[1] << 8)
                              | ((uint32_t)msp_payload[2] << 16)
                              | ((uint32_t)msp_payload[3] << 24);
                rx_set_feature_mask(mask);
            }
            msp_send(cmd, NULL, 0);
            break;

        // ── MSP_SET_RX_CONFIG (45 / 145) ──────────────────────────────────────
        case MSP_SET_RX_CONFIG:
        case MSP_SET_RX_CONFIG_V1:
            if (msp_size >= 1) {
                rx_set_serial_provider(msp_payload[0]);
            }
            msp_send(cmd, NULL, 0);
            break;

        // ── MSP_MIXER_CONFIG (42) ─────────────────────────────────────────────
        // mixerMode(u8) + yaw_motors_reversed(u8).  Quadcopter X = mode 3.
        case MSP_MIXER_CONFIG:
            *p++ = 3;   // mixerMode: QUAD_X
            *p++ = 0;   // yaw_motors_reversed: no
            msp_send(cmd, buf, p - buf);
            break;

        // ── MSP_SENSOR_ALIGNMENT (126) ────────────────────────────────────────
        // Sensor orientation for gyro/acc/mag.  7 bytes for API 1.44.
        // align_gyro(u8) + align_acc(u8) + align_mag(u8) +
        // gyro_detection_flags(u8) + gyro_to_use(u8) +
        // gyro_1_align(u8) + gyro_2_align(u8)
        // Alignment enum: 0 = DEFAULT (no rotation)
        case MSP_SENSOR_ALIGNMENT:
            *p++ = 0;   // align_gyro: DEFAULT
            *p++ = 0;   // align_acc:  DEFAULT
            *p++ = 0;   // align_mag:  DEFAULT
            *p++ = 1;   // gyro_detection_flags: gyro 1 present
            *p++ = 0;   // gyro_to_use: gyro 1
            *p++ = 0;   // gyro_1_align: DEFAULT
            *p++ = 0;   // gyro_2_align: DEFAULT
            msp_send(cmd, buf, p - buf);
            break;

        // ── MSP_ACC_TRIM (240) ────────────────────────────────────────────────
        // Accelerometer trim: pitch(i16) + roll(i16).  Zero = no trim.
        case MSP_ACC_TRIM:
            w16(p, 0);  // pitch trim
            w16(p, 0);  // roll trim
            msp_send(cmd, buf, p - buf);
            break;

        // ── MSP_ANALOG (110) ──────────────────────────────────────────────────
        // Repurposed for CRSF diagnostics while debugging:
        //   vbat (u8, units of 0.1V) = frame_count & 0xFF  → visible as voltage
        //   rssi (u8, 0-255)         = bytes_received & 0xFF → visible as RSSI
        // If frame_count climbs → CRSF frames are decoding correctly.
        // If bytes_received climbs but frame_count stays 0 → parser issue.
        // If bytes_received stays 0 → no bytes from receiver (wiring/power).
        case MSP_ANALOG: {
            uint32_t bc = rx_get_bytes_received();
            uint32_t fc = rx_get_frame_count();
            *p++ = (uint8_t)(fc & 0xFF);   // vbat (0.1V units) = frame counter LSB
            w16(p, 0);                     // amperage
            *p++ = (uint8_t)(bc & 0xFF);   // rssi = byte counter LSB
            msp_send(cmd, buf, p - buf);
            break;
        }

        // ── Catch-all ─────────────────────────────────────────────────────────
        // Respond with empty payload so the configurator does not time out
        // waiting for a reply to commands we do not implement.
        default:
            msp_send(cmd, NULL, 0);
            break;
    }
}

// ── Byte-level state machine ──────────────────────────────────────────────────

void msp_init() {
    msp_state = MSP_IDLE;
}

void msp_process_byte(uint8_t b) {
    switch (msp_state) {
        case MSP_IDLE:
            if (b == '$') msp_state = MSP_HEADER_M;
            break;

        case MSP_HEADER_M:
            if      (b == 'M') msp_state = MSP_HEADER_DIR;
            else if (b == 'X') msp_state = MSP2_HEADER_DIR;
            else               msp_state = MSP_IDLE;
            break;

        case MSP_HEADER_DIR:
            // We only respond to requests ('<'); ignore responses and errors
            if (b == '<') {
                msp_state = MSP_HEADER_SIZE;
            } else {
                msp_state = MSP_IDLE;
            }
            break;

        case MSP_HEADER_SIZE:
            msp_size           = b;
            msp_checksum_acc   = b;  // checksum starts with size byte
            msp_payload_pos    = 0;
            msp_state          = MSP_HEADER_CMD;
            break;

        case MSP_HEADER_CMD:
            msp_cmd           = b;
            msp_checksum_acc ^= b;
            msp_state          = (msp_size > 0) ? MSP_PAYLOAD : MSP_CHECKSUM;
            break;

        case MSP_PAYLOAD:
            if (msp_payload_pos < MSP_MAX_PAYLOAD) {
                msp_payload[msp_payload_pos] = b;
            }
            msp_checksum_acc ^= b;
            msp_payload_pos++;
            if (msp_payload_pos == msp_size) msp_state = MSP_CHECKSUM;
            break;

        case MSP_CHECKSUM:
            if (b == msp_checksum_acc) {
                msp_handle(msp_cmd);
            }
            msp_state = MSP_IDLE;
            break;

        // ── MSP v2 ───────────────────────────────────────────────────────────

        case MSP2_HEADER_DIR:
            if (b == '<') { msp2_crc = 0; msp_state = MSP2_HEADER_FLAG; }
            else           { msp_state = MSP_IDLE; }
            break;

        case MSP2_HEADER_FLAG:
            msp2_crc = crc8_byte(msp2_crc, b);
            msp_state = MSP2_HEADER_CODE0;
            break;

        case MSP2_HEADER_CODE0:
            msp2_code = b;
            msp2_crc  = crc8_byte(msp2_crc, b);
            msp_state = MSP2_HEADER_CODE1;
            break;

        case MSP2_HEADER_CODE1:
            msp2_code |= (uint16_t)b << 8;
            msp2_crc   = crc8_byte(msp2_crc, b);
            msp_state  = MSP2_HEADER_SIZE0;
            break;

        case MSP2_HEADER_SIZE0:
            msp2_size = b;
            msp2_crc  = crc8_byte(msp2_crc, b);
            msp_state = MSP2_HEADER_SIZE1;
            break;

        case MSP2_HEADER_SIZE1:
            msp2_size    |= (uint16_t)b << 8;
            msp2_crc      = crc8_byte(msp2_crc, b);
            msp2_payload_pos = 0;
            msp_state = (msp2_size > 0) ? MSP2_PAYLOAD : MSP2_CHECKSUM;
            break;

        case MSP2_PAYLOAD:
            if (msp2_payload_pos < MSP_MAX_PAYLOAD)
                msp_payload[msp2_payload_pos] = b;
            msp2_crc = crc8_byte(msp2_crc, b);
            msp2_payload_pos++;
            if (msp2_payload_pos == msp2_size) msp_state = MSP2_CHECKSUM;
            break;

        case MSP2_CHECKSUM:
            if (b == msp2_crc) {
                msp2_send(msp2_code, NULL, 0);
            }
            msp_state = MSP_IDLE;
            break;
    }
}
