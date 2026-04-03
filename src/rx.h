#pragma once
#include <stdint.h>

#define RX_CHANNEL_COUNT 16

// Betaflight feature bits for receiver mode (Cleanflight/BF stable mapping)
#define FEATURE_RX_PPM          (1u << 0)
#define FEATURE_RX_SERIAL       (1u << 3)
#define FEATURE_RX_PARALLEL_PWM (1u << 11)
#define FEATURE_RX_MSP          (1u << 12)

void rx_init();
void rx_update();

const uint16_t* rx_get_channels();

uint32_t rx_get_feature_mask();
void rx_set_feature_mask(uint32_t mask);

uint8_t rx_get_serial_provider();
void rx_set_serial_provider(uint8_t provider);

bool rx_serial_enabled();
bool rx_ppm_enabled();

uint32_t rx_get_bytes_received();
uint32_t rx_get_frame_count();
