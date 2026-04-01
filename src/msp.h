#pragma once
#include <stdint.h>

void msp_init();

// Feed one byte from USB serial into the MSP state machine.
// Call for every byte read from Serial in the main loop.
void msp_process_byte(uint8_t b);
