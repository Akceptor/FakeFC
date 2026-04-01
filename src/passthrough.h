#pragma once
#include <stdint.h>

void passthrough_init();

// Feed one byte from USB serial into the CLI command parser.
// Call for every byte read from Serial in the main loop (same bytes as MSP).
// If a "serialpassthrough" command is fully received, this function enters a
// permanent blocking bridge loop — it never returns (reboot to exit).
void passthrough_process_byte(uint8_t b);
