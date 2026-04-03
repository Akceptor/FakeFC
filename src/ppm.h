#pragma once
#include <stdint.h>

void ppm_init();
void ppm_set_enabled(bool enable);
bool ppm_is_enabled();

// Returns pointer to 16-element array of channel values in 1000–2000 µs range
const uint16_t* ppm_get_channels();
