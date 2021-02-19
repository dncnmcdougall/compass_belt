
#ifndef WIRE_COMMON
#define WIRE_COMMON

#include <stdint.h>

bool write8(const uint8_t& address, uint8_t reg, uint8_t value);
uint8_t read8(const uint8_t& address, uint8_t reg);
bool readN(const uint8_t& address, const uint8_t& reg, const uint8_t& n, uint8_t* values, const long& timeout=-1);

#endif
