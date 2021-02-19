#define ENABLE_I2C_DEBUG_BUFFER
#include "WireCommon.h"

#include <stdlib.h>
#include "Arduino.h"
#include <Wire.h>

bool write8(const uint8_t& address, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    auto val = Wire.endTransmission();
    return val == I2C_ERROR_OK;
}

/**************************************************************************/
/*!
  @brief  Abstract away platform differences in Arduino wire library
  */
/**************************************************************************/
uint8_t read8(const uint8_t& address, uint8_t reg) {
    uint8_t value;

    Wire.beginTransmission(address);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();

    Wire.requestFrom(address, (uint8_t)1);
    value = Wire.read();
    Wire.endTransmission();

    return value;
}


/**************************************************************************/
/*!
  @brief  Abstract away platform differences in Arduino wire library
  */
/**************************************************************************/
bool readN(const uint8_t& address, const uint8_t& reg, const uint8_t& n, uint8_t* values, const long& timeout) {

    memset(values, 0, sizeof(uint8_t)*n);

    Wire.beginTransmission(address);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();

    Wire.requestFrom(address, n);
    if ( timeout >= 0 ) {
        long startMillis = millis();
        while ( Wire.available() < n && millis() - startMillis < timeout ) {
            sleep(10);
        }
        if ( Wire.available() < n ) {
            return false;
        }
    } else {
        while ( Wire.available() < n );
    }
    for( uint8_t i=0;i<n;++i) {
        values[i] = Wire.read();
    }
    return true;
}

