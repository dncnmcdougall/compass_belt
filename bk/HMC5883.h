/*!
 * @file Adafruit_HMC5883_U.h
 */
#ifndef _HMC5883_H_
#define _HMC5883_H_

#include "Arduino.h"

#include "Sensor.h"

#include <Wire.h>

/*!
 * @brief I2C address/bits
 */

#define HMC5883_ADDRESS (0x1E)
// #define HMC5883_ADDRESS (0x3C)

/*!
 * @brief Chip ID
 */
#define HMC5883_ID (0b11010100)

/*!
  @brief Registers
  */
#define HMC5883_REGISTER_MAG_CRA_REG_M (0x00)
#define HMC5883_REGISTER_MAG_CRB_REG_M (0x01)
#define HMC5883_REGISTER_MAG_MR_REG_M (0x02)
#define HMC5883_REGISTER_MAG_OUT_X_H_M (0x03)
#define HMC5883_REGISTER_MAG_OUT_X_L_M (0x04)
#define HMC5883_REGISTER_MAG_OUT_Z_H_M (0x05)
#define HMC5883_REGISTER_MAG_OUT_Z_L_M (0x06)
#define HMC5883_REGISTER_MAG_OUT_Y_H_M (0x07)
#define HMC5883_REGISTER_MAG_OUT_Y_L_M (0x08)
#define HMC5883_REGISTER_MAG_SR_REG_M (0x09)
#define HMC5883_REGISTER_MAG_IRA_REG_M (0x0A)
#define HMC5883_REGISTER_MAG_IRB_REG_M (0x0B)
#define HMC5883_REGISTER_MAG_IRC_REG_M (0x0C)
#define HMC5883_REGISTER_MAG_TEMP_OUT_H_M (0x31)
#define HMC5883_REGISTER_MAG_TEMP_OUT_L_M (0x32)

//! Unified sensor driver for the magnetometer ///
class HMC5883 : public Adafruit_Sensor {
public:
    /*!
     * @brief Magnetometer gain settings
     */
    typedef enum: uint8_t {
        HMC5883_MAGGAIN_0_8 = 0x00, // +/- 0.88
        HMC5883_MAGGAIN_1_3 = 0x20, // +/- 1.3
        HMC5883_MAGGAIN_1_9 = 0x40, // +/- 1.9
        HMC5883_MAGGAIN_2_5 = 0x60, // +/- 2.5
        HMC5883_MAGGAIN_4_0 = 0x80, // +/- 4.0
        HMC5883_MAGGAIN_4_7 = 0xA0, // +/- 4.7
        HMC5883_MAGGAIN_5_6 = 0xC0, // +/- 5.6
        HMC5883_MAGGAIN_8_1 = 0xE0  // +/- 8.1
    } hmc5883MagGain;

public:
    /*!
     * @param sensorID sensor ID, -1 by default
     */
    HMC5883(int32_t sensorID = -1);

    bool begin(bool setupWire=true); //!< @return Returns whether connection was successful
    void setMagGain(hmc5883MagGain gain); //!< @param gain Desired magnetic gain
    bool read();

    float x, y, z;
private:
    hmc5883MagGain _magGain;
    float _magGain_LSb_Gauss;
    int32_t _sensorID;

};

#endif
