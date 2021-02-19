/*!
 * @file Adafruit_HMC5883_U.cpp
 *
 * @mainpage Adafruit HMC5883 Unified Library
 *
 * @section intro_sec Introduction
 *
 * This is a library for the HMC5883 magnentometer/compass
 *
 * Designed specifically to work with the Adafruit HMC5883 Breakout
 * http://www.adafruit.com/products/1746
 *
 * These displays use I2C to communicate, 2 pins are required to interface.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit andopen-source hardware by purchasing products
 * from Adafruit!
 *
 * @section author Author
 *
 * Written by Kevin Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 */

#include "HMC5883.h"

#include "WireCommon.h"

#include "print.h"

/***************************************************************************
  MAGNETOMETER
 ***************************************************************************/

/***************************************************************************
  CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
  @brief  Instantiates a new Adafruit_HMC5883 class
  */
/**************************************************************************/
HMC5883::HMC5883(int32_t sensorID) 
    : _sensorID(sensorID)
{
}

/***************************************************************************
  PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
  @brief  Setups the HW
  */
/**************************************************************************/
bool HMC5883::begin(bool setupWire) {
    if( setupWire ) {
        Wire.begin();
    }

    print("Address: ");println(HMC5883_ADDRESS);
    print("setting config: ");
    println(write8(HMC5883_ADDRESS, HMC5883_REGISTER_MAG_CRA_REG_M, 0x40));
    setMagGain(HMC5883_MAGGAIN_1_3);
    print("setting mode: ");println(write8(HMC5883_ADDRESS, HMC5883_REGISTER_MAG_MR_REG_M, 0x00));

    return true;
}

/**************************************************************************/
/*!
  @brief  Sets the magnetometer's gain
  */
/**************************************************************************/
void HMC5883::setMagGain(hmc5883MagGain gain) {
    write8(HMC5883_ADDRESS, HMC5883_REGISTER_MAG_CRB_REG_M, (uint8_t)gain);

    _magGain = gain;

    switch (gain) {
        case HMC5883_MAGGAIN_0_8:
            _magGain_LSb_Gauss = 1370;
            break;
        case HMC5883_MAGGAIN_1_3:
            _magGain_LSb_Gauss = 1090;
            break;
        case HMC5883_MAGGAIN_1_9:
            _magGain_LSb_Gauss = 820;
            break;
        case HMC5883_MAGGAIN_2_5:
            _magGain_LSb_Gauss = 660;
            break;
        case HMC5883_MAGGAIN_4_0:
            _magGain_LSb_Gauss = 440;
            break;
        case HMC5883_MAGGAIN_4_7:
            _magGain_LSb_Gauss = 390;
            break;
        case HMC5883_MAGGAIN_5_6:
            _magGain_LSb_Gauss = 330;
            break;
        case HMC5883_MAGGAIN_8_1:
            _magGain_LSb_Gauss = 230;
            break;
    }
}

/***************************************************************************
  PRIVATE FUNCTIONS
 ***************************************************************************/
/**************************************************************************/
/*!
  @brief  Reads the raw data from the sensor
  */
/**************************************************************************/
bool HMC5883::read() {

    uint8_t values[6];
    if ( !readN(HMC5883_ADDRESS, HMC5883_REGISTER_MAG_OUT_X_H_M, 6, values, 1000) ){
        print("Failed: ");
        print(Wire.available());
        println(" available.");
        Wire.endTransmission();
        return false;
    } else {
        print("success");
    }

    // Shift values to create properly formed integer (low uint8_t first)
    x = (int16_t)(values[1] | ((int16_t)values[0] << 8));
    y = (int16_t)(values[3] | ((int16_t)values[2] << 8));
    z = (int16_t)(values[5] | ((int16_t)values[4] << 8));
    x /= _magGain_LSb_Gauss * SENSORS_GAUSS_TO_MICROTESLA;
    y /= _magGain_LSb_Gauss * SENSORS_GAUSS_TO_MICROTESLA;
    z /= _magGain_LSb_Gauss * SENSORS_GAUSS_TO_MICROTESLA;
    return true;
}

