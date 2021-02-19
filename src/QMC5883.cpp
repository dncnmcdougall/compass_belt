/*!
 * @file Adafruit_QMC5883_U.cpp
 *
 * @mainpage Adafruit QMC5883 Unified Library
 *
 * @section intro_sec Introduction
 *
 * This is a library for the QMC5883 magnentometer/compass
 *
 * Designed specifically to work with the Adafruit QMC5883 Breakout
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

#include "QMC5883.h"

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
  @brief  Instantiates a new Adafruit_qmc5883 class
  */
/**************************************************************************/
QMC5883::QMC5883(int32_t sensorID) 
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
bool QMC5883::begin(bool setupWire) {
    if( setupWire ) {
        Wire.begin();
    }

    print("Address: ");println(QMC5883_ADDRESS);
    print("setting config: ");
    println(setConfig(QMC5883_MODE_CON, QMC5883_RATE_10,QMC5883_RANGE_2, QMC5883_RATIO_256));

    write8(QMC5883_ADDRESS, QMC5883_REGISTER_MAG_CR2_REG_M, 0x00);
    write8(QMC5883_ADDRESS, QMC5883_REGISTER_MAG_RST_REG_M, 0x01);

    return true;
}

bool QMC5883::setConfig(qmc5883Mode_t mode ,qmc5883Rate_t rate
                   ,qmc5883Range_t range ,qmc5883Ratio_t ratio) {
    _magRange = range;

    switch(range) {
        case QMC5883_RANGE_2:
            _magGain_LSb_Gauss = 12000;
            break;
        case QMC5883_RANGE_8:
            _magGain_LSb_Gauss = 3000;
            break;
    }

    uint8_t value = (mode) | (rate << 2) | (range << 4) | (ratio << 6);

    return write8(QMC5883_ADDRESS, QMC5883_REGISTER_MAG_CR1_REG_M, value);
}

/***************************************************************************
  PRIVATE FUNCTIONS
 ***************************************************************************/
/**************************************************************************/
/*!
  @brief  Reads the raw data from the sensor
  */
/**************************************************************************/
bool QMC5883::read() {

    uint8_t magValues[6];
    if ( !readN(QMC5883_ADDRESS, QMC5883_REGISTER_MAG_OUT_X_L_M, 6, magValues, 1000) ){
        print("Failed: ");
        print(Wire.available());
        println(" available.");
        Wire.endTransmission();
        return false;
    }

    // Shift values to create properly formed integer (low uint8_t first)
    x = (int16_t)(magValues[0] | ((int16_t)magValues[1] << 8));
    y = (int16_t)(magValues[2] | ((int16_t)magValues[3] << 8));
    z = (int16_t)(magValues[4] | ((int16_t)magValues[5] << 8));
    // print("read X: ");print(x);
    // x /= _magGain_LSb_Gauss * SENSORS_GAUSS_TO_MICROTESLA;
    // y /= _magGain_LSb_Gauss * SENSORS_GAUSS_TO_MICROTESLA;
    // z /= _magGain_LSb_Gauss * SENSORS_GAUSS_TO_MICROTESLA;
    // print(": ");println(x);

    uint8_t tempValues[2];
    if ( !readN(QMC5883_ADDRESS, QMC5883_REGISTER_MAG_OUT_TEMP_L_M, 2, tempValues, 1000) ){
        print("Failed: ");
        print(Wire.available());
        println(" available.");
        Wire.endTransmission();
        return false;
    }

    temperature = (int16_t)(tempValues[0] | ((int16_t)tempValues[1] << 8));
    temperature /= QMC5883_TEMP_LSB_C;

    return true;
}

