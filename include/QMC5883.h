#ifndef _qmc5883_H_
#define _qmc5883_H_

#include "Arduino.h"

#include "Sensor.h"

#include <Wire.h>

#define QMC5883_ADDRESS (0x0D)

#define QMC5883_ID (0b11010100)

#define QMC5883_TEMP_LSB_C (100)

/*!
 * @brief Registers
 */
#define QMC5883_REGISTER_MAG_OUT_X_L_M (0x00)
#define QMC5883_REGISTER_MAG_OUT_X_H_M (0x01)
#define QMC5883_REGISTER_MAG_OUT_Z_L_M (0x02)
#define QMC5883_REGISTER_MAG_OUT_Z_H_M (0x03)
#define QMC5883_REGISTER_MAG_OUT_Y_L_M (0x04)
#define QMC5883_REGISTER_MAG_OUT_Y_H_M (0x05)
#define QMC5883_REGISTER_MAG_STA_REG_M (0x06)
#define QMC5883_REGISTER_MAG_OUT_TEMP_L_M (0x07)
#define QMC5883_REGISTER_MAG_OUT_TEMP_H_M (0x08)
#define QMC5883_REGISTER_MAG_CR1_REG_M (0x09)
#define QMC5883_REGISTER_MAG_CR2_REG_M (0x0A)
#define QMC5883_REGISTER_MAG_RST_REG_M (0x0B)

//! Unified sensor driver for the magnetometer ///
class QMC5883 {
public:

    typedef enum:uint8_t {
        QMC5883_MODE_SBY = 0b00,
        QMC5883_MODE_CON = 0b01
    } qmc5883Mode_t;

    typedef enum:uint8_t {
        QMC5883_RATE_10 = 0b00,
        QMC5883_RATE_50 = 0b01,
        QMC5883_RATE_100 = 0b10,
        QMC5883_RATE_200 = 0b11
    } qmc5883Rate_t;

    typedef enum:uint8_t {
        QMC5883_RANGE_2 = 0x00,
        QMC5883_RANGE_8 = 0x01
    } qmc5883Range_t;

    typedef enum:uint8_t {
        QMC5883_RATIO_512 = 0b00,
        QMC5883_RATIO_256 = 0b01, 
        QMC5883_RATIO_128 = 0b10, 
        QMC5883_RATIO_64 = 0b11 
    } qmc5883Ratio_t;

public:
    /*!
     * @param sensorID sensor ID, -1 by default
     */
    QMC5883(int32_t sensorID = -1);

    bool begin(bool setupWire=true); //!< @return Returns whether connection was successful
    bool setConfig(qmc5883Mode_t mode ,qmc5883Rate_t rate
                   ,qmc5883Range_t range ,qmc5883Ratio_t ratio);
    bool read();

    float x, y, z, temperature;
private:
    qmc5883Range_t _magRange;
    float _magGain_LSb_Gauss;
    int32_t _sensorID;

};

#endif
