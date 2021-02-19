/*!
 *  @file MPU6050.h
 *
 * 	I2C Driver for MPU6050 6-DoF Accelerometer and Gyro
 *
 * 	This is a library for the Adafruit MPU6050 breakout:
 * 	https://www.adafruit.com/products/3886
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */
#define DISABLE_MPU6050

#ifndef DISABLE_MPU6050

#ifndef _ADAFRUIT_MPU6050_H
#define _ADAFRUIT_MPU6050_H

#include "Arduino.h"

#include "Sensor.h"

#include <Wire.h>

#define MPU6050_ADDRESS 0x68 
#define MPU6050_ID 0x68 

typedef enum: uint8_t {

    MPU6050_SELF_TEST_X = 0x0D,
    MPU6050_SELF_TEST_Y = 0x0E,
    MPU6050_SELF_TEST_Z = 0x0F,
    MPU6050_SELF_TEST_A = 0x10,
    MPU6050_SMPLRT_DIV = 0x19,
    MPU6050_CONFIG = 0x1A,
    MPU6050_GYRO_CONFIG = 0x1B,
    MPU6050_ACCEL_CONFIG = 0x1C,
    MPU6050_INT_PIN_CONFIG = 0x37,
    MPU6050_WHO_AM_I = 0x75,
    MPU6050_SIGNAL_PATH_RESET = 0x68,
    MPU6050_USER_CTRL = 0x6,
    MPU6050_PWR_MGMT_1 = 0x6B,
    MPU6050_PWR_MGMT_2 = 0x6,
    MPU6050_TEMP_H = 0x41,
    MPU6050_TEMP_L = 0x4,
    MPU6050_ACCEL_OUT = 0x3B
} mpu6050Registers_t;

/**
 * @brief FSYNC output values
 *
 * Allowed values for `setFsyncSampleOutput`.
 */
typedef enum:uint8_t {
    MPU6050_FSYNC_OUT_DISABLED = 0,
    MPU6050_FSYNC_OUT_TEMP = 1,
    MPU6050_FSYNC_OUT_GYROX = 2,
    MPU6050_FSYNC_OUT_GYROY = 3,
    MPU6050_FSYNC_OUT_GYROZ = 4,
    MPU6050_FSYNC_OUT_ACCELX = 5,
    MPU6050_FSYNC_OUT_ACCELY = 6,
    MPU6050_FSYNC_OUT_ACCEL_Z = 7,
} mpu6050_fsync_out_t;

/**
 * @brief Clock source options
 *
 * Allowed values for `setClock`.
 */
typedef enum:uint8_t {
    MPU6050_INTR_8MHz = 0,
    MPU6050_PLL_GYROX = 1,
    MPU6050_PLL_GYROY = 2,
    MPU6050_PLL_GYROZ = 3,
    MPU6050_PLL_EXT_32K = 4,
    MPU6050_PLL_EXT_19MHz = 5,
    MPU6050_STOP = 7,
} mpu6050_clock_select_t;

/**
 * @brief Accelerometer range options
 *
 * Allowed values for `setAccelerometerRange`.
 */
typedef enum:uint8_t {
    MPU6050_RANGE_2_G = 0,  ///< +/- 2g (default value)
    MPU6050_RANGE_4_G = 1,  ///< +/- 4g
    MPU6050_RANGE_8_G = 2,  ///< +/- 8g
    MPU6050_RANGE_16_G = 3, ///< +/- 16g
} mpu6050_accel_range_t;

/**
 * @brief Gyroscope range options
 *
 * Allowed values for `setGyroRange`.
 */
typedef enum:uint8_t {
    MPU6050_RANGE_250_DEG = 0,  ///< +/- 250 deg/s (default value)
    MPU6050_RANGE_500_DEG = 1,  ///< +/- 500 deg/s
    MPU6050_RANGE_1000_DEG = 2, ///< +/- 1000 deg/s
    MPU6050_RANGE_2000_DEG = 3, ///< +/- 2000 deg/s
} mpu6050_gyro_range_t;

/**
 * @brief Digital low pass filter bandthwidth options
 *
 * Allowed values for `setFilterBandwidth`.
 */
typedef enum:uint8_t {
    MPU6050_BAND_260_HZ = 0, ///< Docs imply this disables the filter
    MPU6050_BAND_184_HZ = 1, ///< 184 Hz
    MPU6050_BAND_94_HZ = 2,  ///< 94 Hz
    MPU6050_BAND_44_HZ = 3,  ///< 44 Hz
    MPU6050_BAND_21_HZ = 4,  ///< 21 Hz
    MPU6050_BAND_10_HZ = 5,  ///< 10 Hz
    MPU6050_BAND_5_HZ = 6,   ///< 5 Hz
} mpu6050_bandwidth_t;

/**
 * @brief Periodic measurement options
 *
 * Allowed values for `setCycleRate`.
 */
typedef enum:uint8_t {
    MPU6050_CYCLE_1_25_HZ = 0, ///< 1.25 Hz
    MPU6050_CYCLE_5_HZ = 1,    ///< 5 Hz
    MPU6050_CYCLE_20_HZ = 2,   ///< 20 Hz
    MPU6050_CYCLE_40_HZ = 3,   ///< 40 Hz
} mpu6050_cycle_rate_t;

class MPU6050;

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the MPU6050 I2C Digital Potentiometer
 */
class MPU6050 {
public:
    MPU6050(int32_t sensorID = -1);
    ~MPU6050();

    bool begin(bool setupWire);
    bool begin(uint8_t i2c_addr = MPU6050_ADDRESS, TwoWire *wire = &Wire,
               int32_t sensorID = 0);

    // Adafruit_Sensor API/Interface
    bool getEvent(sensors_event_t *accel, sensors_event_t *gyro,
                  sensors_event_t *temp);

    mpu6050_accel_range_t getAccelerometerRange(void);
    void setAccelerometerRange(mpu6050_accel_range_t);

    mpu6050_gyro_range_t getGyroRange(void);
    void setGyroRange(mpu6050_gyro_range_t);

    void setInterruptPinPolarity(bool active_low);
    void setFsyncSampleOutput(mpu6050_fsync_out_t fsync_output);

    mpu6050_fsync_out_t getFsyncSampleOutput(void);
    void setI2CBypass(bool bypass);

    void setClock(mpu6050_clock_select_t);
    mpu6050_clock_select_t getClock(void);

    void setFilterBandwidth(mpu6050_bandwidth_t bandwidth);
    mpu6050_bandwidth_t getFilterBandwidth(void);

    void setSampleRateDivisor(uint8_t);
    uint8_t getSampleRateDivisor(void);

    bool enableSleep(bool enable);
    bool enableCycle(bool enable);

    void setCycleRate(mpu6050_cycle_rate_t rate);
    mpu6050_cycle_rate_t getCycleRate(void);
    void reset(void);

private:
    void _getRawSensorData(void);
    void _scaleSensorData(void);

protected:
    float temperature, ///< Last reading's temperature (C)
          accX,          ///< Last reading's accelerometer X axis m/s^2
          accY,          ///< Last reading's accelerometer Y axis m/s^2
          accZ,          ///< Last reading's accelerometer Z axis m/s^2
          gyroX,         ///< Last reading's gyro X axis in rad/s
          gyroY,         ///< Last reading's gyro Y axis in rad/s
          gyroZ;         ///< Last reading's gyro Z axis in rad/s

    void _read(void);
    virtual bool _init();

private:
    int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
    int32_t sensorID;

    void fillTempEvent(sensors_event_t *temp, uint32_t timestamp);
    void fillAccelEvent(sensors_event_t *accel, uint32_t timestamp);
    void fillGyroEvent(sensors_event_t *gyro, uint32_t timestamp);
};

#endif
#endif
