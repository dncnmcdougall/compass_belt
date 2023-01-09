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

#include <cfloat>

#include <Preferences.h>

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
    , _plane(QMC5883_PLANE_XY_Z_DOWN)
    , _readingCount(-1)
    , _calibrationCount(-1)
    , _minX(INT16_MIN), _maxX(INT16_MAX)
    , _minY(INT16_MIN), _maxY(INT16_MAX)
    , _minZ(INT16_MIN), _maxZ(INT16_MAX)
      , _datumT(0)
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

void QMC5883::startReading() {
    _readingCount = 0;

    x = 0;
    y = 0;
    z = 0;
    temperature = 0;
}

bool QMC5883::readingLoop() {

    uint8_t magValues[6];
    if ( !readN(QMC5883_ADDRESS, QMC5883_REGISTER_MAG_OUT_X_L_M, 6, magValues, 1000) ){
        print("Failed: ");
        print(Wire.available());
        println(" available.");
        Wire.endTransmission();
        return false;
    }

    // Shift values to create properly formed integer (low uint8_t first)
    x += (int16_t)(magValues[0] | ((int16_t)magValues[1] << 8));
    y += (int16_t)(magValues[2] | ((int16_t)magValues[3] << 8));
    z += (int16_t)(magValues[4] | ((int16_t)magValues[5] << 8));
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

    temperature += (int16_t)(tempValues[0] | ((int16_t)tempValues[1] << 8));

    ++_readingCount;

    return true;
}

void QMC5883::endReading() {

    println(" === === ");
    x /= _readingCount;
    y /= _readingCount;
    z /= _readingCount;
    temperature /= _readingCount;
    temperature /= QMC5883_TEMP_LSB_C;

    if ( _calibrationCount == -1 ) {
        print("X: "); print(x); print("  (");
        print(_minX);print(", ");print(_maxX);println(") ");
        print("Y: "); print(y); print("  (");
        print(_minY);print(", ");print(_maxY);println(") ");
        print("Z: "); print(z); print(" (");
        print(_minZ);print(", ");print(_maxZ);println(") ");

        x = ( x - _minX )/(_maxX - _minX);
        y = ( y - _minY )/(_maxY - _minY);
        z = ( z - _minZ )/(_maxZ - _minZ);
        x = ( x * 2 ) - 1 ;
        y = ( y * 2 ) - 1 ;
        z = ( z * 2 ) - 1 ;

        temperature -= _datumT;

        print("X: "); print(x); print("  ");
        print("Y: "); print(y); print("  ");
        print("Z: "); print(z); println(" uT");
        switch( _plane ) {
            case QMC5883_PLANE_XY_Z_UP:
                print("XY Z up ");
                break;
            case QMC5883_PLANE_YZ_X_UP:
                print("YZ X up ");
                break;
            case QMC5883_PLANE_ZX_Y_UP:
                print("ZX Y up ");
                break;
            case QMC5883_PLANE_XY_Z_DOWN:
                print("XY Z down ");
                break;
            case QMC5883_PLANE_YZ_X_DOWN:
                print("YZ X down ");
                break;
            case QMC5883_PLANE_ZX_Y_DOWN:
                print("ZX Y down ");
                break;
        }
        float h = heading();
        print("heading: "); print(h); print(": "); println(h*360); 
    }

    _readingCount = -1;
}


void QMC5883::startCalibrating() {
    _minX = FLT_MAX;
    _minY = FLT_MAX;
    _minZ = FLT_MAX;

    _maxX = -FLT_MAX;
    _maxY = -FLT_MAX;
    _maxZ = -FLT_MAX;

    _datumT = 0;

    _calibrationCount = 0;

    println("Please rotate in plane");
}

void QMC5883::calibrateLoop(){ 
    startReading(); 
    readingLoop(); 

    _minX = min(_minX, x);
    _maxX = max(_maxX, x);

    _minY = min(_minY, y);
    _maxY = max(_maxY, y);

    _minZ = min(_minZ, z);
    _maxZ = max(_maxZ, z);

    // Turn into an average.
    _datumT += temperature;

    ++_calibrationCount;
}

void QMC5883::endCalibrating() {

    print("Count: "); println(_calibrationCount);
    float rX = _maxX - _minX;
    float rY = _maxY - _minY;
    float rZ = _maxZ - _minZ;
    print("X: "); print(_minX);print(", ");print(_maxX);print(": ");println(rX);
    print("Y: "); print(_minY);print(", ");print(_maxY);print(": ");println(rY);
    print("Z: "); print(_minZ);print(", ");print(_maxZ);print(": ");println(rZ);


    float minR = min(rX, min(rY, rZ));
    if ( minR == rX ) {
        print("YZ plane."); 
        if ( _maxX < 0 ) {
            _plane = QMC5883_PLANE_YZ_X_DOWN;
            println("X down."); 
        } else  {
            _plane = QMC5883_PLANE_YZ_X_UP;
            println("X up.");
        }
    } else if ( minR == rY ) {
        print("ZX plane."); 
        if ( _maxY < 0 ) { 
            _plane = QMC5883_PLANE_ZX_Y_DOWN;
            println("Y down."); 
        } else {
            _plane = QMC5883_PLANE_ZX_Y_UP;
            println("Y up.");
        }
    } else if (minR == rZ ) {
        print("XY plane."); 
        if ( _maxZ < 0 ) {
            _plane = QMC5883_PLANE_XY_Z_DOWN;
            println("Z down."); 
        } else {
            _plane = QMC5883_PLANE_XY_Z_UP;
            println("Z up.");
        }
    } else {
        println("WE SHOULD NOT GET HERE.");
    }

    _datumT /= _calibrationCount;

    _calibrationCount = -1;
    startReading();
    readingLoop();
    endReading(); 
}

void QMC5883::setAngleToHeading() {
    _angle = 0;
    _angle = heading();
    print("Setting angle to: ");println(_angle*360);
}

// bool QMC5883::loadCalibration() {
//   preferences.begin("QMC5883", false);

//   preferences.getFloat("maxX", INT16_MAX);
//   preferences.getFloat("maxY", INT16_MAX);
//   preferences.getFloat("maxZ", INT16_MAX);
//   preferences.getFloat("minX", INT16_MIN);
//   preferences.getFloat("minY", INT16_MIN);
//   preferences.getFloat("minZ", INT16_MIN);
//   preferences.getFloat("datumT", 0.0f);
//   preferences.getFloat("angle", 0.0f);

//   preferences.end();
//   return true;
// }

// bool QMC5883::saveCalibration(int address) const {
//   preferences.begin("QMC5883", false);

//   preferences.putFloat("maxX", _maxX);
//   preferences.putFloat("maxY", _maxY);
//   preferences.putFloat("maxZ", _maxZ);
//   preferences.putFloat("minX", _minX);
//   preferences.putFloat("minY", _minY);
//   preferences.putFloat("minZ", _minZ);
//   preferences.putFloat("datumT", _datumT);
//   preferences.putFloat("angle", _angle);

//   preferences.end();
//   return true;
// }

float QMC5883::heading() const {
    float fwd = 0;
    float left = 0;
    switch( _plane ) {
        case QMC5883_PLANE_XY_Z_UP:
            fwd = x;
            left = -y;
            break;
        case QMC5883_PLANE_YZ_X_UP:
            fwd = y;
            left = -z;
            break;
        case QMC5883_PLANE_ZX_Y_UP:
            fwd = z;
            left = -x;
            break;
        case QMC5883_PLANE_XY_Z_DOWN:
            fwd = x;
            left = y;
            break;
        case QMC5883_PLANE_YZ_X_DOWN:
            fwd = y;
            left = z;
            break;
        case QMC5883_PLANE_ZX_Y_DOWN:
            fwd = z;
            left = x;
            break;
        default:
            return -1;
    }

    float heading = atan2f(left, fwd);
    heading += _angle*2*PI;
    if(heading < 0) heading += 2*PI;
    if(heading > 2*PI) heading -= 2*PI;
    heading /= (2*PI);

    return heading;
}
/***************************************************************************
  PRIVATE FUNCTIONS
 ***************************************************************************/
