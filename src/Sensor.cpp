#include "Sensor.h"
#include "print.h"

/**************************************************************************/
/*!
    @brief  Prints sensor information to serial console
*/
/**************************************************************************/
void Adafruit_Sensor::printSensorDetails(void) {
  sensor_t sensor;
  getSensor(&sensor);
  println(F("------------------------------------"));
  print(F("Sensor:       "));
  println(sensor.name);
  print(F("Type:         "));
  switch ((sensors_type_t)sensor.type) {
  case SENSOR_TYPE_ACCELEROMETER:
    print(F("Acceleration (m/s2)"));
    break;
  case SENSOR_TYPE_MAGNETIC_FIELD:
    print(F("Magnetic (uT)"));
    break;
  case SENSOR_TYPE_ORIENTATION:
    print(F("Orientation (degrees)"));
    break;
  case SENSOR_TYPE_GYROSCOPE:
    print(F("Gyroscopic (rad/s)"));
    break;
  case SENSOR_TYPE_LIGHT:
    print(F("Light (lux)"));
    break;
  case SENSOR_TYPE_PRESSURE:
    print(F("Pressure (hPa)"));
    break;
  case SENSOR_TYPE_PROXIMITY:
    print(F("Distance (cm)"));
    break;
  case SENSOR_TYPE_GRAVITY:
    print(F("Gravity (m/s2)"));
    break;
  case SENSOR_TYPE_LINEAR_ACCELERATION:
    print(F("Linear Acceleration (m/s2)"));
    break;
  case SENSOR_TYPE_ROTATION_VECTOR:
    print(F("Rotation vector"));
    break;
  case SENSOR_TYPE_RELATIVE_HUMIDITY:
    print(F("Relative Humidity (%)"));
    break;
  case SENSOR_TYPE_AMBIENT_TEMPERATURE:
    print(F("Ambient Temp (C)"));
    break;
  case SENSOR_TYPE_OBJECT_TEMPERATURE:
    print(F("Object Temp (C)"));
    break;
  case SENSOR_TYPE_VOLTAGE:
    print(F("Voltage (V)"));
    break;
  case SENSOR_TYPE_CURRENT:
    print(F("Current (mA)"));
    break;
  case SENSOR_TYPE_COLOR:
    print(F("Color (RGBA)"));
    break;
  }

  println();
  print(F("Driver Ver:   "));
  println(sensor.version);
  print(F("Unique ID:    "));
  println(sensor.sensor_id);
  print(F("Min Value:    "));
  println(sensor.min_value);
  print(F("Max Value:    "));
  println(sensor.max_value);
  print(F("Resolution:   "));
  println(sensor.resolution);
  println(F("------------------------------------\n"));
}
