
#include <stdlib.h>
#include <iostream>
#include <string>

#include "Arduino.h"
#include <WiFi.h>
#include <MQTT.h>
#include <Wire.h>

#include "config.h"
#include "print.h"
#include "QMC5883.h"
#include "HC595.h"

#include "AsyncTask.h"

WiFiClient net;
MQTTClient client;
QMC5883 mag;
HC595 shift(12, 14, 13);

#define BUTTON 18
#define STATUS_PIN 19

#define LOOP_DELAY 10
#define WAIT_TIME 5000
#define CALIBRATION_TIME 15000
#define READ_TIME 100.0f
#define POINT_TIME 2000.0f

#define SHORT_PRESS_TIME 400
#define LONG_PRESS_TIME 1500
#define DOUBLE_PRESS_SPACE 400

#define CHECK_BUTTON false
#define CONTINUE true

unsigned long buttonStart = 0;
unsigned long buttonEnd = 0;
unsigned long lastButtonEnd = 0;
unsigned int shortPresses = 0;
unsigned int longPresses = 0;

status_t status = WAIT;
unsigned long lastPoint = 0;
unsigned long currentTime;

uint8_t shiftData = 0;
bool shiftForward = true;

void messageReceived(String& topic, String& payload) {
    println("incoming: "+topic+" - "+payload);
}

bool linToCurve(float value, float time, float mid, float width) {
    bool val =  time*expf(-0.5f*powf((value-mid)/width,2)) > 0.45f;
    // print("linToCurve( ");print(value);print(", ");print(time);print(") -> ");println(val);
    return val;
}

uint8_t interpolate8(float value, float timeValue) {

    uint8_t out = 0;
    out |= (linToCurve(value, timeValue, 0.000f, 0.15f) );
    out |= (linToCurve(value, timeValue, 0.125f, 0.15f) << 1);
    out |= (linToCurve(value, timeValue, 0.250f, 0.15f) << 2);
    out |= (linToCurve(value, timeValue, 0.375f, 0.15f) << 3);
    out |= (linToCurve(value, timeValue, 0.500f, 0.15f) << 4);
    out |= (linToCurve(value, timeValue, 0.625f, 0.15f) << 5);
    out |= (linToCurve(value, timeValue, 0.750f, 0.15f) << 6);
    out |= (linToCurve(value, timeValue, 0.875f, 0.15f) << 7);

    return out;
}

void connect() {
    digitalWrite(STATUS_PIN, HIGH);

    unsigned long lastMillis = 0;

    if (WiFi.status() != WL_CONNECTED) {
        print("Connecting to WiFi: ");
        println(ssid);
        WiFi.begin(ssid, pswd);
        delay(10);
        lastMillis = millis();
        while(WiFi.status() != WL_CONNECTED) {
            delay(500);
            print(".");
            if (millis() - lastMillis > 10000) {
                println("WiFi connection Failed");
                return;
            }
        }
        println("");
        print("WiFi connected: ");println(WiFi.localIP());
    }

    print("Beginning connection to ");println(MQTT_addr.toString());
    client.begin(MQTT_addr, 1883, net);
    client.onMessage(messageReceived);

    lastMillis = millis();
    while( !client.connect("ESP32", MQTT_user, MQTT_pswd) ) {
        delay(500);
        print(".");
        if (millis() - lastMillis > 10000) {
            println("Failed to connect to MQTT.");
            return;
        }
    }
    println("");
    println("MQTT connected ");

    mag.startReading();
    lastMillis = millis();
    while( millis() - lastMillis < READ_TIME) {
        delay(LOOP_DELAY);
    }
    mag.endReading();
    client.publish("mag/x",String(mag.x));
    client.publish("mag/y",String(mag.y));
    client.publish("mag/z",String(mag.z));
    client.publish("mag/temp",String(mag.temperature));

    client.loop();
    client.disconnect();
    WiFi.disconnect();

    digitalWrite(STATUS_PIN, LOW);
}

bool checkButton() {
    bool buttonDown = digitalRead(BUTTON) == LOW;

    if ( buttonDown ) {
        if ( buttonStart == 0 ) { // just pressed
            buttonStart = millis();
        }
        return CHECK_BUTTON;
    } else { // button up
        if ( buttonStart == 0 ) { // Not pressed and not in a press
            if ( millis() - buttonEnd < DOUBLE_PRESS_SPACE ) {
                return CHECK_BUTTON;
            } else {
                return CONTINUE;
            }
        } else { // was pressed, but released
            lastButtonEnd = buttonEnd;
            buttonEnd = millis();
            unsigned long pressDuration = buttonEnd - buttonStart;
            buttonStart = 0;
            if ( pressDuration < SHORT_PRESS_TIME) {
                shortPresses++;
                return CHECK_BUTTON;
            } else if (  pressDuration > LONG_PRESS_TIME ) {
                longPresses++;
                return CONTINUE;
            } else {
                //Um... Medum press?
                return CONTINUE;
            }
        }
    }
}

void setup() {
    pinMode(BUTTON, INPUT);
    pinMode(STATUS_PIN, OUTPUT);

    digitalWrite(STATUS_PIN, HIGH);

    Serial.begin(9600, SERIAL_8N1, -1, -1, false, 5000UL);
    delay(10);
    println("Hellow world");
    println("Starting...");

    shift.begin();
    for( uint8_t i = 1; i > 0; i <<= 1 )
    {
        println("" + i);
        shift.setData(i);
        shift.shift();
        delay(200);
    }
    println("shifted");
    shift.setData(0);
    shift.shift();
    println("Connecting");

    println("Connecting to Wire...");
    if (!Wire.begin(21,22) ) {
        println("Failed to begin wire");
    } else {
        print("Wire at ");println(Wire.getClock());
        println("Connecting to Mag Sensor...");

        if(!mag.begin(false))
        {
            println("Ooops, no QMC5883 detected ... Check your wiring!");
        }
        println("Done");
    }
}

AsyncTask readNorth(READ_NORTH, POINT_NORTH, READ_TIME,
                    [&](const unsigned long& currentTime) { 
                        digitalWrite(STATUS_PIN, HIGH); 
                        mag.startReading(); 
                    },
                    [&](const unsigned long& currentTime) { 
                        mag.readingLoop();
                    },
                    [&](const unsigned long& currentTime) { 
                        digitalWrite(STATUS_PIN, LOW); 
                        mag.endReading();
                    });

AsyncTask pointNorth(POINT_NORTH, WAIT, POINT_TIME,
                     [&](const unsigned long& currentTime) { 
                         digitalWrite(STATUS_PIN, HIGH); 
                     },
                     [&](const unsigned long& currentTime) { 
                        float timeValue = ((float)(currentTime - pointNorth.taskStart))/pointNorth.taskDuration;
                        shiftData = interpolate8(mag.heading(), timeValue);
                        shift.setData(shiftData); shift.shift();
                     },
                     [&](const unsigned long& currentTime) { 
                        digitalWrite(STATUS_PIN, LOW); 
                        shiftData=0; shift.setData(shiftData); shift.shift();
                        lastPoint = currentTime;
                     });

AsyncTask calibrate(CALIBRATE_RANGE, CALIBRATE_ANGLE, CALIBRATION_TIME,
                    [&] (const unsigned long& currentTime) {
                        println("start: CALIBRATION_RANGE");
                        mag.startCalibrating();
                        shiftData=1; shift.setData(shiftData); shift.shift();
                    }, 
                    [&] (const unsigned long& currentTime) {
                        mag.calibrateLoop();
                        if ( currentTime%1000 < 200 ) {
                            digitalWrite(STATUS_PIN, HIGH);
                        } else {
                            digitalWrite(STATUS_PIN, LOW);
                        }
                        if ( currentTime%500 < LOOP_DELAY ) {
                            if ( shiftForward ) {
                                shiftData<<=1; 
                            } else {
                                shiftData>>=1; 
                            } 
                            if ( shiftData == 0 ) {
                                if ( shiftForward) {
                                    shiftData = 1 << 6;
                                } else {
                                    shiftData = 1 << 1;
                                }
                                shiftForward = !shiftForward;
                            }
                            shift.setData(shiftData); shift.shift();
                        }
                    },
                    [&] (const unsigned long& currentTime) {
                        mag.endCalibrating();
                        println("end: CALIBRATION_RANGE");
                    });

void loop() {
    delay(LOOP_DELAY);
    currentTime = millis();

    if ( checkButton() == CONTINUE ) {
        if ( shortPresses == 1 ) {
            println(" short press");
            if ( status == CALIBRATE_ANGLE ) {
                shiftData <<= 1;
                shiftData = shiftData == 0 ? 1 : shiftData;
                shift.setData(shiftData); shift.shift();
            } else {
                status = READ_NORTH;
            }
            shortPresses = 0;
        } else if ( shortPresses == 2 ) {
            println(" double short press");
            if ( status == CALIBRATE_ANGLE ) {
                // Do nothing
            } else {
                status = CONNECT;
            }
            shortPresses = 0;
        } else if ( longPresses == 1) {
            println(" long press");
            if ( status == CALIBRATE_ANGLE ) {
                mag.setAngleToHeading();
                status = WAIT;
                println("end: CALIBRATION_ANGLE");
            } else {
                status = CALIBRATE_RANGE;
            }
            longPresses = 0;
        }

        switch( status ) {
            case WAIT:
                digitalWrite(STATUS_PIN, LOW);
                if ( currentTime - lastPoint > WAIT_TIME) {
                    status = READ_NORTH;
                }
                break;
            case CONNECT:
                println("status: CONNECT");
                shift.setData(0);
                shift.shift();
                connect();
                status = WAIT;
                break;
            default:
                status = readNorth.taskLoop(status, currentTime);
                status = pointNorth.taskLoop(status, currentTime);
                status = calibrate.taskLoop(status, currentTime);
                break;
        }
    }
}
