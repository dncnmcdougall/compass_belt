
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


WiFiClient net;
MQTTClient client;
QMC5883 mag;
HC595 shift(12, 14, 13);

#define BUTTON 18
#define STATUS_PIN 19

unsigned long connectedMillis = 0;
unsigned long lastMillis = 0;
unsigned long buttonMillis = 0;
bool magError = false;

uint8_t shiftData = 0;

void messageReceived(String& topic, String& payload) {
    println("incoming: "+topic+" - "+payload);
}

void connect() {
    digitalWrite(STATUS_PIN, HIGH);

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

    digitalWrite(STATUS_PIN, LOW);
}

void setup() {
    pinMode(BUTTON, INPUT);
    pinMode(STATUS_PIN, OUTPUT);

    digitalWrite(STATUS_PIN, HIGH);

    Serial.begin(9600, SERIAL_8N1, -1, -1, false, 5000UL);
    delay(10);
    println("Starting...");

    shift.begin();
    for( uint8_t i = 1; i > 0; i <<= 1 )
    {
        shift.setData(i);
        shift.shift();
        delay(100);
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

    connect();
    client.subscribe("/test");
    client.publish("status/","starting");
}

void loop() {
    delay(20);

    if ( client.connected() && !client.loop() ) {
        println("loop failed");
    } else if ( millis() - connectedMillis > 30000 ) {
        shift.setData(0);
        shift.shift();
        connect();
        connectedMillis = millis();
    }

    magError = !mag.read();

    if ( digitalRead(BUTTON) == LOW ) {
        buttonMillis = millis();
        shift.setData(0);
        shift.shift();
    }

    if ( millis() - buttonMillis < 5000 ) {
        unsigned long diff = millis() - buttonMillis;

        if ( (diff/100) % 5 == 0 ) {
            digitalWrite(STATUS_PIN, HIGH);
        } else {
            digitalWrite(STATUS_PIN, LOW);
        }
    }  else {
        digitalWrite(STATUS_PIN, LOW);
    }

    if ( millis() - lastMillis > 2000) {
        if ( millis() - buttonMillis < 5000 ) {
            client.publish("status/","button");
            println("status: button");
        } else {
            client.publish("status/","running");
            println("status: tick");

            shiftData = shiftData >= 128 || shiftData == 0? 1: shiftData << 1;
            shift.setData(shiftData);
            shift.shift();
            print("set data: "); println(shiftData);
            client.publish("out/data",String(shiftData));

            if ( magError ) {
                println("Failed to read mag sensor");
                client.publish("mag/error","1");
            } else {
                client.publish("mag/error","0");

                /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
                print("X: "); print(mag.x); print("  ");
                client.publish("mag/x",String(mag.x));

                print("Y: "); print(mag.y); print("  ");
                client.publish("mag/y",String(mag.y));

                print("Z: "); print(mag.z); println(" uT");
                client.publish("mag/z",String(mag.z));

                print("temp: "); print(mag.temperature); println(" C");
                client.publish("mag/temp",String(mag.temperature));

                float heading = atan2(mag.y, mag.x);
                if(heading < 0) heading += 2*PI;
                if(heading > 2*PI) heading -= 2*PI;
                float headingDegrees = heading * 180/M_PI; 
                print("Heading (degrees): "); println(headingDegrees);
                client.publish("mag/dir",String(headingDegrees));
            }
        }

        lastMillis = millis();
    } 

}
