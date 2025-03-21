#include <Wire.h>
#include "MAX3010x.h"
#include "MAX3010x_core.h"
#include "MAX3010x_multiLed_core.h"

#define SDA_I2C 21
#define SCL_I2C 22
#define I2C_ADDRESS 0x57  // Default I2C address for MAX3010x

TwoWire myWire = Wire;  // Using the default I2C instance

// Corrected: Use both template parameters required
MAX3010xMultiLed<MAX3010x, MAX3010xSample> myPulseSensor(I2C_ADDRESS, myWire);

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_I2C, SCL_I2C);

    Serial.println("Scanning for I2C devices...");

    byte count = 0;
    for (byte i = 8; i < 120; i++) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
            Serial.print("Found I2C device at address 0x");
            Serial.println(i, HEX);
            count++;
        }
    }
    if (count == 0) Serial.println("No I2C devices found.");
    delay(2000);

    if (!myPulseSensor.begin()) { 
        Serial.println("MAX3010x not found! Check wiring.");
        while (1);
    }

    Serial.println("MAX3010x Initialized!");

    // Configure sensor settings using MAX3010x constants
    myPulseSensor.setMode(MAX3010xMultiLed<MAX3010x, MAX3010xSample>::MODE_SPO2);  // Set SpO2 Mode
    myPulseSensor.setLedCurrent(MAX3010xMultiLed<MAX3010x, MAX3010xSample>::LED_RED, 0xFF);  // Red LED
    myPulseSensor.setLedCurrent(MAX3010xMultiLed<MAX3010x, MAX3010xSample>::LED_IR, 0xFF);   // IR LED
    myPulseSensor.setLedCurrent(MAX3010xMultiLed<MAX3010x, MAX3010xSample>::LED_GREEN_CH1, 100);
    myPulseSensor.setLedCurrent(MAX3010xMultiLed<MAX3010x, MAX3010xSample>::LED_GREEN_CH2, 0);

    Serial.println("Sensor configured and set to SpO2 mode.");
}

void loop() {
    MAX3010xSample sample = myPulseSensor.readSample(1000);  // Get sensor sample

    if (sample.valid) {     
        long irValue = sample.ir;
        long redValue = sample.red;

        Serial.print("IR: ");
        Serial.print(irValue);
        Serial.print(" Red: ");
        Serial.println(redValue);

        if (irValue < 50000) {
            Serial.println(" No Finger Present!");
        } else {
            Serial.println(" Calculating...");
        }
    } else {
        Serial.println("Failed to read sample!");
    }

    delay(1000);
}
