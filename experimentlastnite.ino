#include <Wire.h>
#include <MAX3010x.h>

#define SDA_I2C 21
#define SCL_I2C 22

MAX30101 myPulseSensor;   // Create a MAX30101 object

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_I2C, SCL_I2C);

  Serial.println("Scanning for I2C devices...");

  if (!myPulseSensor.begin()) {
    Serial.println("MAX30101 not found!");
  } else {
    Serial.println("MAX30101 Initialized!");
 }

  // Configure sensor for SpO2 mode (Red + IR)
    myPulseSensor.setMode(MAX30101::MODE_SPO2);

 Mode mode = myPulseSensor.getMode();         // Use the full class name
    if (Mode mode == MAX30101::MODE_SPO2) {       // Use the full constant name
      Serial.println("Sensor is in SpO2 mode.");
    } else {
      Serial.println("Sensor is NOT in SpO2 mode.");
    }


  

  // Configure sensor for SpO2 mode (Red + IR)
  myPulseSensor.setMode(MAX30101::MODE_SPO2);
  myPulseSensor.setLedCurrent(MAX30101::LED_RED, 255); // Set Red LED brightness
  myPulseSensor.setLedCurrent(MAX30101::LED_IR, 255);  // Set IR LED brightness
  myPulseSensor.setLedCurrent(MAX30101::LED_GREEN_CH1, 100); // Set Green LED brightness
  myPulseSensor.setLedCurrent(MAX30101::LED_GREEN_CH2, 0);   // Turn off Green Channel 2

  Serial.println("Sensor initialized and set to SpO2 mode.");
}

void loop() {
  // Read sample from the sensor
  MAX30101Sample sample;
  sample = myPulseSensor.readSample(1000); // Get sensor sample with a 1000 ms timeout

  if (sample.valid) {
    long irValue = sample.ir;   // Get IR LED value
    long redValue = sample.red; // Get Red LED value

    // Display the IR and Red LED values for debugging
    Serial.print("IR: ");
    Serial.print(irValue);
    Serial.print(" Red: ");
    Serial.print(redValue);

    // Enhanced finger detection (adjusted threshold)
    if (irValue < 1000 && redValue < 5) {
      Serial.println(" No Finger Present!");
    } else {
      // If IR value is within a valid range, consider a finger detected
      Serial.println(" Finger Detected");
      
      // Simple Heart Rate Calculation (BPM) based on IR value peaks
      static unsigned long lastTime = 0;
      static long lastIRValue = 0;

      if (irValue > 10000) {  // Threshold to detect pulse
        unsigned long currentTime = millis();
        if (lastIRValue != 0 && currentTime - lastTime > 500) { // If a pulse interval is detected
          unsigned long pulseInterval = currentTime - lastTime;
          float bpm = 60000.0 / pulseInterval;
          Serial.print(" Heart Rate: ");
          Serial.print(bpm);
          Serial.println(" BPM");
        }
        lastTime = millis();
        lastIRValue = irValue;
      }

      // Simple SpO2 Calculation (ratio of Red to IR light)
      if (irValue > 10000 && redValue > 10) {  // Adjust threshold for SpO2 calculation
        float ratio = (float)redValue / (float)irValue;
        float spo2 = 110 - (25 * ratio); // Simplified SpO2 calculation
        if (spo2 > 100) {
          spo2 = 100; // Ensure SpO2 does not exceed 100%
        } else if (spo2 < 0) {
          spo2 = 0;  // Ensure SpO2 does not go below 0%
        }

        Serial.print(" SpO2: ");
        Serial.print(spo2);
        Serial.println(" %");
      }
    }

  } else {
    Serial.println("Failed to read sample!");
  }

  delay(1000); // Add delay before reading the next sample
}

