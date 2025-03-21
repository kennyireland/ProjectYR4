#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#define SDA_I2C 21                  
#define SCL_I2C 22                  
#define PANIC_BUTTON_PIN 27   // Define the panic button pin
#define PANIC_HOLD_TIME 3000  // Time in milliseconds (3 sec)

Adafruit_MMA8451 mma = Adafruit_MMA8451();

float GtoMeterSQConversion(int rawValue); // Function prototype
long measureSampleTime();

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_I2C, SCL_I2C);
    pinMode(PANIC_BUTTON_PIN, INPUT_PULLUP);  // Set panic button as input with pull-up

    Serial.println("Scanning for I2C devices..");

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

    Serial.println("Adafruit MMA8451 test!");

    if (!mma.begin()) {
        Serial.println("Could not start");
        while (1);
    }
    Serial.println("MMA8451 found!");
  
    mma.setRange(MMA8451_RANGE_2_G);

    Serial.print("Range = "); Serial.print(2 << mma.getRange());  
    Serial.println("G");

    long sampleTime = measureSampleTime();
    Serial.print("Time between Samples: ");
    Serial.print(sampleTime);
    Serial.println(" milliseconds");
}

void loop() {
    check_panic_button(); // Check for panic button press
    mma.read();  // Read new data from accelerometer

    Serial.printf("TS: %lu\tX: %.3f\t Y: %.3f\t Z: %.3f M/S^2\n", millis(), 
        GtoMeterSQConversion(mma.x), GtoMeterSQConversion(mma.y), GtoMeterSQConversion(mma.z)); 

    Serial.println(); 
    delay(500);
}

// Function for converting raw values to meters per second squared
float GtoMeterSQConversion(int rawValue) {
    return (rawValue / 4096.0) * 9.80665;
}

// Measure sample time
long measureSampleTime() {
    long t1 = millis();
    mma.read();
    long t2 = millis();
    return (t2 - t1);
}

// Panic button check function
void check_panic_button() {
    unsigned long pressStartTime = 0;   //Unsigned long for safer code prevents overflow issues, a long can be negative which may cause overflow issues
    bool buttonPressed = false;

    while (digitalRead(PANIC_BUTTON_PIN) == LOW) {  // Button is pressed
        if (!buttonPressed) {
            pressStartTime = millis();
            buttonPressed = true;
        }
        
        if (millis() - pressStartTime >= PANIC_HOLD_TIME) {  
            Serial.println(" PANIC BUTTON ALERT! ");
            alarm();  // Call alarm function (if you want to send an email or alert)
            break;
        }
        delay(50); // Debounce
    }
}

// Alarm function (modify as needed)
void alarm() {
    Serial.println("ALERT: Emergency triggered!");
    // Add email sending or buzzer activation here
}
