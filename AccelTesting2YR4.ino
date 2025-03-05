#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>


#define SDA_I2C 21                  // defining SDA  pin 21    Default pins of the esp32 for I2C communication
#define SCL_I2C 22                  // defining SCL  pin 22 


Adafruit_MMA8451 mma = Adafruit_MMA8451();

float GtoMeterSQConversion(int rawValue); //Function prototype
long measureSampleTime();

void setup(void) {

      Serial.begin(115200);
      Wire.begin(SDA_I2C, SCL_I2C);
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
  

  if (! mma.begin()) {
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("MMA8451 found!");
 /*
  Accelerometer range set to +-2G (MMA8451 has ranges: +-2G, +-4G, +-8G) G = Gravity acceleration
  MMA8451 => In +-2G mode 14-bit Sensor(values -8192 to +8191)
  Each axis measurement range from -8192 to +8191
  1G = 9.80665 m/s^2
  Sensor outputs 4096 counts per G
  +-2G mode => 1G 8192/2 = 4096
  Acceleration (m/s^2) => (Raw value/4096) * 9.80665
 */
  mma.setRange(MMA8451_RANGE_2_G);
/*
  mma.getRange returns an integer value that represents the range setting (0 => +-2G, 1 => +-4G, 2 => +-8G).
  2 << mma.getRange() use's bitwise left shift (<<) operator
  Shifts binary representaion of a number to the left by set number of positions
  (X << N) X left by N number of bits or X * 2.
*/
  Serial.print("Range = "); Serial.print(2 << mma.getRange());  
  Serial.println("G");

}

void loop() {
    mma.read(); // Read new data from accelerometer

    long sampleTime = measureSampleTime(); //Measuring the timing between samples
    Serial.print("Time between Samples: ");
    Serial.print(sampleTime);
    Serial.println(" microSeconds");
    
    
    Serial.print("Raw Data:");
    Serial.print("\t X: ");Serial.print(mma.x);  
    Serial.print("\t\t Y: ");Serial.print(mma.y);
    Serial.print("\t Z: ");Serial.print(mma.z);
    //Serial.println("\n");
    Serial.println(); 
  
    Serial.printf("TS: %lu\tX: %.3f\t Y: %.3f\t Z: %.3f M/S^2\n", millis(), GtoMeterSQConversion(mma.x), GtoMeterSQConversion(mma.y), GtoMeterSQConversion(mma.z)); 
    Serial.println(); 
    delay(500);

}
// Function for conversion of rawValues to meters per second squared 1G = 9.80665
float GtoMeterSQConversion(int rawValue) {
  return(rawValue / 4096.0) * 9.80665;
} 

long measureSampleTime () {
    long t1 = micros();
    mma.read();
    long t2 = micros();

    //Serial.print("t1: ");Serial.print(t1);
    //Serial.print(" :: t2: ");Serial.print(t2);
    //Serial.print(" :: Diff ");Serial.print(t2 - t1);
    return (t2 - t1);
}
