#include <PulseSensorPlayground.h>
 
const int PulseWire = 34;       // Analog input pin for the pulse sensor
const int LED13 = 13;           // Pin 13 for the LED
int Threshold = 750;            // Adjust this value to your sensor's sensitivity
 
PulseSensorPlayground pulseSensor;
 
void setup() {
  Serial.begin(115200);
  pulseSensor.analogInput(PulseWire);
  pulseSensor.blinkOnPulse(LED13);
  pulseSensor.setThreshold(Threshold);
 
  if (pulseSensor.begin()) {
    Serial.println("Pulse sensor started successfully.");
  }
}
 
void loop() {
  int myBPM = pulseSensor.getBeatsPerMinute();
  if (pulseSensor.sawStartOfBeat()) {
    Serial.print("BPM: ");
    Serial.println(myBPM);
  }
  delay(20);
}
