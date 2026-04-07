#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

/*
  ========================================================================
  Arduino Uno Test Program for the TCS3472x Color Sensor
  ========================================================================
  
  HOW TO USE (Arduino IDE):
  1. Open this file in the Arduino IDE.
  2. Go to: Sketch -> Include Library -> Manage Libraries
  3. Search for "Adafruit TCS34725" and install it.
  4. Ensure your TCS3472 sensor is wired to the standard Uno I2C pins:
     - VIN  -> 5V or 3.3V (most Adafruit/generic breakouts have a 3.3V regulator, so 5V is fine)
     - GND  -> GND
     - SCL  -> A5 (Analog 5) or the dedicated SCL pin near the USB port
     - SDA  -> A4 (Analog 4) or the dedicated SDA pin near the USB port
     
  5. Upload to your Arduino Uno and open the Serial Monitor at 115200 baud!
*/

// Initialise with 50ms integration time and 4X gain (good standard values)
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(115200);
  
  // Wait to ensure Serial monitor connects before printing
  delay(1000); 
  Serial.println("\n--- Arduino Uno TCS3472 Color Sensor Test ---");

  // Initialize the sensor
  if (tcs.begin()) {
    Serial.println("Success: Found TCS3472 Sensor on I2C bus!");
  } else {
    Serial.println("ERROR: No TCS3472 found. Check your A4/A5 connections!");
    while (1) {
      delay(100); // Halt execution if sensor not found
    } 
  }
}

void loop() {
  uint16_t r, g, b, c, colorTemp, lux;

  // Grab the raw reading
  tcs.getRawData(&r, &g, &b, &c);
  
  // Calculate relative color temperature and Lux
  // dn40 is Adafruit's newer, more accurate temperature algorithm
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  // Print results
  // C = "Clear" light (unfiltered intensity)
  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K | ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" | ");
  
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.println(c, DEC);

  // Wait half a second between scans
  delay(500);
}
