#include <Arduino.h>
#include <Adafruit_NeoPixel.h> // onboard RGB library

#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

// define allocates to Flash Memory (non-volatile)
#define PIN 38 // RGB
#define NUMPIXELS 1

// Set SCL and SDA pins
#define SCL_PIN 9 // Set pin 9 as SCL pin
#define SDA_PIN 10 // Set pin 10 as SDA pin

// onboard RGB setup
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Threshold distance in cm (triggers green color)
const float THRESHOLD_MIN = 300.0; // 0.3 meters
const float THRESHOLD_MAX = 1500.0; // 1.5 meters

void setup()
{
  // while (!Serial) {}
  Wire.begin(SDA_PIN, SCL_PIN);

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(50);

  Serial.begin(9600);
  pixels.begin();
}

void loop()
{
  const float distance = sensor.read();
  Serial.println(distance);

  // if object is within the THRESHOLD
  if (distance <= THRESHOLD_MAX && distance >= THRESHOLD_MIN) {
    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // GREEN
   } else {
    pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // RED
  }
  pixels.show();
  delay(100);
}
