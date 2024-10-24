#include <Arduino.h>
#include <HCSR04.h>            // HCSR04 ultrasonic sensor@^2.0.3 library
#include <Adafruit_NeoPixel.h> // onboard RGB library

// define allocates to Flash Memory (non-volatile)

#define PIN 38 // RGB
#define NUMPIXELS 1

// sensor 1
const uint8_t trig1 = 6;
const uint8_t echo1 = 7;

// sensor 2
const uint8_t trig2 = 12;
const uint8_t echo2 = 11;

// onboard RGB setup
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// setting up both sensors
HCSR04 sensor1(trig1, echo1);
HCSR04 sensor2(trig2, echo2);

// Threshold distance in cm (triggers green color)
const float THRESHOLD = 10.0;

void setup()
{
  Serial.begin(9600);
  pixels.begin();
}

void loop()
{
  // get distance measurements
  float dist1 = sensor1.dist();
  float dist2 = sensor2.dist();

  // print statements for debugging
  Serial.print("sensor 1: ");
  Serial.println(dist1);
  Serial.print("sensor 2: ");
  Serial.println(dist2);

  // if object is within the THRESHOLD
  if (dist1 <= THRESHOLD && dist2 <= THRESHOLD)
  {
    Serial.println("THRESHOLD MET!");
    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // GREEN
  }
  else
  {
    pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // RED
  }
  pixels.show();
  delay(100);
}
