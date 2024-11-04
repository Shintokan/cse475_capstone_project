#include <Arduino.h>
#include <Adafruit_NeoPixel.h> // onboard RGB library

#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

// define allocates to Flash Memory (non-volatile)
// #define PIN 38 // RGB
// #define NUMPIXELS 1

// Set SCL and SDA pins
#define SCL_PIN 9 // Set pin 9 as SCL pin
#define SDA_PIN 10 // Set pin 10 as SDA pin

#define buttonPIN 18
#define picturePIN 6

#define vibrationPIN 7

// onboard RGB setup
// Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Threshold distance in cm (triggers green color)
const float THRESHOLD_MIN = 300.0; // 0.3 meters
const float THRESHOLD_MAX = 3000.0; // 3 meters

int buttonState = 0;

unsigned long lastPulseTime = 0;
bool pulseState = false;

// polls button press
int poll_button() {
  return digitalRead(buttonPIN);
}

// polls and returns sensor value (in mm)
float poll_sensor() {
  float distance = sensor.read();
  Serial.println(distance);

  return distance;
}

// send signal to ESP32 sense to take and process image
// IMPLEMENT:
// - communication to the esp32 sense
// - image processing ML
// - classified image --> audio output
void take_picture() {
 digitalWrite(picturePIN, HIGH);
 delay(100);
 digitalWrite(picturePIN, LOW);

 while (digitalRead(buttonPIN) == HIGH) {
    if (digitalRead(buttonPIN) == LOW) {
      break;
    }
 }
}

void send_vibration() {
  digitalWrite(picturePIN, HIGH);
}

void tof_sensor_init() {
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
}

void setup()
{
  // initialize
  Wire.begin(SDA_PIN, SCL_PIN);

  // initialize sensor input
  tof_sensor_init();

  pinMode(buttonPIN, INPUT);
  pinMode(picturePIN, OUTPUT);
  pinMode(vibrationPIN, OUTPUT);
  
  Serial.begin(9600);
  // pixels.begin();
}

void loop()
{
  if (poll_button() == HIGH) {
    // if button is pressed, stop vibrating
    digitalWrite(vibrationPIN, LOW);

    // begin picture taking/recognition process
    take_picture();
  }

  float distance = poll_sensor();
  // if object is within the THRESHOLD
  if (distance >= THRESHOLD_MIN && distance <= THRESHOLD_MAX) {
    // pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // GREEN
    int pulseInterval = map(distance, THRESHOLD_MIN, THRESHOLD_MAX, 50, 500); // pulses between 50ms and 500ms based on threshold range

    // Pulse the vibration
    unsigned long currentTime = millis();
    if (currentTime - lastPulseTime >= pulseInterval) {
      pulseState = !pulseState;
      digitalWrite(vibrationPIN, pulseState);
      lastPulseTime = currentTime;
    }
   } else {
    // pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // RED
    digitalWrite(vibrationPIN, LOW);
  }
  // pixels.show();
  delay(10);
}
