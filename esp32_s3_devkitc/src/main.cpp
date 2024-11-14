/*
Things to implement:


Audio data:
- bluetooth A2DP protocol --> set up this board as a source
- set up bidirectional communication for ESP-NOW on both boards
- error handling for ESP-NOW communication
- reconnection logic for bluetooth headphones
- edge cases for missing audio files or communication failures


Power management:
- low-power modes for both boards
- optimize bluetooth and ESP-now to conserve battery


*/
#include <BfButton.h>
#include <Wire.h>
#include <VL53L1X.h>

// Set SCL and SDA pins for ToF sensors
#define SCL_PIN 9 // Set pin 9 as SCL pin
#define SDA_PIN 10 // Set pin 10 as SDA pin
 
#define picturePIN 6 // picture digital output
#define vibrationPIN 7 // vibration motor
#define buttonPIN 11 // encoder button
#define DT 12 // encoder
#define CLK 13 // encoder

BfButton btn(BfButton::STANDALONE_DIGITAL, buttonPIN, true, LOW);
VL53L1X sensor;

// Threshold distance in cm (triggers green color)
const float THRESHOLD_MIN = 0.0; // 0.3 meters
float THRESHOLD_MAX = 1500.0;

// Controlling vibration pulsing
unsigned long lastPulseTime = 0;
bool pulseState = false;

// Controlling encoder rotation
int counter = 0;
int aState;
int aLastState;  

enum SystemState {
  NORMAL,
  CALIBRATION
};

SystemState systemState = NORMAL;

//Button press hanlding function
void pressHandler (BfButton *btn, BfButton::press_pattern_t pattern) {
  switch (pattern) {
    case BfButton::SINGLE_PRESS:
      if (systemState == CALIBRATION) {
        digitalWrite(vibrationPIN, LOW);
        systemState = NORMAL;
        //Exiting calibration mode
      } else {
        digitalWrite(vibrationPIN, LOW); // make sure vibration is turned off before starting image sequence

        take_picture();
        //Picture taken
      }
      break;

    case BfButton::DOUBLE_PRESS:
      digitalWrite(picturePIN, HIGH);
      delay(1000);
      digitalWrite(picturePIN, LOW);
      break;
      
    // Dial calibration mode
    case BfButton::LONG_PRESS:
      // when long button is pressed, go into the Encoder calibration task
      if (systemState == NORMAL) {
        systemState = CALIBRATION;
        digitalWrite(vibrationPIN, HIGH);
        // enter calibration mode

        // ensure audio is in format 44.1 kHz, 16-bit stereo PCM
        // Audio say "entering calibration mode"
        // "maximum distance threshold set to THRESHOLD_MAX
        // "one notch to the left decreases threshold by 100mm"
        // "one notch to the right increases threshold by 100mm"

        // using ESP-NOW:
        // 1. request audio file from SD card of the sense --> need to set up bidirectional communication
        // 2. ensure audio compatibility (44.1kHz, 16-bit stereo PCM)
        // 3. using A2DP, send data via blyetooth
      }
      break;
  }
}

void handleEncoderCalibration() {
  aState = digitalRead(CLK);
  if (aState != aLastState) {     
    if (digitalRead(DT) != aState) { 
      counter++;
    } else {
      counter--;
    }
    counter = constrain(counter, 0, 32);
    THRESHOLD_MAX = 1500 + (counter * 50);
  }
  aLastState = aState;

  // TO ADD: functionality to buzz the user if they reach max dial bounds (counter == 0 or 32)
}


// polls and returns sensor value (in mm)
float poll_sensor() {
  float distance = sensor.read();
  return distance;
}

// send signal to ESP32 sense to take and process image
// IMPLEMENT:
// - communication to the esp32 sense
// - image processing ML
// - classified image --> audio output
void take_picture() {
  digitalWrite(picturePIN, HIGH);
  delay(1000);
  digitalWrite(picturePIN, LOW);
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
  Serial.begin(9600);
  // initialize sensor input
  Wire.begin(SDA_PIN, SCL_PIN);
  tof_sensor_init();

  // Pin initializations for sending signals to vibration motor and the picture modules
  pinMode(picturePIN, OUTPUT);
  pinMode(vibrationPIN, OUTPUT);
  
  // Pin initializations for encoder
  pinMode(CLK,INPUT_PULLUP);
  pinMode(DT,INPUT_PULLUP);
  aLastState = digitalRead(CLK);

  //Button settings
  btn.onPress(pressHandler)
  .onDoublePress(pressHandler) // default timeout
  .onPressFor(pressHandler, 1000); // custom timeout for 1 second
  // pixels.begin();
}

void loop()
{
  btn.read(); // Read button state

  if (systemState == NORMAL) {
    float distance = poll_sensor();

    if (distance >= THRESHOLD_MIN && distance <= THRESHOLD_MAX) {
      unsigned long currentTime = millis();
      int pulseInterval = map(distance, THRESHOLD_MIN, THRESHOLD_MAX, 50, 500);
      if (!pulseState && currentTime - lastPulseTime >= pulseInterval) {
        digitalWrite(vibrationPIN, HIGH);
        pulseState = true;
        lastPulseTime = currentTime;
      } else if (pulseState && currentTime - lastPulseTime >= 100) {
        digitalWrite(vibrationPIN, LOW);
        pulseState = false;
        lastPulseTime = currentTime;
      }
    } else {
      digitalWrite(vibrationPIN, LOW);
      pulseState = false;
    }
  } else if (systemState == CALIBRATION) {
    handleEncoderCalibration();
  }
}

