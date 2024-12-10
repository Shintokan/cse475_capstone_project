#include <BfButton.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <HardwareSerial.h>

// Set SCL and SDA pins for ToF sensors
#define SCL_PIN 5 // Set pin 9 as SCL pin
#define SDA_PIN 6 // Set pin 10 as SDA pin
 
#define vibrationPIN 45 // vibration motor
#define buttonPIN 11 // encoder button
#define DT 12 // encoder
#define CLK 10 // encoder

#define BUZZER_PIN 47 // buzzer output pin
#define SAMPLE_RATE 5000    // Set sample rate
#define RESOLUTION 8        // PWM resolution

BfButton btn(BfButton::STANDALONE_DIGITAL, buttonPIN, true, LOW);
VL53L1X sensor;

// Establish UART for inter-board communication
HardwareSerial senseSerial(1); // Using UART1
const unsigned long TIMEOUT = 10000; // 10 seconds timeout

// Threshold distance in cm (triggers green color)
const float THRESHOLD_MIN = 1000.0; // 0.3 meters
float THRESHOLD_MAX = 2300.0;

// Controlling vibration pulsing
unsigned long lastPulseTime = 0;
bool pulseState = false;

// Controlling encoder rotation
int distCounter = 16; // starts at 1.5m
int volumeCounter = 25; // starts at 25% (25 of 100)
int aState;
int aLastState;  

// State definitions for distance calibration
enum SystemState {
  NORMAL,
  CALIBRATION
};
SystemState systemState = NORMAL;

// FreeRTOS task handles
TaskHandle_t taskButtonHandle = NULL;
TaskHandle_t taskDistanceSensorHandle = NULL;
TaskHandle_t taskEncoderHandle = NULL;

// FreeRTOS semaphores
SemaphoreHandle_t i2cSemaphore = NULL;

// RTOS Task - Button
void taskButton(void *pvParameters) {
  for (;;) {
    btn.read();
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent task starvation
  }
}

// RTOS Task - ToF Sensor
void taskDistanceSensor(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdTRUE) {
      float distance = poll_sensor();
      xSemaphoreGive(i2cSemaphore);

      if (systemState == NORMAL) {
        if (distance >= THRESHOLD_MIN && distance <= THRESHOLD_MAX) {
          unsigned long currentTime = millis();
          int pulseInterval = map(distance, THRESHOLD_MIN, THRESHOLD_MAX, 50, 1000);
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
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // Match the sensor's continuous reading rate
  }
}

// RTOS Task - Encoder
void taskEncoder(void *pvParameters) {
  for (;;) {
    aState = digitalRead(CLK);
    if (aState != aLastState) {     
      if (digitalRead(DT) != aState) { 
        distCounter++;
        volumeCounter++;
      } else {
        distCounter--;
        volumeCounter--;
      }
      distCounter = constrain(distCounter, 0, 32);
      volumeCounter = constrain(volumeCounter, 0, 20);
      if (systemState == CALIBRATION) {
        // distCounter = constrain(distCounter, 0, 32);
        THRESHOLD_MAX = 2300 + (distCounter * 50);
      } else {
        // Adjust volume here
        volumeCounter = constrain(volumeCounter, 0, 20);
      }
    }
    aLastState = aState;
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent task starvation
  }
}

//Button press hanlding function
void pressHandler (BfButton *btn, BfButton::press_pattern_t pattern) {
  switch (pattern) {
    case BfButton::SINGLE_PRESS:
      if (systemState == CALIBRATION) {
        digitalWrite(vibrationPIN, LOW);
        systemState = NORMAL;
        //Exiting calibration mode
      } else {
        take_picture();
        //Picture taken
      }
      break;

    case BfButton::DOUBLE_PRESS:
      break;
      
    // Dial calibration mode
    case BfButton::LONG_PRESS:
      // when long button is pressed, go into the Encoder calibration task
      if (systemState == NORMAL) {
        systemState = CALIBRATION;
        digitalWrite(vibrationPIN, LOW);
      }
      break;
  }
}

// polls and returns sensor value (in mm)
float poll_sensor() {
  float distance = sensor.read();
  return distance;
}

// send signal to ESP32 sense to take and process image
void take_picture() {
  digitalWrite(vibrationPIN, LOW);
  
  senseSerial.flush();
  senseSerial.write('1');

  // Wait 500ms for the ML function to execute
  vTaskDelay(pdMS_TO_TICKS(500));

  // Read the response
  int result = senseSerial.read();
  if (result == 0) {
    ledcWriteTone(BUZZER_PIN, 500);  // play at 500Hz
    vTaskDelay(pdMS_TO_TICKS(2000));  // Play for 200ms
    ledcWriteTone(BUZZER_PIN, 0);    // Stop the sound
  }
  if (result >= 1 && result <= 5) {
    playAudio(result);
  }
}

void playAudio(int count) {
  for (int i = 0; i < count; i++) {
    ledcWriteTone(BUZZER_PIN, 500);  // play at 500Hz
    vTaskDelay(pdMS_TO_TICKS(200));  // Play for 200ms
    ledcWriteTone(BUZZER_PIN, 0);    // Stop the sound
    vTaskDelay(pdMS_TO_TICKS(100));  // Delay in between pulses
  }
}

void tof_sensor_init() {
  if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdTRUE) {
    sensor.setTimeout(500);
    if (!sensor.init())
    {
      while (1);
    }
    sensor.setDistanceMode(VL53L1X::Long); // Sets long distance mode
    sensor.setMeasurementTimingBudget(50000); // Allows 50000us (50 ms) for a measurement

    sensor.startContinuous(50); // Starts continuous readings once every 50ms
    xSemaphoreGive(i2cSemaphore);
  }
}

void setup()
{
  // Initialize serial UART between boards
  senseSerial.begin(115200, SERIAL_8N1, 16, 17); // RX: 18, TX: 17

  // Initialize I2C semaphore
  i2cSemaphore = xSemaphoreCreateMutex();

  // initialize sensor input
  Wire.begin(SDA_PIN, SCL_PIN);
  tof_sensor_init();

  // Pin initializations for vibration motor and speaker
  pinMode(vibrationPIN, OUTPUT);

  // Initializations for buzzer speaker
  pinMode(BUZZER_PIN, OUTPUT);
  ledcAttach(BUZZER_PIN, SAMPLE_RATE, RESOLUTION); // Configure PWM with updated syntax
  
  // Pin initializations for encoder
  pinMode(CLK,INPUT_PULLUP);
  pinMode(DT,INPUT_PULLUP);
  aLastState = digitalRead(CLK);

  //Button settings
  btn.onPress(pressHandler)
  .onDoublePress(pressHandler) // default timeout
  .onPressFor(pressHandler, 1000); // custom timeout for 1 second


  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(taskButton, "ButtonTask", 2048, NULL, 1, &taskButtonHandle, 0);
  xTaskCreatePinnedToCore(taskDistanceSensor, "DistanceTask", 2048, NULL, 2, &taskDistanceSensorHandle, 0);
  xTaskCreatePinnedToCore(taskEncoder, "EncoderTask", 2048, NULL, 1, &taskEncoderHandle, 1);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to prevent watchdog timer issues
}

