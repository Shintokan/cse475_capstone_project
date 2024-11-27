/* Includes ---------------------------------------------------------------- */
#include <haptic-cane-new-model_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include <WiFi.h>
// CHANGE: adding in info for sd card
#include "FS.h"
#include "SD.h"
#include "SPI.h"

int imageCount = 1;       // File Counter
bool camera_sign = false; // Check camera status
bool sd_sign = false;     // Check sd status

#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
#include "camera_pins.h"

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; // points to the output of the capture

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,   // QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, // 0-63 lower number means higher quality
    .fb_count = 1,      // if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
void writeFile(fs::FS &fs, const char *path, uint8_t *data, size_t len);

// SD card write file
void writeFile(fs::FS &fs, const char *path, uint8_t *data, size_t len)
{
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.write(data, len) == len)
  {
    Serial.println("File written");
  }
  else
  {
    Serial.println("Write failed");
  }
  file.close();
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
  // we already have a RGB888 buffer, so recalculate offset into pixel index
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0)
  {
    out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];

    // go to the next pixel
    out_ptr_ix++;
    pixel_ix += 3;
    pixels_left--;
  }

  // and done
  return 0;
}

/**
 * @brief      Arduino setup function
 */
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  // comment out the below line to start inference immediately after upload
  while (!Serial)
    ;
  Serial.println("Edge Impulse Inferencing Demo");
  if (ei_camera_init() == false)
  {
    ei_printf("Failed to initialize Camera!\r\n");
  }
  else
  {
    ei_printf("Camera initialized\r\n");
  }

  ei_printf("\nStarting continious inference in 2 seconds...\n");
  ei_sleep(2000);

  // CHANGE: making sure sd card is initialized properly
  if (!SD.begin(21))
  {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized successfully");
}

/**
 * @brief      Get data and run inferencing
 *
 * @param[in]  debug  Get debug info if true
 */
void loop()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "c")
    {
      snapshot_buf = (uint8_t *)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

      // check if allocation was successful
      if (snapshot_buf == nullptr)
      {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
      }

      ei::signal_t signal;
      signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
      signal.get_data = &ei_camera_get_data;

      if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false)
      {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
      }

      // print few pixels to make sure captued image is not empty
      ei_printf("First few pixels: %d, %d, %d\r\n", snapshot_buf[0], snapshot_buf[1], snapshot_buf[2]);

      // Run the classifier
      ei_impulse_result_t result = {0};

      EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
      if (err != EI_IMPULSE_OK)
      {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
      }

      // print the predictions
      ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
      ei_printf("Object detection bounding boxes:\r\n");
      ei_printf("Number of bounding boxes: %d\r\n", result.bounding_boxes_count);
      // result.bounding_boxes_count

      const char *highest_confidence_label = NULL;
      float highest_confidence = 0.0f;

      for (uint32_t i = 0; i < result.bounding_boxes_count; i++)
      {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);

        if (bb.value == 0)
        {
          continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                  bb.label,
                  bb.value,
                  bb.x,
                  bb.y,
                  bb.width,
                  bb.height);
        if (bb.value > highest_confidence)
        {
          highest_confidence = bb.value;
          highest_confidence_label = bb.label;
        }
      }
      if (highest_confidence_label != NULL)
      {
        ei_printf("Object with highest confidence: %s (confidence: %f)\r\n",
                  highest_confidence_label, highest_confidence);
      }
      else
      {
        ei_printf("No objects detected.\r\n");
      }
#endif
      free(snapshot_buf);
    }
  }
}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void)
{
  if (is_initialised)
    return true;

  // initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  is_initialised = true;
  return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void)
{

  // deinitialize the camera
  esp_err_t err = esp_camera_deinit();

  if (err != ESP_OK)
  {
    ei_printf("Camera deinit failed\n");
    return;
  }

  is_initialised = false;
  return;
}

/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf)
{
  bool do_resize = false;

  if (!is_initialised)
  {
    ei_printf("ERR: Camera is not initialized\r\n");
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb)
  {
    ei_printf("Camera capture failed\n");
    return false;
  }
  const char *filePath = "/image.jpg";

  writeFile(SD, filePath, fb->buf, fb->len);

  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

  esp_camera_fb_return(fb);

  if (!converted)
  {
    ei_printf("Conversion failed\n");
    return false;
  }

  if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS))
  {
    do_resize = true;
  }

  if (do_resize)
  {
    ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
  }

  return true;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif