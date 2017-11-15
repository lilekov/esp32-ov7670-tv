// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <byteswap.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// #define ESPIDFV21RC 1

#ifdef ESPIDFV21RC
  #include "esp_heap_alloc_caps.h"
#else
  #include "esp_heap_caps.h"
#endif

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "esp_attr.h"

#include "soc/gpio_struct.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "camera.h"

#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include "bitmap.h"

#include "telnet.h"

static const char* TAG = "ESPILICAM";

SemaphoreHandle_t dispSem = NULL;
SemaphoreHandle_t dispDoneSem = NULL;
SemaphoreHandle_t captureDoneSem = NULL;
SemaphoreHandle_t captureSem = NULL;

void capture_wait_finish() {
  xSemaphoreTake(captureDoneSem, portMAX_DELAY);
}

void capture_request() {
  xSemaphoreGive(captureSem);
}


static EventGroupHandle_t espilicam_event_group;
EventBits_t uxBits;
const int MOVIEMODE_ON_BIT = BIT0;

bool is_moviemode_on()
{
    return (xEventGroupGetBits(espilicam_event_group) & MOVIEMODE_ON_BIT) ? 1 : 0;
}

static void set_moviemode(bool c) {
    if (is_moviemode_on() == c) {
        return;
    } else {
      if (c) {
      xEventGroupSetBits(espilicam_event_group, MOVIEMODE_ON_BIT);
      } else {
      xEventGroupClearBits(espilicam_event_group, MOVIEMODE_ON_BIT);
      }
    }
}

static void captureTask(void *pvParameters) {
  err_t err;
  bool movie_mode = false;
  xSemaphoreGive(captureDoneSem);
  while(1) {
     //frame++;
     movie_mode = is_moviemode_on();
     if (!movie_mode)
     xSemaphoreTake(captureSem, portMAX_DELAY);

     err = camera_run();

     if (!movie_mode)
       xSemaphoreGive(captureDoneSem);
     // only return when LCD finished display .. sort of..
   } // end while(1)

}

// CAMERA CONFIG

static camera_pixelformat_t s_pixel_format;
static camera_config_t config = {
    .ledc_channel = LEDC_CHANNEL_0,
    .ledc_timer = LEDC_TIMER_0,
    .pin_d0 = CONFIG_D0,
    .pin_d1 = CONFIG_D1,
    .pin_d2 = CONFIG_D2,
    .pin_d3 = CONFIG_D3,
    .pin_d4 = CONFIG_D4,
    .pin_d5 = CONFIG_D5,
    .pin_d6 = CONFIG_D6,
    .pin_d7 = CONFIG_D7,
    .pin_xclk = CONFIG_XCLK,
    .pin_pclk = CONFIG_PCLK,
    .pin_vsync = CONFIG_VSYNC,
    .pin_href = CONFIG_HREF,
    .pin_sscb_sda = CONFIG_SDA,
    .pin_sscb_scl = CONFIG_SCL,
    .pin_reset = CONFIG_RESET,
    .xclk_freq_hz = CONFIG_XCLK_FREQ,
    .test_pattern_enabled = CONFIG_ENABLE_TEST_PATTERN,
    };

static camera_model_t camera_model;

#define CAMERA_PIXEL_FORMAT CAMERA_PF_RGB565
//#define CAMERA_PIXEL_FORMAT CAMERA_PF_YUV422
#define CAMERA_FRAME_SIZE CAMERA_FS_QVGA


// DISPLAY LOGIC
static inline uint8_t clamp(int n)
{
    n = n>255 ? 255 : n;
    return n<0 ? 0 : n;
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
static inline uint16_t ILI9341_color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

uint16_t get_grayscale_pixel_as_565(uint8_t pix) {
    // R = (img[n]&248)<<8; // 5 bit cao cua Y
    // G = (img[n]&252)<<3; // 6 bit cao cua Y
    // B = (img[n]&248)>>3; // 5 bit cao cua Y
    uint16_t graypixel=((pix&248)<<8)|((pix&252)<<3)|((pix&248)>>3);
    return graypixel;

}

// integers instead of floating point...
static inline uint16_t fast_yuv_to_rgb565(int y, int u, int v) {
int a0 = 1192 * (y - 16);
int a1 = 1634 * (v - 128);
int a2 = 832 * (v - 128);
int a3 = 400 * (u - 128);
int a4 = 2066 * (u - 128);
int r = (a0 + a1) >> 10;
int g = (a0 - a2 - a3) >> 10;
int b = (a0 + a4) >> 10;
return ILI9341_color565(clamp(r),clamp(g),clamp(b));

}

// fast but uses floating points...
static inline uint16_t fast_pascal_to_565(int Y, int U, int V) {
  uint8_t r, g, b;
  r = clamp(1.164*(Y-16) + 1.596*(V-128));
  g = clamp(1.164*(Y-16) - 0.392*(U-128) - 0.813*(V-128));
  b = clamp(1.164*(Y-16) + 2.017*(U-128));
  return ILI9341_color565(r,g,b);
}

//Warning: This gets squeezed into IRAM.
volatile static uint32_t *currFbPtr __attribute__ ((aligned(4))) = NULL;

inline uint8_t unpack(int byteNumber, uint32_t value) {
    return (value >> (byteNumber * 8));
}

// camera code

const static char http_hdr[] = "HTTP/1.1 200 OK\r\n";
const static char http_stream_hdr[] =
        "Content-type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n\r\n";
const static char http_jpg_hdr[] =
        "Content-type: image/jpg\r\n\r\n";
const static char http_pgm_hdr[] =
        "Content-type: image/x-portable-graymap\r\n\r\n";
const static char http_stream_boundary[] = "--123456789000000000000987654321\r\n";
const static char http_bitmap_hdr[] =
        "Content-type: image/bitmap\r\n\r\n";
const static char http_yuv422_hdr[] =
        "Content-Disposition: attachment; Content-type: application/octet-stream\r\n\r\n";

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
static ip4_addr_t s_ip_addr;


// command parser...
#include "smallargs.h"

#define UNUSED(x) ((void)x)
#define RESPONSE_BUFFER_LEN 256
#define CMD_BUFFER_LEN 128

static sarg_root root;
static char telnet_cmd_response_buff[RESPONSE_BUFFER_LEN];
static char telnet_cmd_buffer[CMD_BUFFER_LEN];

static void handle_camera_config_chg(bool reinit_reqd) {
    if (reinit_reqd) {
    ESP_LOGD(TAG, "Reconfiguring camera...");
    esp_err_t err;
    vTaskDelay(100 / portTICK_RATE_MS);
    err = reset_pixformat();
    config.pixel_format = s_pixel_format;
    err = camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        //return;
    }
    return;
    vTaskDelay(100 / portTICK_RATE_MS);
    }
}


static int help_cb(const sarg_result *res)
{
    UNUSED(res);
    char *buf;
    int ret;

    ret = sarg_help_text(&root, &buf);
    if(ret != SARG_ERR_SUCCESS)
        return ret;
//    we can't spare much memory!!
//    int length = 0;
//    length += sprintf(telnet_cmd_response_buff+length, "%s\n", buf);
//    ESP_LOGD(TAG," help_cb: %s",telnet_cmd_response_buff);
//    telnet_esp32_sendData((uint8_t *)telnet_cmd_response_buff, strlen(telnet_cmd_response_buff));
    telnet_esp32_sendData((uint8_t *)buf, strlen(buf));
    free(buf);
    return 0;
}

static int sys_stats_cb(const sarg_result *res)
{
     uint8_t level = 0;
     size_t free8start=0, free32start=0, free8=0, free32=0, tstk=0;
     level = res->int_val;
     uint8_t length = 0;
     if (level == 0) {

       #ifdef ESPIDFV21RC
           free8=xPortGetFreeHeapSizeCaps(MALLOC_CAP_8BIT);
           free32=xPortGetFreeHeapSizeCaps(MALLOC_CAP_32BIT);
           free8start=xPortGetMinimumEverFreeHeapSizeCaps(MALLOC_CAP_8BIT);
           free32start=xPortGetMinimumEverFreeHeapSizeCaps(MALLOC_CAP_32BIT);
       #else
           free32=heap_caps_get_largest_free_block(MALLOC_CAP_32BIT);
           free8=heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
           free8start=heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
           free32start=heap_caps_get_minimum_free_size(MALLOC_CAP_32BIT);
       #endif

      tstk = uxTaskGetStackHighWaterMark(NULL);
      length += sprintf(telnet_cmd_response_buff+length,
        "Stack: %db, free 8-bit=%db, free 32-bit=%db, min 8-bit=%db, min 32-bit=%db.\n",
        tstk,free8,free32, free8start, free32start);
     } else if (level == 1) {
      //vTaskList(telnet_cmd_response_buff);
      //vTaskGetRunTimeStats(telnet_cmd_response_buff);
      length += sprintf(telnet_cmd_response_buff+length, "not implemented\n");
     }

    telnet_esp32_sendData((uint8_t *)telnet_cmd_response_buff, strlen(telnet_cmd_response_buff));
    return SARG_ERR_SUCCESS;
}

static int  videomode_cb(const sarg_result *res) {
      int bval = 0;
      uint8_t length = 0;
      bval = res->int_val;
      bool movie_mode = false;
      // let capture task handle this...
      if (bval == 0) movie_mode = false;
      else if (bval == 1) movie_mode = true;
      else {
      movie_mode = true;
      }
       // set event group...
       set_moviemode(movie_mode);
/*
       if (movie_mode)
        xEventGroupSetBits(espilicam_event_group, MOVIEMODE_ON_BIT);
       else
        xEventGroupClearBits(espilicam_event_group, MOVIEMODE_ON_BIT);
*/
       if (movie_mode ) {
         // start capture task!
         length += sprintf(telnet_cmd_response_buff+length, "video mode on\n");
         capture_request();
       } else {
         capture_wait_finish();
         length += sprintf(telnet_cmd_response_buff+length, "video mode off\n");
       }
       telnet_esp32_sendData((uint8_t *)telnet_cmd_response_buff, strlen(telnet_cmd_response_buff));
      return SARG_ERR_SUCCESS;
}

static int  ov7670_xclck_cb(const sarg_result *res) {
      int speed = 0;
      speed = res->int_val;
      ESP_LOGD(TAG, "Switch XCLCK to %dMHZ",speed);
      speed = speed * 1000000;
      config.xclk_freq_hz = speed;
      reset_xclk(&config);
      handle_camera_config_chg(true);
      return SARG_ERR_SUCCESS;
}

static int  ov7670_pixformat_cb(const sarg_result *res) {
  if (strcmp("yuv422", res->str_val) == 0) {
    //
    ESP_LOGD(TAG, "Switch pixel format to YUV422");
    s_pixel_format = CAMERA_PF_YUV422;
    handle_camera_config_chg(true);
  } else if (strcmp("rgb565", res->str_val) == 0) {
    //
    ESP_LOGD(TAG, "Switch pixel format to RGB565");
    s_pixel_format = CAMERA_PF_RGB565;
    handle_camera_config_chg(true);
  }
  return SARG_ERR_SUCCESS;
}

static int  ov7670_framerate_cb(const sarg_result *res) {
  int framerate = 0;
  framerate = res->int_val;
  ESP_LOGD(TAG, "Switch framerate to %dFPS",framerate);
  sensor_t* s_sensor = get_cam_sensor();
  if (s_sensor != NULL) {
    // this framerate parameter is a hack, openmv params are different
    if (framerate == 14) s_sensor->set_framerate(s_sensor,0);
    if (framerate == 15) s_sensor->set_framerate(s_sensor,1);
    if (framerate == 25) s_sensor->set_framerate(s_sensor,2);
    if (framerate == 30) s_sensor->set_framerate(s_sensor,3);
  //  handle_camera_config_chg(true);
  }
  return SARG_ERR_SUCCESS;
}

static int  ov7670_colorbar_cb(const sarg_result *res) {

  bool onoff = false;
  if (res->int_val == 1) onoff = true;
  config.test_pattern_enabled = onoff;
  sensor_t* s_sensor = get_cam_sensor();
  if (s_sensor != NULL) {
      ESP_LOGD(TAG, "Set Colorbar (Test Pattern) %d",config.test_pattern_enabled);
      s_sensor->set_colorbar(s_sensor, config.test_pattern_enabled);
  }
  handle_camera_config_chg(false);
  return SARG_ERR_SUCCESS;
}


static int  ov7670_saturation_cb(const sarg_result *res) {
  int level = 0;
  level = res->int_val;
  ESP_LOGD(TAG, "Switch saturation (0-256) to %d",level);
  sensor_t* s_sensor = get_cam_sensor();
  if (s_sensor != NULL) {
    s_sensor->set_saturation(s_sensor,level);
  }
  return SARG_ERR_SUCCESS;
}

static int  ov7670_hue_cb(const sarg_result *res) {
  int level = 0;
  level = res->int_val;
  ESP_LOGD(TAG, "Switch saturation (-180 to 180) to %d",level);
  sensor_t* s_sensor = get_cam_sensor();
  if (s_sensor != NULL) {
    s_sensor->set_hue(s_sensor,level);
  }
  return SARG_ERR_SUCCESS;
}


static int  ov7670_brightness_cb(const sarg_result *res) {
  int level = 0;
  level = res->int_val;
  ESP_LOGD(TAG, "Switch brightness (-4 to 4) to %d",level);
  sensor_t* s_sensor = get_cam_sensor();
  if (s_sensor != NULL) {
    s_sensor->set_brightness(s_sensor,level);
  }
  return SARG_ERR_SUCCESS;
}

static int  ov7670_contrast_cb(const sarg_result *res) {
  int level = 0;
  level = res->int_val;
  ESP_LOGD(TAG, "Switch contrast (-4 to 4) to %d",level);
  sensor_t* s_sensor = get_cam_sensor();
  if (s_sensor != NULL) {
    s_sensor->set_contrast(s_sensor,level);
  }
  return SARG_ERR_SUCCESS;
}


static int  ov7670_hflip_cb(const sarg_result *res) {
  //set_vflip
  bool onoff = false;
  if (res->int_val == 1) onoff = true;
  sensor_t* s_sensor = get_cam_sensor();
  if (s_sensor != NULL) {
      ESP_LOGD(TAG, "Set hflip = %d",onoff);
      s_sensor->set_hmirror(s_sensor, onoff);
  }
  //handle_camera_config_chg(false);
  return SARG_ERR_SUCCESS;
}

static int  ov7670_vflip_cb(const sarg_result *res) {
  bool onoff = false;
  if (res->int_val == 1) onoff = true;
  sensor_t* s_sensor = get_cam_sensor();
  if (s_sensor != NULL) {
      ESP_LOGD(TAG, "Set vflip = %d",onoff);
      s_sensor->set_vflip(s_sensor, onoff);
  }
  return SARG_ERR_SUCCESS;
}


static int  ov7670_lightmode_cb(const sarg_result *res) {
  int level = 0;
  level = res->int_val;
  ESP_LOGD(TAG, "Switch lightmode (0 - 5) to %d",level);
  sensor_t* s_sensor = get_cam_sensor();
  if (s_sensor != NULL) {
    s_sensor->set_ov7670_light_mode(s_sensor,level);
  }
  return SARG_ERR_SUCCESS;
}


static int  ov7670_nightmode_cb(const sarg_result *res) {
  int level = 0;
  level = res->int_val;
  ESP_LOGD(TAG, "Switch nightmode effect (0 - 3) to %d",level);
  sensor_t* s_sensor = get_cam_sensor();
  if (s_sensor != NULL) {
    s_sensor->set_ov7670_night_mode(s_sensor,level);
  }
  return SARG_ERR_SUCCESS;
}


static int  ov7670_special_effects_cb(const sarg_result *res) {
  int level = 0;
  level = res->int_val;
  ESP_LOGD(TAG, "Switch special effect (0 - 8) to %d",level);
  sensor_t* s_sensor = get_cam_sensor();
  if (s_sensor != NULL) {
    s_sensor->set_special_effect(s_sensor,level);
  }
  return SARG_ERR_SUCCESS;
}


static int  ov7670_gamma_cb(const sarg_result *res) {
  int level = 0;
  level = res->int_val;
  ESP_LOGD(TAG, "Switch gamma (0 - 1) to %d",level);
  sensor_t* s_sensor = get_cam_sensor();
  if (s_sensor != NULL) {
    s_sensor->set_ov7670_gamma(s_sensor,level);
  }
  return SARG_ERR_SUCCESS;
}

static int  ov7670_whitebalance_cb(const sarg_result *res) {
  int level = 0;
  level = res->int_val;
  ESP_LOGD(TAG, "Switch whitebalance effect (0 - 2) to %d",level);
  sensor_t* s_sensor = get_cam_sensor();
  if (s_sensor != NULL) {
    s_sensor->set_ov7670_whitebalance(s_sensor,level);
  }
  return SARG_ERR_SUCCESS;
}


const static sarg_opt my_opts[] = {
    {"h", "help", "show help text", BOOL, help_cb},
    {"s", "stats", "system stats (0=mem,1=tasks)", INT, sys_stats_cb},
    {NULL, "clock", "set camera xclock frequency", INT, ov7670_xclck_cb},
    {NULL, "pixformat", "set pixel format (yuv422, rgb565)", STRING, ov7670_pixformat_cb},
    {NULL, "framerate", "set framerate (14,15,25,30)", INT, ov7670_framerate_cb},
    {NULL, "colorbar", "set test pattern (0=off/1=on)", INT, ov7670_colorbar_cb},
    {NULL, "saturation", "set saturation (1-256)", INT, ov7670_saturation_cb},
    {NULL, "hue", "set hue (-180 to 180)", INT, ov7670_hue_cb},
    {NULL, "brightness", "set brightness (-4 to 4)", INT, ov7670_brightness_cb},
    {NULL, "contrast", "set contrast (-4 to 4)", INT, ov7670_contrast_cb},
    {NULL, "hflip", "flip horizontal (0=off/1=on)", INT, ov7670_hflip_cb},
    {NULL, "vflip", "flip vertical (0=off/1=on)", INT, ov7670_vflip_cb},
    {NULL, "light", "ov7670 light mode (0 - 5)", INT, ov7670_lightmode_cb},
    {NULL, "night", "ov7670 night mode (0 - 3)", INT, ov7670_nightmode_cb},
    {NULL, "effect", "special effects (0 - 8)", INT, ov7670_special_effects_cb},
    {NULL, "gamma", "ov7670 gamma mode (0=disabled,1=slope1)", INT, ov7670_gamma_cb},
    {NULL, "whitebalance", "ov7670 whitebalance (0,1,2)", INT, ov7670_whitebalance_cb},
    {NULL, "video", "video mode (0=off,1=on)", INT, videomode_cb},
    {NULL, NULL, NULL, INT, NULL}
};


static int handle_command(uint8_t *cmdLine, size_t len)
{
    int ret = sarg_init(&root, my_opts, "ESPILICAM");
    assert(ret == SARG_ERR_SUCCESS);
    // lots of redundant code here! //strcpy or memcpy would suffice
    size_t cmd_len = len;
    if (len > CMD_BUFFER_LEN) cmd_len = CMD_BUFFER_LEN;
    for (int i = 0; i < cmd_len; i++)
      telnet_cmd_buffer[i] = *(cmdLine+i);
    telnet_cmd_buffer[cmd_len-1] = '\0';
    ESP_LOGD(TAG, "Processing telnet_cmd_buffer len=%d - contents=%s",cmd_len,(char*)telnet_cmd_buffer);
    if(telnet_cmd_buffer != NULL) {
        // next command will call sarg_parse and call callbacks as needed...
        ret = sarg_parse_command_buffer(&root, telnet_cmd_buffer, cmd_len);
        if(ret != SARG_ERR_SUCCESS) {
            ESP_LOGE(TAG, "Command parsing failed");
            sarg_destroy(&root);
            return -1;
        }
        // command has been parsed and executed!
        ESP_LOGD(TAG, "Command parser completed...");
    }
    sarg_destroy(&root);
    return 0;
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            s_ip_addr = event->event_info.got_ip.ip_info.ip;
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            /* This is a workaround as ESP32 WiFi libs don't currently
             auto-reassociate. */
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_NONE) );
    ESP_LOGI(TAG, "Connecting to \"%s\"", wifi_config.sta.ssid);
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected");
}


// COMMAND PARSER
uint8_t getHexVal(char c)
{
   if(c >= '0' && c <= '9')
     return (uint8_t)(c - '0');
   else
     return (uint8_t)(c-'A'+10);
}

static void recvData(uint8_t *buffer, size_t size) {
//  char cmdRecptMessage[100];
  int length = 0;
  ESP_LOGD(TAG, "We received: %.*s", size, buffer);
  handle_command(buffer, size);
  // have to wait for callback for actual response.. echo recpt for now
//  length += sprintf(cmdRecptMessage, "%s","#: ");
//  if (strlen(telnet_cmd_response_buff) > 0)
//    sprintf(cmdRecptMessage+length, "%s\n", telnet_cmd_response_buff);
//  telnet_esp32_sendData((uint8_t *)cmdRecptMessage, strlen(cmdRecptMessage));
}

static void telnetTask(void *data) {
  ESP_LOGD(TAG, "Listening for telnet clients...");
  telnet_esp32_listenForClients(recvData);
  ESP_LOGD(TAG, "stopping telnetTask");
  ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));
  vTaskDelete(NULL);
}

static void convert_fb32bit_line_to_bmp565(uint32_t *srcline, uint8_t *destline, const camera_pixelformat_t format) {

  uint16_t pixel565 = 0;
  uint16_t pixel565_2 = 0;
  uint32_t long2px = 0;
  uint16_t *sptr;
  int current_src_pos=0, current_dest_pos=0;
  for (int current_pixel_pos = 0; current_pixel_pos < 320; current_pixel_pos += 2)
  {
    current_src_pos = current_pixel_pos/2;
    long2px = srcline[current_src_pos];
    if (format == CAMERA_PF_YUV422) {
        uint8_t y1, y2, u, v;
        y1 = unpack(0,long2px);
        v = unpack(1,long2px);;
        y2 = unpack(2,long2px);
        u = unpack(3,long2px);

        pixel565 = fast_yuv_to_rgb565(y1,u,v);
        pixel565_2 = fast_yuv_to_rgb565(y2,u,v);

        sptr = &destline[current_dest_pos];
        *sptr = pixel565;
        sptr = &destline[current_dest_pos+2];
        *sptr = pixel565_2;
        current_dest_pos += 4;

    } else if (format == CAMERA_PF_RGB565) {
      pixel565 =  (unpack(2,long2px) << 8) | unpack(3,long2px);
      pixel565_2 = (unpack(0,long2px) << 8) | unpack(1,long2px);

      sptr = &destline[current_dest_pos];
      *sptr = pixel565;
      sptr = &destline[current_dest_pos+2];
      *sptr = pixel565_2;
      current_dest_pos += 4;
    }
  }
}


// TODO: handle http request while videomode on

static void http_server_netconn_serve(struct netconn *conn)
{
    struct netbuf *inbuf;
    char *buf;
    u16_t buflen;
    err_t err;
    /* Read the data from the port, blocking if nothing yet there.
     We assume the request (the part we care about) is in one netbuf */
    err = netconn_recv(conn, &inbuf);
    if (err == ERR_OK) {
        netbuf_data(inbuf, (void**) &buf, &buflen);

        /* Is this an HTTP GET command? (only check the first 5 chars, since
         there are other formats for GET, and we're keeping it very simple )*/
        if (buflen >= 5 && buf[0] == 'G' && buf[1] == 'E' && buf[2] == 'T'
                && buf[3] == ' ' && buf[4] == '/') {

          // disable videomode (autocapture) to allow streaming...
          bool s_moviemode = is_moviemode_on();
          set_moviemode(false);

          /* Send the HTTP header
             * subtract 1 from the size, since we dont send the \0 in the string
             * NETCONN_NOCOPY: our data is const static, so no need to copy it
             */
          netconn_write(conn, http_hdr, sizeof(http_hdr) - 1,
                    NETCONN_NOCOPY);


           //check if a stream is requested.
           if (buf[5] == 's') {
                //Send mjpeg stream header
                err = netconn_write(conn, http_stream_hdr, sizeof(http_stream_hdr) - 1,
                    NETCONN_NOCOPY);
                ESP_LOGD(TAG, "Stream started.");

                //Run while everyhting is ok and connection open.
                while(err == ERR_OK) {
                    ESP_LOGD(TAG, "Capture frame");

                    capture_request();
                    capture_wait_finish();

                    if (err != ESP_OK) {
                        ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
                    } else {
                        ESP_LOGD(TAG, "Done");
                        //stream an image..
                        if((s_pixel_format == CAMERA_PF_RGB565) || (s_pixel_format == CAMERA_PF_YUV422)) {
                            // write mime boundary start
                            err = netconn_write(conn, http_bitmap_hdr, sizeof(http_bitmap_hdr) - 1,
                                NETCONN_NOCOPY);
                            // write bitmap header
                            char *bmp = bmp_create_header565(camera_get_fb_width(), camera_get_fb_height());
                            err = netconn_write(conn, bmp, sizeof(bitmap565), NETCONN_NOCOPY);
                            free(bmp);
                            // convert framebuffer on the fly...
                            // only rgb and yuv...
                            uint8_t s_line[320*2];
                            uint32_t *fbl;
                            for (int i = 0; i < 240; i++) {
                              fbl = &currFbPtr[(i*320)/2];  //(i*(320*2)/4); // 4 bytes for each 2 pixel / 2 byte read..
                              convert_fb32bit_line_to_bmp565(fbl, s_line,s_pixel_format);
                              err = netconn_write(conn, s_line, 320*2,
                                            NETCONN_COPY);
                            }
                        }
                        else { // stream jpeg
                            err = netconn_write(conn, http_jpg_hdr, sizeof(http_jpg_hdr) - 1,
                                NETCONN_NOCOPY);
                            if(err == ERR_OK)
                              err = netconn_write(conn, camera_get_fb(), camera_get_data_size(),
                                              NETCONN_COPY);
                        }
                        if(err == ERR_OK)
                        {
                            //Send boundary to next jpeg
                            err = netconn_write(conn, http_stream_boundary,
                                    sizeof(http_stream_boundary) -1, NETCONN_NOCOPY);
                        }
                        vTaskDelay(30 / portTICK_RATE_MS);
                    }
                }
                ESP_LOGD(TAG, "Stream ended.");
                ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));
            } else {
                if (s_pixel_format == CAMERA_PF_JPEG) {
                    netconn_write(conn, http_jpg_hdr, sizeof(http_jpg_hdr) - 1, NETCONN_NOCOPY);
                } else if (s_pixel_format == CAMERA_PF_GRAYSCALE) {
                    netconn_write(conn, http_pgm_hdr, sizeof(http_pgm_hdr) - 1, NETCONN_NOCOPY);
                    if (memcmp(&buf[5], "pgm", 3) == 0) {
                        char pgm_header[32];
                        snprintf(pgm_header, sizeof(pgm_header), "P5 %d %d %d\n", camera_get_fb_width(), camera_get_fb_height(), 255);
                        netconn_write(conn, pgm_header, strlen(pgm_header), NETCONN_COPY);
                    }
                    else {
                      char outstr[120];
                      get_image_mime_info_str(outstr);
                      netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);
                      //netconn_write(conn, http_yuv422_hdr, sizeof(http_yuv422_hdr) - 1, NETCONN_NOCOPY);
                    }
                } else
                 if (s_pixel_format == CAMERA_PF_RGB565) {
                    netconn_write(conn, http_bitmap_hdr, sizeof(http_bitmap_hdr) - 1, NETCONN_NOCOPY);
                    if (memcmp(&buf[5], "bmp", 3) == 0) {
                        char *bmp = bmp_create_header565(camera_get_fb_width(), camera_get_fb_height());
                        err = netconn_write(conn, bmp, sizeof(bitmap565), NETCONN_COPY);
                        free(bmp);
                    }
                    else {
                      char outstr[120];
                      get_image_mime_info_str(outstr);
                      netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);
                      //netconn_write(conn, http_yuv422_hdr, sizeof(http_yuv422_hdr) - 1, NETCONN_NOCOPY);
                    }
                } else if (s_pixel_format == CAMERA_PF_YUV422) {
                  if (memcmp(&buf[5], "bmp", 3) == 0) {
                      // send YUV converted to 565 2bpp for now...
                      netconn_write(conn, http_bitmap_hdr, sizeof(http_bitmap_hdr) - 1, NETCONN_NOCOPY);
                      char *bmp = bmp_create_header565(camera_get_fb_width(), camera_get_fb_height());
                      err = netconn_write(conn, bmp, sizeof(bitmap565), NETCONN_COPY);
                      free(bmp);
                  } else {
                    char outstr[120];
                    get_image_mime_info_str(outstr);
                    netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);
                    //netconn_write(conn, http_yuv422_hdr, sizeof(http_yuv422_hdr) - 1, NETCONN_NOCOPY);
                  }
                } else {
                  char outstr[120];
                  get_image_mime_info_str(outstr);
                  netconn_write(conn, outstr, sizeof(outstr) - 1, NETCONN_NOCOPY);
                }
                // handle non streaming images (http../get and http:../bmp )

                  ESP_LOGD(TAG, "Image requested.");
                  //ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));

                  bool s_moviemode = is_moviemode_on();
                  set_moviemode(false);
                  capture_request();
                  capture_wait_finish();
                  set_moviemode(s_moviemode);

                  //ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));

                  if (err != ESP_OK) {
                      ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
                  } else {
                      ESP_LOGD(TAG, "Done");
                      //ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));
                      //Send jpeg
                      if ((s_pixel_format == CAMERA_PF_RGB565) || (s_pixel_format == CAMERA_PF_YUV422)) {
                        ESP_LOGD(TAG, "Converting framebuffer to RGB565 requested, sending...");
                        uint8_t s_line[320*2];
                        uint32_t *fbl;
                        for (int i = 0; i < 240; i++) {
                          fbl = &currFbPtr[(i*320)/2];  //(i*(320*2)/4); // 4 bytes for each 2 pixel / 2 byte read..
                          convert_fb32bit_line_to_bmp565(fbl, s_line,s_pixel_format);
                          err = netconn_write(conn, s_line, 320*2,
                                        NETCONN_COPY);
                        }
                    //    ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));

                      } else
                        err = netconn_write(conn, camera_get_fb(), camera_get_data_size(),
                          NETCONN_NOCOPY);
                  } // handle .bmp and std gets...

            }
        // end GET request:
        set_moviemode(s_moviemode);
        }
    }
    /* Close the connection (server closes in HTTP) */
    netconn_close(conn);
    /* Delete the buffer (netconn_recv gives us ownership,
     so we have to make sure to deallocate the buffer) */
    netbuf_delete(inbuf);
}

static void http_server(void *pvParameters)
{
    struct netconn *conn, *newconn;
    err_t err;
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 80);
    netconn_listen(conn);
    do {
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK) {
            http_server_netconn_serve(newconn);
            netconn_delete(newconn);
        }
    } while (err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
}

// I (11781) ESPILICAM: Free heap: 214720
// I (11781) ESPILICAM: Free (largest free blocks) 8bit-capable memory : 113804K, 32-bit capable memory 113804K
// I (11781) ESPILICAM: Free (min free size) 8bit-capable memory : 212956K, 32-bit capable memory 272940K
static void heap_mem_log()
{
    size_t free8start, free32start, free8, free32;

    #ifdef ESPIDFV21RC
        free8=xPortGetFreeHeapSizeCaps(MALLOC_CAP_8BIT);
        free32=xPortGetFreeHeapSizeCaps(MALLOC_CAP_32BIT);
        free8start=xPortGetMinimumEverFreeHeapSizeCaps(MALLOC_CAP_8BIT);
        free32start=xPortGetMinimumEverFreeHeapSizeCaps(MALLOC_CAP_32BIT);
    #else
        free32=heap_caps_get_largest_free_block(MALLOC_CAP_32BIT);
        free8=heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
        free8start=heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
        free32start=heap_caps_get_minimum_free_size(MALLOC_CAP_32BIT);
    #endif

    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());
    ESP_LOGI(TAG, "Free (largest free blocks) 8bit-capable memory : %d, 32-bit capable memory %d", free8, free32);
    ESP_LOGI(TAG, "Free (min free size) 8bit-capable memory : %d, 32-bit capable memory %d", free8start, free32start);
}

void app_main()
{
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("gpio", ESP_LOG_WARN);

    heap_mem_log();
    ESP_LOGI(TAG, "Allocating Frame Buffer 1 memory...");
    // 320*240*2 = 153600
    // heap_caps_malloc
    currFbPtr=heap_caps_malloc(320*240*2, MALLOC_CAP_32BIT);
    if (currFbPtr == NULL) {
        ESP_LOGE(TAG, "Not enough memory to allocate 1");
        return;
    }

    heap_mem_log();

    vTaskDelay(1000 / portTICK_RATE_MS);
    ESP_LOGI(TAG,"Starting nvs_flash_init");
    nvs_flash_init();

    vTaskDelay(3000 / portTICK_RATE_MS);

    ESP_LOGI(TAG,"Starting ESPILICAM");
    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());

    initialise_wifi();

    // VERY UNSTABLE without this delay after init'ing wifi...
    // however, much more stable with a new Power Supply
    vTaskDelay(5000 / portTICK_RATE_MS);

    ESP_LOGI(TAG, "Wifi Initialized...");
    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());



    esp_err_t ret;

    // camera init
    esp_err_t err = camera_probe(&config, &camera_model);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera probe failed with error 0x%x", err);
        return;
    }
    if (camera_model == CAMERA_OV7725) {
        ESP_LOGI(TAG, "Detected OV7725 camera, using grayscale bitmap format");
        s_pixel_format = CAMERA_PIXEL_FORMAT;
        config.frame_size = CAMERA_FRAME_SIZE;
    } else if (camera_model == CAMERA_OV7670) {
        ESP_LOGI(TAG, "Detected OV7670 camera");
        s_pixel_format = CAMERA_PIXEL_FORMAT;
        config.frame_size = CAMERA_FRAME_SIZE;
    } else if (camera_model == CAMERA_OV2640) {
        ESP_LOGI(TAG, "Detected OV2640 camera, using JPEG format");
        s_pixel_format = CAMERA_PF_JPEG;
        config.frame_size = CAMERA_FS_VGA;
        config.jpeg_quality = 15;
    } else {
        ESP_LOGE(TAG, "Camera not supported");
        return;
    }

    heap_mem_log();

    config.displayBuffer = currFbPtr;
    config.pixel_format = s_pixel_format;
    err = camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }

    vTaskDelay(2000 / portTICK_RATE_MS);

    dispSem=xSemaphoreCreateBinary();
    dispDoneSem=xSemaphoreCreateBinary();
    espilicam_event_group = xEventGroupCreate();

    xSemaphoreGive(dispDoneSem);

    captureSem=xSemaphoreCreateBinary();
    captureDoneSem=xSemaphoreCreateBinary();

    ESP_LOGD(TAG, "Starting OV7670 capture task...");
    xTaskCreatePinnedToCore(&captureTask, "captureTask", 2048, NULL, 5, NULL,1);

    vTaskDelay(1000 / portTICK_RATE_MS);

    ESP_LOGD(TAG, "Starting http_server task...");
    // keep an eye on stack... 5784 min with 8048 stck size last count..
    xTaskCreatePinnedToCore(&http_server, "http_server", 4096, NULL, 5, NULL,1);

    ESP_LOGI(TAG, "open http://" IPSTR "/bmp for single image/bitmap image", IP2STR(&s_ip_addr));
    ESP_LOGI(TAG, "open http://" IPSTR "/stream for multipart/x-mixed-replace stream of bitmaps", IP2STR(&s_ip_addr));
    ESP_LOGI(TAG, "open http://" IPSTR "/get for raw image as stored in framebuffer ", IP2STR(&s_ip_addr));

    ESP_LOGD(TAG, "Starting telnetd task...");
    // keep an eye on this - stack free was at 4620 at min with 8048
    xTaskCreatePinnedToCore(&telnetTask, "telnetTask", 5120, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "telnet to \"telnet " IPSTR "\" to access command console, type \"help\" for commands", IP2STR(&s_ip_addr));

    heap_mem_log();

    ESP_LOGI(TAG, "task stack: %d", uxTaskGetStackHighWaterMark(NULL));

    ESP_LOGI(TAG, "Camera demo ready.");

}
