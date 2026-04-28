#include "CameraAddon.h"

#include "esp_timer.h"
#include "img_converters.h"
#include "sdkconfig.h"
#include "esp32-hal-ledc.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#endif

#include <cstring>

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *kStreamContentType = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *kStreamBoundary = "\r\n--" PART_BOUNDARY "\r\n";
static const char *kStreamPart = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

#if defined(LED_GPIO_NUM)
#define CONFIG_LED_MAX_INTENSITY 255
#endif

CameraAddon *CameraAddon::s_instance_ = nullptr;

CameraAddon::CameraAddon()
    : stream_httpd_(nullptr),
      capture_cb_(nullptr),
      pending_fb_(nullptr),
      pending_alloc_(nullptr),
      quality_user_set_(false),
      camera_ready_(false),
      ra_filter_inited_(false),
      sensor_dirty_(0),
      framesize_(FRAMESIZE_SVGA),
      quality_(12),
      brightness_(1),
      contrast_(0),
      saturation_(0),
      gainceiling_(static_cast<gainceiling_t>(0)),
      colorbar_(0),
      awb_(1),
      agc_(1),
      aec_(1),
      hmirror_(0),
      vflip_(0),
      awb_gain_(0),
      agc_gain_(0),
      aec_value_(300),
      aec2_(0),
      dcw_(1),
      bpc_(0),
      wpc_(1),
      raw_gma_(1),
      lenc_(0),
      special_effect_(0),
      wb_mode_(0),
      ae_level_(0),
#if defined(LED_GPIO_NUM)
      led_duty_(0),
      is_streaming_(false),
#endif
      ra_filter_{} {
  memset(&ra_filter_, 0, sizeof(ra_filter_));
}

CameraAddon::~CameraAddon() {
  if (stream_httpd_) {
    httpd_stop(stream_httpd_);
    stream_httpd_ = nullptr;
  }
  if (s_instance_ == this) {
    s_instance_ = nullptr;
  }
  releaseFrame();
  if (pending_alloc_) {
    free(pending_alloc_);
    pending_alloc_ = nullptr;
  }
  if (camera_ready_) {
    esp_camera_deinit();
    camera_ready_ = false;
  }
  if (ra_filter_inited_ && ra_filter_.values) {
    free(ra_filter_.values);
    ra_filter_.values = nullptr;
    ra_filter_inited_ = false;
  }
}

void CameraAddon::markDirty(uint32_t bit) {
  sensor_dirty_ |= bit;
}

bool CameraAddon::applyDirtySensorBits(uint32_t mask) {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    return false;
  }
  bool ok = true;
  if (mask & kDirtyContrast) {
    ok = ok && (s->set_contrast(s, contrast_) >= 0);
  }
  if (mask & kDirtyGainCeiling) {
    ok = ok && (s->set_gainceiling(s, gainceiling_) >= 0);
  }
  if (mask & kDirtyColorBar) {
    ok = ok && (s->set_colorbar(s, colorbar_) >= 0);
  }
  if (mask & kDirtyAwb) {
    ok = ok && (s->set_whitebal(s, awb_) >= 0);
  }
  if (mask & kDirtyAgc) {
    ok = ok && (s->set_gain_ctrl(s, agc_) >= 0);
  }
  if (mask & kDirtyAec) {
    ok = ok && (s->set_exposure_ctrl(s, aec_) >= 0);
  }
  if (mask & kDirtyHmirror) {
    ok = ok && (s->set_hmirror(s, hmirror_) >= 0);
  }
  if (mask & kDirtyAwbGain) {
    ok = ok && (s->set_awb_gain(s, awb_gain_) >= 0);
  }
  if (mask & kDirtyAgcGain) {
    ok = ok && (s->set_agc_gain(s, agc_gain_) >= 0);
  }
  if (mask & kDirtyAecValue) {
    ok = ok && (s->set_aec_value(s, aec_value_) >= 0);
  }
  if (mask & kDirtyAec2) {
    ok = ok && (s->set_aec2(s, aec2_) >= 0);
  }
  if (mask & kDirtyDcw) {
    ok = ok && (s->set_dcw(s, dcw_) >= 0);
  }
  if (mask & kDirtyBpc) {
    ok = ok && (s->set_bpc(s, bpc_) >= 0);
  }
  if (mask & kDirtyWpc) {
    ok = ok && (s->set_wpc(s, wpc_) >= 0);
  }
  if (mask & kDirtyRawGma) {
    ok = ok && (s->set_raw_gma(s, raw_gma_) >= 0);
  }
  if (mask & kDirtyLenc) {
    ok = ok && (s->set_lenc(s, lenc_) >= 0);
  }
  if (mask & kDirtySpecialEffect) {
    ok = ok && (s->set_special_effect(s, special_effect_) >= 0);
  }
  if (mask & kDirtyWbMode) {
    ok = ok && (s->set_wb_mode(s, wb_mode_) >= 0);
  }
  if (mask & kDirtyAeLevel) {
    ok = ok && (s->set_ae_level(s, ae_level_) >= 0);
  }
#if defined(LED_GPIO_NUM)
  if (mask & kDirtyLed) {
    if (is_streaming_) {
      enableLed(true);
    }
  }
#endif
  return ok;
}

void CameraAddon::fillCameraHardwareConfig(camera_config_t *config) {
  memset(config, 0, sizeof(*config));
  config->ledc_channel = LEDC_CHANNEL_0;
  config->ledc_timer = LEDC_TIMER_0;
  config->pin_d0 = Y2_GPIO_NUM;
  config->pin_d1 = Y3_GPIO_NUM;
  config->pin_d2 = Y4_GPIO_NUM;
  config->pin_d3 = Y5_GPIO_NUM;
  config->pin_d4 = Y6_GPIO_NUM;
  config->pin_d5 = Y7_GPIO_NUM;
  config->pin_d6 = Y8_GPIO_NUM;
  config->pin_d7 = Y9_GPIO_NUM;
  config->pin_xclk = XCLK_GPIO_NUM;
  config->pin_pclk = PCLK_GPIO_NUM;
  config->pin_vsync = VSYNC_GPIO_NUM;
  config->pin_href = HREF_GPIO_NUM;
  config->pin_sccb_sda = SIOD_GPIO_NUM;
  config->pin_sccb_scl = SIOC_GPIO_NUM;
  config->pin_pwdn = PWDN_GPIO_NUM;
  config->pin_reset = RESET_GPIO_NUM;
  config->xclk_freq_hz = 10000000;
  config->frame_size = framesize_;
  config->pixel_format = PIXFORMAT_JPEG;
  config->grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config->fb_location = CAMERA_FB_IN_PSRAM;
  {
    int jq = quality_user_set_ ? quality_ : 12;
    if (psramFound() && !quality_user_set_) {
      jq = 10;
    }
    config->jpeg_quality = jq;
    quality_ = jq;
  }
  config->fb_count = 2;

  if (psramFound()) {
    config->fb_count = 2;
    config->grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config->fb_count = 1;
    config->fb_location = CAMERA_FB_IN_DRAM;
  }
}

bool CameraAddon::initCamera() {
  if (camera_ready_) {
    return false;
  }

  camera_config_t config{};
  fillCameraHardwareConfig(&config);

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    log_e("Camera init failed 0x%x", err);
    return false;
  }

  camera_ready_ = true;

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, vflip_);
    s->set_brightness(s, brightness_);
    s->set_saturation(s, saturation_);
    if (s->pixformat == PIXFORMAT_JPEG) {
      s->set_framesize(s, framesize_);
    }
    s->set_quality(s, quality_);
  }

  applyDirtySensorBits(sensor_dirty_);
  sensor_dirty_ = 0;
  return true;
}

bool CameraAddon::connectWiFi(const char *ssid, const char *password, uint32_t timeoutMs) {
  if (!ssid || !ssid[0]) {
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password ? password : "");
  WiFi.setSleep(false);

  const uint32_t wifi_start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifi_start > timeoutMs) {
      log_e("WiFi: timeout. Verifique SSID/senha e alcance do AP.");
      WiFi.disconnect(true);
      return false;
    }
    delay(500);
  }
  return true;
}

void CameraAddon::disconnectWiFi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

bool CameraAddon::begin(const char *ssid, const char *password) {
  if (!initCamera()) {
    return false;
  }
  return connectWiFi(ssid, password);
}

bool CameraAddon::setFrameSize(framesize_t size) {
  framesize_ = size;
  if (!camera_ready_) {
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  if (!s || s->pixformat != PIXFORMAT_JPEG) {
    return false;
  }
  return s->set_framesize(s, size) >= 0;
}

bool CameraAddon::setQuality(int val) {
  quality_ = val;
  quality_user_set_ = true;
  if (!camera_ready_) {
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_quality(s, val) >= 0);
}

bool CameraAddon::setBrightness(int val) {
  brightness_ = val;
  if (!camera_ready_) {
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_brightness(s, val) >= 0);
}

bool CameraAddon::setContrast(int val) {
  contrast_ = val;
  if (!camera_ready_) {
    markDirty(kDirtyContrast);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_contrast(s, val) >= 0);
}

bool CameraAddon::setSaturation(int val) {
  saturation_ = val;
  if (!camera_ready_) {
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_saturation(s, val) >= 0);
}

bool CameraAddon::setGainCeiling(gainceiling_t val) {
  gainceiling_ = val;
  if (!camera_ready_) {
    markDirty(kDirtyGainCeiling);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_gainceiling(s, val) >= 0);
}

bool CameraAddon::setColorBar(bool val) {
  colorbar_ = val ? 1 : 0;
  if (!camera_ready_) {
    markDirty(kDirtyColorBar);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_colorbar(s, colorbar_) >= 0);
}

bool CameraAddon::setAWB(bool val) {
  awb_ = val ? 1 : 0;
  if (!camera_ready_) {
    markDirty(kDirtyAwb);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_whitebal(s, awb_) >= 0);
}

bool CameraAddon::setAGC(bool val) {
  agc_ = val ? 1 : 0;
  if (!camera_ready_) {
    markDirty(kDirtyAgc);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_gain_ctrl(s, agc_) >= 0);
}

bool CameraAddon::setAEC(bool val) {
  aec_ = val ? 1 : 0;
  if (!camera_ready_) {
    markDirty(kDirtyAec);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_exposure_ctrl(s, aec_) >= 0);
}

bool CameraAddon::setHMirror(bool val) {
  hmirror_ = val ? 1 : 0;
  if (!camera_ready_) {
    markDirty(kDirtyHmirror);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_hmirror(s, hmirror_) >= 0);
}

bool CameraAddon::setVFlip(bool val) {
  vflip_ = val ? 1 : 0;
  if (!camera_ready_) {
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_vflip(s, vflip_) >= 0);
}

bool CameraAddon::setAWBGain(bool val) {
  awb_gain_ = val ? 1 : 0;
  if (!camera_ready_) {
    markDirty(kDirtyAwbGain);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_awb_gain(s, awb_gain_) >= 0);
}

bool CameraAddon::setAGCGain(int val) {
  agc_gain_ = val;
  if (!camera_ready_) {
    markDirty(kDirtyAgcGain);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_agc_gain(s, val) >= 0);
}

bool CameraAddon::setAECValue(int val) {
  aec_value_ = val;
  if (!camera_ready_) {
    markDirty(kDirtyAecValue);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_aec_value(s, val) >= 0);
}

bool CameraAddon::setAEC2(bool val) {
  aec2_ = val ? 1 : 0;
  if (!camera_ready_) {
    markDirty(kDirtyAec2);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_aec2(s, aec2_) >= 0);
}

bool CameraAddon::setDCW(bool val) {
  dcw_ = val ? 1 : 0;
  if (!camera_ready_) {
    markDirty(kDirtyDcw);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_dcw(s, dcw_) >= 0);
}

bool CameraAddon::setBPC(bool val) {
  bpc_ = val ? 1 : 0;
  if (!camera_ready_) {
    markDirty(kDirtyBpc);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_bpc(s, bpc_) >= 0);
}

bool CameraAddon::setWPC(bool val) {
  wpc_ = val ? 1 : 0;
  if (!camera_ready_) {
    markDirty(kDirtyWpc);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_wpc(s, wpc_) >= 0);
}

bool CameraAddon::setRawGMA(bool val) {
  raw_gma_ = val ? 1 : 0;
  if (!camera_ready_) {
    markDirty(kDirtyRawGma);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_raw_gma(s, raw_gma_) >= 0);
}

bool CameraAddon::setLenc(bool val) {
  lenc_ = val ? 1 : 0;
  if (!camera_ready_) {
    markDirty(kDirtyLenc);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_lenc(s, lenc_) >= 0);
}

bool CameraAddon::setSpecialEffect(int val) {
  special_effect_ = val;
  if (!camera_ready_) {
    markDirty(kDirtySpecialEffect);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_special_effect(s, val) >= 0);
}

bool CameraAddon::setWBMode(int val) {
  wb_mode_ = val;
  if (!camera_ready_) {
    markDirty(kDirtyWbMode);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_wb_mode(s, val) >= 0);
}

bool CameraAddon::setAELevel(int val) {
  ae_level_ = val;
  if (!camera_ready_) {
    markDirty(kDirtyAeLevel);
    return true;
  }
  sensor_t *s = esp_camera_sensor_get();
  return s && (s->set_ae_level(s, val) >= 0);
}

bool CameraAddon::setLedIntensity(int val) {
#if defined(LED_GPIO_NUM)
  if (val < 0 || val > CONFIG_LED_MAX_INTENSITY) {
    return false;
  }
  led_duty_ = val;
  if (!camera_ready_) {
    markDirty(kDirtyLed);
    return true;
  }
  if (is_streaming_) {
    enableLed(true);
  }
  return true;
#else
  (void)val;
  return false;
#endif
}

CameraFrame CameraAddon::capture() {
  CameraFrame out{};
  if (pending_fb_ || pending_alloc_) {
    return out;
  }

#if defined(LED_GPIO_NUM)
  enableLed(true);
  vTaskDelay(150 / portTICK_PERIOD_MS);
#endif

  camera_fb_t *fb = esp_camera_fb_get();

#if defined(LED_GPIO_NUM)
  enableLed(false);
#endif

  if (!fb) {
    log_e("Camera capture failed");
    return out;
  }

  if (fb->format != PIXFORMAT_JPEG) {
    uint8_t *jpg = nullptr;
    size_t jpg_len = 0;
    if (!frame2jpg(fb, quality_, &jpg, &jpg_len)) {
      esp_camera_fb_return(fb);
      return out;
    }
    esp_camera_fb_return(fb);
    pending_alloc_ = jpg;
    out.buf = jpg;
    out.len = jpg_len;
    struct timeval tv {};
    gettimeofday(&tv, nullptr);
    out.timestamp = tv;
    return out;
  }

  pending_fb_ = fb;
  out.buf = fb->buf;
  out.len = fb->len;
  out.timestamp = fb->timestamp;
  return out;
}

bool CameraAddon::releaseFrame() {
  if (pending_fb_) {
    esp_camera_fb_return(pending_fb_);
    pending_fb_ = nullptr;
    return true;
  }
  if (pending_alloc_) {
    free(pending_alloc_);
    pending_alloc_ = nullptr;
    return true;
  }
  return false;
}

void CameraAddon::onCapture(void (*callback)(uint8_t *buf, size_t len)) {
  capture_cb_ = callback;
}

bool CameraAddon::captureAndNotify() {
  if (!capture_cb_) {
    return false;
  }
  CameraFrame f = capture();
  if (!f.buf) {
    return false;
  }
  capture_cb_(f.buf, f.len);
  return releaseFrame();
}

String CameraAddon::getStreamURL() const {
  return String("http://") + WiFi.localIP().toString() + ":81/stream";
}

void CameraAddon::setupLedFlash() {
#if defined(LED_GPIO_NUM)
  ledcAttach(LED_GPIO_NUM, 5000, 8);
#else
  log_i("LED flash is disabled -> LED_GPIO_NUM undefined");
#endif
}

#if defined(LED_GPIO_NUM)
void CameraAddon::enableLed(bool en) {
  int duty = en ? led_duty_ : 0;
  if (en && is_streaming_ && (led_duty_ > CONFIG_LED_MAX_INTENSITY)) {
    duty = CONFIG_LED_MAX_INTENSITY;
  }
  ledcWrite(LED_GPIO_NUM, duty);
  log_i("Set LED intensity to %d", duty);
}
#endif

CameraAddon::RaFilter *CameraAddon::raFilterInit(RaFilter *filter, size_t sample_size) {
  memset(filter, 0, sizeof(RaFilter));
  filter->values = (int *)malloc(sample_size * sizeof(int));
  if (!filter->values) {
    return nullptr;
  }
  memset(filter->values, 0, sample_size * sizeof(int));
  filter->size = sample_size;
  return filter;
}

int CameraAddon::raFilterRun(RaFilter *filter, int value) {
  if (!filter->values) {
    return value;
  }
  filter->sum -= filter->values[filter->index];
  filter->values[filter->index] = value;
  filter->sum += filter->values[filter->index];
  filter->index++;
  filter->index = filter->index % filter->size;
  if (filter->count < filter->size) {
    filter->count++;
  }
  return filter->sum / (int)filter->count;
}

esp_err_t CameraAddon::streamHandlerThunk(httpd_req_t *req) {
  if (!s_instance_) {
    return ESP_FAIL;
  }
  return s_instance_->streamHandler(req);
}

esp_err_t CameraAddon::streamHandler(httpd_req_t *req) {
  camera_fb_t *fb = nullptr;
  struct timeval timestamp {};
  esp_err_t res = ESP_OK;
  size_t jpg_buf_len = 0;
  uint8_t *jpg_buf = nullptr;
  char part_buf[128];

  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, kStreamContentType);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "60");

#if defined(LED_GPIO_NUM)
  is_streaming_ = true;
  enableLed(true);
#endif

  while (true) {
    jpg_buf = nullptr;
    jpg_buf_len = 0;
    fb = esp_camera_fb_get();
    if (!fb) {
      log_e("Camera capture failed");
      res = ESP_FAIL;
    } else {
      timestamp.tv_sec = fb->timestamp.tv_sec;
      timestamp.tv_usec = fb->timestamp.tv_usec;
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &jpg_buf, &jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = nullptr;
        if (!jpeg_converted) {
          log_e("JPEG compression failed");
          res = ESP_FAIL;
        }
      } else {
        jpg_buf_len = fb->len;
        jpg_buf = fb->buf;
      }
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, kStreamBoundary, strlen(kStreamBoundary));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), kStreamPart, (unsigned)jpg_buf_len, (int)timestamp.tv_sec, (int)timestamp.tv_usec);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_buf_len);
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = nullptr;
      jpg_buf = nullptr;
    } else if (jpg_buf) {
      free(jpg_buf);
      jpg_buf = nullptr;
    }
    if (res != ESP_OK) {
      log_e("Send frame failed");
      break;
    }
    int64_t fr_end = esp_timer_get_time();

    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;

    frame_time /= 1000;
    uint32_t avg_frame_time = (uint32_t)raFilterRun(&ra_filter_, (int)frame_time);
    log_i(
      "MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps)", (uint32_t)(jpg_buf_len), (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time, avg_frame_time,
      1000.0 / avg_frame_time
    );
  }

#if defined(LED_GPIO_NUM)
  is_streaming_ = false;
  enableLed(false);
#endif

  return res;
}

bool CameraAddon::startStream() {
  if (!camera_ready_) {
    return false;
  }
  if (stream_httpd_) {
    return true;
  }

  s_instance_ = this;

  if (!ra_filter_inited_) {
    if (!raFilterInit(&ra_filter_, 20)) {
      s_instance_ = nullptr;
      return false;
    }
    ra_filter_inited_ = true;
  }

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;

  if (httpd_start(&stream_httpd_, &config) != ESP_OK) {
    log_e("Stream httpd start failed");
    stream_httpd_ = nullptr;
    if (s_instance_ == this) {
      s_instance_ = nullptr;
    }
    return false;
  }

  httpd_uri_t stream_uri{};
  stream_uri.uri = "/stream";
  stream_uri.method = HTTP_GET;
  stream_uri.handler = streamHandlerThunk;
  stream_uri.user_ctx = this;

  if (httpd_register_uri_handler(stream_httpd_, &stream_uri) != ESP_OK) {
    httpd_stop(stream_httpd_);
    stream_httpd_ = nullptr;
    if (s_instance_ == this) {
      s_instance_ = nullptr;
    }
    return false;
  }

  log_i("Stream server on port 81");
  return true;
}
