#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <sys/time.h>

#include "board_config.h"
#include "esp_camera.h"
#include "esp_http_server.h"

/** Frame JPEG devolvido por capture(); chame releaseFrame() depois de usar. */
struct CameraFrame {
  uint8_t *buf;
  size_t len;
  struct timeval timestamp;
};

/**
 * Addon de câmera (OV2640/OV5640, …) com init opcional de WiFi e stream MJPEG na porta 81.
 * initCamera() e connectWiFi() são independentes; begin() apenas encadeia os dois.
 * Use uma única instância (handlers HTTP usam contexto estático interno).
 */
class CameraAddon {
 public:
  CameraAddon();
  ~CameraAddon();

  /**
   * Inicializa só o driver da câmera (pinos em board_config.h / camera_pins.h).
   * Não usa rede. Pode chamar setFrameSize/setQuality/etc. antes; o stream continua a ser com startStream().
   * @return false se já estiver inicializada ou se esp_camera_init falhar.
   */
  bool initCamera();

  /**
   * Liga WiFi em modo estação e espera até conectar ou esgotar o tempo.
   * Não depende da câmera (pode chamar antes ou depois de initCamera()).
   * @param ssid rede (obrigatório)
   * @param password palavra-passe (nullptr ou "" para rede aberta)
   * @param timeoutMs tempo máximo em milissegundos (omissão 60000)
   * @return false em timeout ou SSID inválido; em timeout faz disconnect WiFi (a câmera não é desligada).
   */
  bool connectWiFi(const char *ssid, const char *password, uint32_t timeoutMs = 60000);

  /**
   * Desliga a estação WiFi (útil para poupar energia ou modo só câmera após testes).
   */
  void disconnectWiFi();

  /** Indica se initCamera() já teve sucesso. */
  bool isCameraReady() const { return camera_ready_; }

  /** Indica se o WiFi está em WL_CONNECTED. */
  bool isWiFiConnected() const { return WiFi.status() == WL_CONNECTED; }

  /**
   * Equivalente a initCamera() seguido de connectWiFi() com o timeout por omissão.
   * Se o WiFi falhar, a câmera mantém-se inicializada (pode usar só captura ou voltar a connectWiFi).
   */
  bool begin(const char *ssid, const char *password);

  /**
   * Define o tamanho da imagem (enum framesize_t, ex.: FRAMESIZE_SVGA).
   * Requer pixformat JPEG; antes de initCamera() altera a configuração de init.
   * @return false se o sensor não estiver disponível após init e a operação falhar.
   */
  bool setFrameSize(framesize_t size);

  /**
   * Qualidade JPEG do sensor (tipicamente 0–63; menor = melhor qualidade / mais banda).
   */
  bool setQuality(int val);

  /** Brilho (faixa típica do driver -2 a 2; 0 = neutro). */
  bool setBrightness(int val);

  /** Contraste (faixa típica -2 a 2). */
  bool setContrast(int val);

  /** Saturação (faixa típica -2 a 2). */
  bool setSaturation(int val);

  /** Teto de ganho AGC (enum gainceiling_t). */
  bool setGainCeiling(gainceiling_t val);

  /** Barra de cores de teste (true/false). */
  bool setColorBar(bool val);

  /** Balanço de branco automático (true/false). */
  bool setAWB(bool val);

  /** Controle automático de ganho (true/false). */
  bool setAGC(bool val);

  /** Controle automático de exposição (true/false). */
  bool setAEC(bool val);

  /** Espelhamento horizontal (true/false). */
  bool setHMirror(bool val);

  /** Inversão vertical (true/false). */
  bool setVFlip(bool val);

  /** Ganho manual do AWB (true/false). */
  bool setAWBGain(bool val);

  /** Valor de ganho AGC (depende do sensor; tipicamente 0–30). */
  bool setAGCGain(int val);

  /** Valor de exposição AEC manual. */
  bool setAECValue(int val);

  /** Modo AEC2 (true/false). */
  bool setAEC2(bool val);

  /** Downsize EN (DCW) (true/false). */
  bool setDCW(bool val);

  /** Correção de píxeis pretos BPC (true/false). */
  bool setBPC(bool val);

  /** Correção de píxeis brancos WPC (true/false). */
  bool setWPC(bool val);

  /** Raw GMA (true/false). */
  bool setRawGMA(bool val);

  /** Correção de lente LENC (true/false). */
  bool setLenc(bool val);

  /** Efeito especial (0 = sem efeito; valores específicos do sensor). */
  bool setSpecialEffect(int val);

  /** Modo de WB (0 = auto, outros conforme sensor). */
  bool setWBMode(int val);

  /** Nível de AE (-2 a 2 típico). */
  bool setAELevel(int val);

  /**
   * Intensidade do LED de flash (0–255 duty típico).
   * Com LED_GPIO_NUM indefinido no board, retorna false e não altera nada.
   */
  bool setLedIntensity(int val);

  /**
   * Captura um frame JPEG.
   * @return frame com buf/len/timestamp; use releaseFrame() após consumir os dados.
   * Se já existir frame pendente sem releaseFrame(), retorna buf=nullptr e len=0.
   */
  CameraFrame capture();

  /** Devolve o framebuffer ao driver (obrigatório após capture()). */
  bool releaseFrame();

  /** Regista callback void(uint8_t* buf, size_t len) para captureAndNotify(). */
  void onCapture(void (*callback)(uint8_t *buf, size_t len));

  /** Captura JPEG e invoca o callback registado; liberta o frame em seguida. */
  bool captureAndNotify();

  /**
   * Dono único da câmera: inicia tarefa FreeRTOS que faz esp_camera_fb_get() a ritmo fixo (até ~15 Hz).
   * O /stream e copyLatestJpeg() deixam de capturar — só leem o último JPEG desta tarefa.
   * Chamar depois de initCamera(). Não depende do loop() do Arduino.
   */
  /**
   * @param stack_words stack da tarefa de captura (só esp_camera_fb_get + cópia; default 8192).
   * O sink LCD corre noutra tarefa com stack maior para não sobrecarregar cam_task.
   */
  bool startFrameService(uint32_t stack_words = 8192, UBaseType_t priority = 3);

  /** Para a tarefa de captura e liberta buffers internos (chamado automaticamente no destrutor). */
  void stopFrameService();

  /** true se startFrameService() está ativo. */
  bool isFrameServiceRunning() const { return frame_service_started_; }

  /**
   * FPS máximo de envio no MJPEG /stream (1–15). O handler não chama esp_camera_fb_get;
   * envia cópias do último frame; se a rede for lenta, o FPS efetivo fica abaixo deste teto.
   */
  void setStreamMaxFps(uint8_t fps);

  /**
   * Intervalo mínimo entre chamadas ao sink auxiliar (LCD, ML, etc.), em ms.
   * O sink corre na tarefa de captura, com cópia do JPEG (ponteiro só válido durante a chamada).
   */
  void setAuxJpegDeliverPeriodMs(uint32_t ms);

  /**
   * Sink opcional: recebe (jpeg, len, ctx) com cópia temporária; não guarde o ponteiro.
   * nullptr desativa.
   */
  void setAuxJpegSink(void (*sink)(const uint8_t *jpeg, size_t len, void *ctx), void *ctx);

  /**
   * Cópia thread-safe do último JPEG capturado pela tarefa interna.
   * @param timeout_ms tempo máximo de espera pelo mutex (0 = não bloqueia; use ~100–500 para bloquear).
   * @return false se sem frame ainda, dst_cap insuficiente, ou timeout.
   */
  bool copyLatestJpeg(uint8_t *dst, size_t dst_cap, size_t *out_len, struct timeval *timestamp_out,
                      uint32_t timeout_ms = 100);

  /** Número de sequência monotónico do último frame (útil para detetar frame novo). */
  uint32_t getLatestFrameSequence() const;

  /**
   * Inicia o servidor MJPEG em http://<IP>:81/stream (mesma lógica que stream_handler).
   */
  bool startStream();

  /** @deprecated Preferir setStreamMaxFps() com o serviço de frames ativo. */
  void setStreamMinPeriodMs(uint32_t ms) { stream_min_period_ms_ = ms; }

  /** URL do stream MJPEG após WiFi ligado. */
  String getStreamURL() const;

  /** Configura LEDC no pino LED (5000 Hz, 8 bits), como setupLedFlash() original. */
  void setupLedFlash();

 private:
  /** Monta camera_config_t a partir dos pinos do board e dos membros (framesize, quality, PSRAM). */
  void fillCameraHardwareConfig(camera_config_t *cfg);

#if defined(LED_GPIO_NUM)
  void enableLed(bool en);
#endif

  static esp_err_t streamHandlerThunk(httpd_req_t *req);
  esp_err_t streamHandler(httpd_req_t *req);
  esp_err_t streamMjpegFromSharedLatest(httpd_req_t *req);

  static void captureWorkerThunk(void *arg);
  void captureWorkerLoop();

  static void auxDeliverThunk(void *arg);
  void auxDeliverLoop();
  void drainAuxJpegQueue();

  enum SensorDirty : uint32_t {
    kDirtyContrast = 1u << 0,
    kDirtyGainCeiling = 1u << 1,
    kDirtyColorBar = 1u << 2,
    kDirtyAwb = 1u << 3,
    kDirtyAgc = 1u << 4,
    kDirtyAec = 1u << 5,
    kDirtyHmirror = 1u << 6,
    kDirtyAwbGain = 1u << 7,
    kDirtyAgcGain = 1u << 8,
    kDirtyAecValue = 1u << 9,
    kDirtyAec2 = 1u << 10,
    kDirtyDcw = 1u << 11,
    kDirtyBpc = 1u << 12,
    kDirtyWpc = 1u << 13,
    kDirtyRawGma = 1u << 14,
    kDirtyLenc = 1u << 15,
    kDirtySpecialEffect = 1u << 16,
    kDirtyWbMode = 1u << 17,
    kDirtyAeLevel = 1u << 18,
#if defined(LED_GPIO_NUM)
    kDirtyLed = 1u << 19,
#endif
  };

  bool applyDirtySensorBits(uint32_t mask);
  void markDirty(uint32_t bit);

  httpd_handle_t stream_httpd_;
  uint32_t stream_min_period_ms_;
  void (*capture_cb_)(uint8_t *buf, size_t len);

  bool frame_service_started_;
  bool run_capture_worker_;
  TaskHandle_t capture_task_handle_;
  SemaphoreHandle_t latest_jpeg_mutex_;
  uint8_t *latest_jpeg_buf_;
  uint8_t *stream_tx_buf_;
  size_t latest_jpeg_cap_;
  size_t latest_jpeg_len_;
  struct timeval latest_timestamp_;
  uint32_t latest_sequence_;
  int64_t mjpeg_next_deadline_us_;
  uint8_t stream_max_fps_;
  uint32_t aux_deliver_period_ms_;
  uint32_t last_aux_deliver_ms_;
  void (*aux_jpeg_sink_)(const uint8_t *jpeg, size_t len, void *ctx);
  void *aux_jpeg_sink_ctx_;

  struct AuxJpegMsg {
    uint8_t *data;
    size_t len;
  };

  QueueHandle_t aux_jpeg_queue_;
  bool run_aux_deliver_task_;
  TaskHandle_t aux_deliver_task_handle_;

  camera_fb_t *pending_fb_;
  uint8_t *pending_alloc_;
  bool quality_user_set_;
  bool camera_ready_;
  bool ra_filter_inited_;
  uint32_t sensor_dirty_;

  framesize_t framesize_;
  int quality_;
  int brightness_;
  int contrast_;
  int saturation_;
  gainceiling_t gainceiling_;
  int colorbar_;
  int awb_;
  int agc_;
  int aec_;
  int hmirror_;
  int vflip_;
  int awb_gain_;
  int agc_gain_;
  int aec_value_;
  int aec2_;
  int dcw_;
  int bpc_;
  int wpc_;
  int raw_gma_;
  int lenc_;
  int special_effect_;
  int wb_mode_;
  int ae_level_;

#if defined(LED_GPIO_NUM)
  int led_duty_;
  bool is_streaming_;
#endif

  typedef struct {
    size_t size;
    size_t index;
    size_t count;
    int sum;
    int *values;
  } RaFilter;

  RaFilter ra_filter_;

  static int raFilterRun(RaFilter *filter, int value);
  static RaFilter *raFilterInit(RaFilter *filter, size_t sample_size);

  static CameraAddon *s_instance_;
};
