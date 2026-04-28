/**
 * Exemplo: initCamera() + connectWiFi() independentes.
 * Coloque wifi_secrets.h na pasta include/ do projeto (ver wifi_secrets.example.h).
 */

#include "CameraAddon.h"

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#else
#include "wifi_secrets.example.h"
#endif

CameraAddon camera;

void onImageCaptured(uint8_t *buf, size_t len) {
  (void)buf;
  Serial.printf("Imagem capturada: %u bytes\n", (unsigned)len);
  Serial.flush();
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("Boot...");
  Serial.flush();

  camera.setBrightness(1);
  camera.setSaturation(0);
  camera.setVFlip(false);
  camera.setQuality(10);
  camera.setFrameSize(FRAMESIZE_SVGA);

  if (!camera.initCamera()) {
    Serial.println("Falha initCamera");
    return;
  }

  camera.setupLedFlash();
  camera.onCapture(onImageCaptured);

  if (camera.connectWiFi(WIFI_SSID, WIFI_PASSWORD)) {
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    if (camera.startStream()) {
      Serial.println(camera.getStreamURL());
    }
  } else {
    Serial.println("Sem WiFi — só captura local.");
  }
  Serial.flush();
}

void loop() {
  camera.captureAndNotify();
  delay(5000);
}
