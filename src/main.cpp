/**
 * Exemplo mínimo do addon CameraAddon (ESP32S3 + OV2640 + stream MJPEG :81).
 * initCamera() e connectWiFi() são independentes: pode usar só a câmera sem rede.
 *
 * WiFi: copie include/wifi_secrets.example.h -> include/wifi_secrets.h (gitignored).
 */

 #include "CameraAddon.h"

 #if __has_include("wifi_secrets.h")
 #include "wifi_secrets.h"
 #else
 #include "wifi_secrets.example.h"
 #endif
 
 void onImageCaptured(uint8_t *buf, size_t len);
 void initCamera();
 
 
 CameraAddon camera;
 
 void setup() {
   Serial.begin(115200);
   delay(1500);
   Serial.println();
   Serial.println("Boot: a iniciar...");
   Serial.flush();
 
   initCamera();
 }
 
 void loop() {
   camera.captureAndNotify();
   delay(5000);
 }
 
 void onImageCaptured(uint8_t *buf, size_t len) {
   (void)buf;
   Serial.printf("Imagem capturada: %u bytes\n", (unsigned)len);
   Serial.flush();
 }
 
 void initCamera(){
   camera.setBrightness(1);
   camera.setSaturation(0);
   camera.setVFlip(false);
   camera.setQuality(20);
   camera.setFrameSize(FRAMESIZE_SVGA);
 
   if (!camera.initCamera()) {
     Serial.println("Falha: initCamera.");
     Serial.flush();
     return;
   }
   Serial.println("Câmera OK.");
   Serial.flush();
 
   camera.setupLedFlash();
   //caso tenha configurado um callback para capturar a imagem, utilize:
   //a callback é chamada sempre que uma imagem é capturada, ou seja, você pode processar a imagem da sua maneira.
   camera.onCapture(onImageCaptured);
 
   if (camera.connectWiFi(WIFI_SSID, WIFI_PASSWORD)) {
     Serial.print("WiFi OK, IP: ");
     Serial.println(WiFi.localIP());
     if (camera.startStream()) {
       Serial.println("Stream: " + camera.getStreamURL());
     } else {
       Serial.println("Falha ao iniciar stream HTTP.");
     }
   } else {
     Serial.println("Sem WiFi: captura local continua a funcionar (sem stream no browser).");
   }
   Serial.flush();
 }