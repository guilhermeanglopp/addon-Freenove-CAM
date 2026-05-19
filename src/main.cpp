#include "iimages.h"

#include "CameraAddon.h"
#include "img_converters.h"

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#else
#include "wifi_secrets.example.h"
#endif

// Include for plugins of chip 0
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define VSPI FSPI
#endif
// Include external libraries and files
#include <SPI.h>
#include <Arduino_GFX_Library.h>
#include <displayfk.h>
#include <cstring>

    /* Project setup:
    * MCU: ESP32S3
    * Display: ST7796
    * Touch: FT6336U
    *
    * Câmera: dono único na biblioteca (startFrameService). O stream /stream lê o mesmo buffer;
    * o LCD atualiza via setAuxJpegSink a ~1 Hz (setAuxJpegDeliverPeriodMs), sem depender do loop().
    */
// Defines for font and files
#define FORMAT_SPIFFS_IF_FAILED false
const int DISPLAY_W = 480;
const int DISPLAY_H = 320;
const int DISP_FREQUENCY = 27000000;
const int TOUCH_MAP_X0 = 0;
const int TOUCH_MAP_Y0 = 0;
const int TOUCH_MAP_X1 = 320;
const int TOUCH_MAP_Y1 = 480;
const bool TOUCH_SWAP_XY = true;
const bool TOUCH_INVERT_X = false;
const bool TOUCH_INVERT_Y = true;
const int DISP_MOSI = 47;
const int DISP_MISO = -1;
const int DISP_SCLK = 48;
const int DISP_CS = 14;
const int DISP_DC = 21;
const int DISP_RST = 3;
 const int TOUCH_SCL = -1;
const int TOUCH_SDA = -1;
const int TOUCH_INT = -1;
const int TOUCH_RST = -1;
const uint8_t rotationScreen = 3; // This value can be changed depending of orientation of your screen
const bool isIPS = true; // Come display can use this as bigEndian flag

// Prototypes for each screen
void screen0();
void loadWidgets();
void initCamera();
void rotate_image(uint16_t* src, uint16_t* dst, int width, int height, int angle) ;

static void aux_jpeg_to_lcd(const uint8_t *jpeg, size_t len, void * /*ctx*/);

// Create global SPI object
#if defined(CONFIG_IDF_TARGET_ESP32S3)
SPIClass spi_shared(FSPI);
#else
SPIClass spi_shared(HSPI);
#endif
Arduino_DataBus *bus = nullptr;
Arduino_GFX *tft = nullptr;
DisplayFK myDisplay;
// Create global objects. Constructor is: xPos, yPos and indexScreen
Image iimages(110, 45, 0);
const uint8_t qtdImagem = 1;
Image *arrayImagem[qtdImagem] = {&iimages};

CameraAddon camera;

/** Pixels RGB565 em RAM: mesmo layout que iimagesPixels em iimages.h (240x240). */
static uint16_t live_pixels[iimagesW * iimagesH];
/** Máscara 1 bit/pixel, opaco em todo o bitmap (como iimagesMask). */
static uint8_t live_mask[(iimagesW * iimagesH) / 8];

void setup(){

    Serial.begin(115200);
    // Start SPI object for display
    spi_shared.begin(DISP_SCLK, DISP_MISO, DISP_MOSI);
    bus = new Arduino_HWSPI(DISP_DC, DISP_CS, DISP_SCLK, DISP_MOSI, DISP_MISO, &spi_shared);
    tft = new Arduino_ST7796(bus, DISP_RST, rotationScreen, isIPS, 320, 480);
    tft->begin(DISP_FREQUENCY);
    myDisplay.setDrawObject(tft); // Reference to object to draw on screen
    // Setup touch
    myDisplay.setTouchCorners(TOUCH_MAP_X0, TOUCH_MAP_X1, TOUCH_MAP_Y0, TOUCH_MAP_Y1);
    myDisplay.setInvertAxis(TOUCH_INVERT_X, TOUCH_INVERT_Y);
    myDisplay.setSwapAxis(TOUCH_SWAP_XY);
    //myDisplay.startTouchFT6336(DISPLAY_W, DISPLAY_H, rotationScreen, TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST);
    //myDisplay.enableTouchLog();
    loadWidgets(); // This function is used to setup with widget individualy
    myDisplay.loadScreen(screen0); // Use this line to change between screens
    myDisplay.createTask(false, 3); // Initialize the task to read touch and draw

    initCamera();
}

void loop(){
    /* A câmera e o stream são geridos pela biblioteca; o loop pode ficar vazio ou tratar outra lógica.
       Exemplo de frame esporádico noutra tarefa:
         uint8_t buf[8000]; size_t n;
         if (camera.copyLatestJpeg(buf, sizeof(buf), &n, nullptr, 50)) { ... }
    */
    delay(200);
}

void screen0(){

    tft->fillScreen(CFK_WHITE);
    WidgetBase::backgroundColor = CFK_WHITE;
    //This screen has a/an imagem
    myDisplay.drawWidgetsOnScreen(0);
}

// Configure each widgtes to be used
void loadWidgets(){


    std::memcpy(live_pixels, iimagesPixels, sizeof(live_pixels));
    std::memset(live_mask, 0xff, sizeof(live_mask));

    ImageFromPixelsConfig configImage0 = {
            .pixels = live_pixels,
            .maskAlpha = live_mask,
            .cb = nullptr,
            .angle = 0.0f,
            .width = static_cast<uint16_t>(iimagesW),
            .height = static_cast<uint16_t>(iimagesH),
            .backgroundColor = CFK_WHITE
        };
    iimages.setupFromPixels(configImage0);
    myDisplay.setImage(arrayImagem,qtdImagem);


}

static void aux_jpeg_to_lcd(const uint8_t *jpeg, size_t len, void * /*ctx*/) {
    if (!jpeg || len == 0) {
        return;
    }
    if (jpg2rgb565(jpeg, len, reinterpret_cast<uint8_t *>(live_pixels), JPG_SCALE_NONE)) {
        iimages.forceUpdate();
    }
}

void initCamera() {
    camera.setBrightness(1);
    camera.setSaturation(0);
    camera.setVFlip(false);
    camera.setHMirror(true);
    camera.setQuality(20);
    camera.setFrameSize(FRAMESIZE_240X240);

    if (!camera.initCamera()) {
        Serial.println("Câmera: falha em initCamera.");
        return;
    }
    Serial.println("Câmera OK.");

    camera.setLedIntensity(0);
    camera.setupLedFlash();

    /* Dono único: até ~15 capturas/s; stream no máximo 15 FPS; sink do LCD ~1/s. */
    camera.setStreamMaxFps(10);
    camera.setAuxJpegDeliverPeriodMs(2000);
    camera.setAuxJpegSink(aux_jpeg_to_lcd, nullptr);

    if (!camera.startFrameService()) {
        Serial.println("Falha: startFrameService (PSRAM/memória?).");
        return;
    }

    if (camera.connectWiFi(WIFI_SSID, WIFI_PASSWORD)) {
        Serial.print("WiFi OK, IP: ");
        Serial.println(WiFi.localIP());
        if (camera.startStream()) {
            Serial.println("Stream: " + camera.getStreamURL());
        } else {
            Serial.println("Falha ao iniciar stream HTTP.");
        }
    } else {
        Serial.println("Sem WiFi: stream indisponível; captura para LCD continua.");
    }
}

