# Addon-Freenove-CAM

Projeto **PlatformIO** para **ESP32-S3** com câmera (OV2640 / sensores compatíveis com `esp_camera`), centrado na biblioteca **`CameraAddon`**: inicialização da câmera, WiFi opcional, captura JPEG, callback de imagem e stream **MJPEG** na porta **81** (`/stream`).

> **Aviso legal:** o software é fornecido “como está”, sem garantias de qualquer tipo. Consulte o ficheiro [`LICENSE`](LICENSE) (MIT). O uso é da sua exclusiva responsabilidade (hardware, rede, privacidade de imagem, conformidade legal, etc.).

---

## Funcionalidades

| Área | Descrição |
|------|-----------|
| **Câmera** | `initCamera()` — `esp_camera_init` com pinos de `board_config.h` / `camera_pins.h`, JPEG, PSRAM quando disponível. |
| **WiFi** | `connectWiFi()` com timeout configurável; `disconnectWiFi()`; independente da câmera. |
| **Atalho** | `begin(ssid, pass)` = `initCamera()` + `connectWiFi()` (60 s de timeout no WiFi). |
| **Sensor** | Setters (`setBrightness`, `setFrameSize`, …) alinhados com o controlo HTTP típico dos exemplos Espressif; devolvem `bool`. |
| **Captura** | `capture()` → `CameraFrame` (`buf`, `len`, `timestamp`); obrigatório `releaseFrame()`. JPEG não nativo usa buffer temporário libertado em `releaseFrame()`. |
| **Callback** | `onCapture(fn)` + `captureAndNotify()` para processar JPEG sem gerir manualmente o return do framebuffer. |
| **Stream** | `startStream()` — servidor HTTP só com MJPEG, porta **81**, filtro de média móvel para FPS, LED durante stream se `LED_GPIO_NUM` existir. |
| **URL** | `getStreamURL()` → `http://<IP>:81/stream`. |
| **LED flash** | `setupLedFlash()` + `setLedIntensity()` quando o board define `LED_GPIO_NUM`. |

---

## Requisitos

- [PlatformIO](https://platformio.org/) (VS Code / Cursor com extensão, ou CLI).
- Placa **ESP32-S3** com câmera (ex.: perfil `freenove_esp32_s3_wroom` em [`platformio.ini`](platformio.ini)).
- **PSRAM** recomendada para resoluções e JPEG mais exigentes (ver comentários em `lib/CameraAddon/board_config.h`).

---

## Estrutura do repositório

```
├── platformio.ini          # Ambiente de build, monitor série, flags USB CDC
├── LICENSE                 # Licença MIT
├── README.md               # Este ficheiro
├── .gitignore              # Ignora build, credenciais locais, etc.
├── include/
│   ├── wifi_secrets.example.h   # Modelo de SSID/senha (versionado)
│   └── wifi_secrets.h           # Credenciais reais (NÃO versionado — ver .gitignore)
├── src/
│   └── main.cpp            # Firmware de exemplo usando CameraAddon
├── lib/CameraAddon/
│   ├── CameraAddon.h / CameraAddon.cpp   # Biblioteca principal
│   ├── board_config.h      # Modelo de câmera (descomente o seu board)
│   ├── camera_pins.h       # Pinos por modelo de placa
│   └── camera_index.h      # Recursos do exemplo web (se usados noutro alvo)
└── examples/CameraAddon_Example/
    └── example.ino         # Exemplo espelhado (referência)
```

---

## Configuração rápida

### 1. Modelo de placa / câmera

Edite **`lib/CameraAddon/board_config.h`**: mantenha **um** `#define CAMERA_MODEL_...` activo (por omissão `CAMERA_MODEL_ESP32S3_EYE`). Os pinos vêm de **`lib/CameraAddon/camera_pins.h`**.

### 2. Credenciais WiFi (sem ir para o GitHub)

1. Copie o modelo:  
   `include/wifi_secrets.example.h` → `include/wifi_secrets.h`
2. Edite `WIFI_SSID` e `WIFI_PASSWORD` em **`include/wifi_secrets.h`**.

O ficheiro **`include/wifi_secrets.h`** está listado no **`.gitignore`**, por isso não deve ser enviado em `git push`. O `src/main.cpp` usa `__has_include("wifi_secrets.h")`: se o ficheiro não existir (clone limpo), compila com o `.example.h` (credenciais fictícias — o WiFi não ligará até criar `wifi_secrets.h`).

### 3. Monitor série (ESP32-S3)

Em **`platformio.ini`**, `ARDUINO_USB_CDC_ON_BOOT` está em **0** por omissão (UART via conversor USB–serial: ROM + sketch na mesma porta COM). Se a sua placa usar **só USB nativo** do S3, comente `CDC=0` e active `CDC=1`, e abra a porta **USB JTAG/serial** no monitor.

- Velocidade típica: **115200** baud.

---

## Fluxo típico no código

1. **Opcional:** `setFrameSize`, `setQuality`, `setBrightness`, … **antes** de `initCamera()` (valores são aplicados no init ou em modo “dirty” até à primeira inicialização).
2. **`initCamera()`** — só driver da câmera; funciona **sem** WiFi.
3. **`setupLedFlash()`** — se o board tiver LED de flash definido.
4. **`onCapture(callback)`** — se quiser `captureAndNotify()` no `loop`.
5. **`connectWiFi(ssid, password)`** — só se precisar de rede (ex.: `getStreamURL`, browser).
6. **`startStream()`** — MJPEG em `:81/stream` (não exige WiFi no código, mas o URL só faz sentido com IP atribuído).
7. **`loop`:** `capture()` / `releaseFrame()` **ou** `captureAndNotify()`.

**Uma instância** de `CameraAddon`: o handler HTTP do stream usa contexto estático interno.

---

## Compilar e gravar

```bash
pio run -t upload
pio device monitor -b 115200
```

(ou os botões equivalentes na IDE.)

---

## Créditos e código de terceiros

- A lógica de **stream MJPEG**, **LED** e **init** de câmera inspira-se nos exemplos oficiais **Espressif** / ecossistema **esp32-camera** (frequentemente licenciados sob **Apache License 2.0**). Consulte as licenças dos componentes que a PlatformIO/Espressif puxa para o build (`esp_camera`, `esp_http_server`, Arduino-ESP32, etc.).
- O ficheiro **`LICENSE`** na raiz aplica-se ao **código deste repositório** tal como o contribuintes o licenciam sob **MIT**. Não substitui nem remove obrigações de licenças de dependências externas.

---

## Licença

Este projeto é licenciado sob a **MIT License** — ver [`LICENSE`](LICENSE).
