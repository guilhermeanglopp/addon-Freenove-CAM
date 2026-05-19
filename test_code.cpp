/**
 * O exemplo mínimo da câmera foi integrado em src/main.cpp.
 *
 * Lá encontrará:
 * - CameraAddon, initCamera(), WiFi e stream (como antes);
 * - buffers live_pixels / live_mask no formato de include/iimages.h (RGB565 240×240 + máscara);
 * - onImageCaptured: JPEG → RGB565 (jpg2rgb565) e iimages.forceUpdate() no mesmo sítio do widget Image.
 *
 * Este ficheiro não faz parte do firmware (não está em src/). Mantém-se só como nota.
 */
