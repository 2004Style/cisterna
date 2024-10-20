#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClient.h>

#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27

#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

const char* ssid = "ConvertSystems";  // Nombre de tu red WiFi
const int tcpPort = 81;  // Puerto para el servidor TCP

WiFiServer server(tcpPort);  // Crear un servidor TCP

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Configuración de la cámara
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_SVGA;// Tamaños disponibles:
  // FRAMESIZE_96X96, // 96x96
  // FRAMESIZE_QQVGA, // 160x120
  // FRAMESIZE_QVGA,  // 320x240
  // FRAMESIZE_CIF,   // 352x288
  // FRAMESIZE_VGA,   // 640x480
  // FRAMESIZE_SVGA,  // 800x600
  // FRAMESIZE_XGA,   // 1024x768
  // FRAMESIZE_SXGA,  // 1280x1024
  // FRAMESIZE_UXGA,  // 1600x1200
  // FRAMESIZE_1080P, // 1920x1080
  // FRAMESIZE_1440P, // 2560x1440
  // FRAMESIZE_2K,    // 2048x1536
  // FRAMESIZE_4K,    // 3840x2160
  config.pixel_format = PIXFORMAT_JPEG;  // Para streaming en formato JPEG
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 10; // Calidad del JPEG
  config.fb_count = 3;

  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 3;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("¡Error al inicializar la cámara! Código de error: 0x%x", err);
    return;
  }

  WiFi.begin(ssid);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("¡WiFi conectado!");

  server.begin();  // Iniciar el servidor TCP
  Serial.print("¡Cámara lista! Conéctate IP: ");
  Serial.print(WiFi.localIP());
  Serial.print(" Port: ");
  Serial.println(tcpPort);
}

void loop() {
  WiFiClient client = server.available();  // Esperar a que un cliente se conecte
  if (client) {
    Serial.println("Cliente conectado");
    while (client.connected()) {
      camera_fb_t *fb = esp_camera_fb_get();  // Capturar una imagen
      if (!fb) {
        Serial.println("¡Error al capturar la imagen!");
        break;
      }

      // Enviar el tamaño de la imagen (4 bytes)
      client.write((const uint8_t*)&fb->len, sizeof(fb->len)); // Enviar tamaño de la imagen
      // Enviar la imagen al cliente
      client.write((const uint8_t*)fb->buf, fb->len);  // Enviar la imagen
      esp_camera_fb_return(fb);  // Liberar la imagen

      delay(10);  // Ajusta el tiempo de espera para el flujo (30 ms = aproximadamente 33 fps)
    }
    client.stop();  // Desconectar al cliente
    Serial.println("Cliente desconectado");
  }
}
