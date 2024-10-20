#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// Pines de la cámara AI-Thinker
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

httpd_handle_t server = NULL;

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = nullptr;

  // Establecer el tipo de respuesta como un flujo de video
  httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");

  while (true) {
    fb = esp_camera_fb_get();  // Capturar una imagen
    if (!fb) {
      Serial.println("¡Error al capturar la imagen!");
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }

    // Enviar el encabezado del flujo
    httpd_resp_send_chunk(req, "--frame\r\n", strlen("--frame\r\n"));
    httpd_resp_send_chunk(req, "Content-Type: image/jpeg\r\n", strlen("Content-Type: image/jpeg\r\n"));
    
    // Enviar el tamaño de la imagen
    char len_str[32];
    sprintf(len_str, "Content-Length: %d\r\n\r\n", fb->len);
    httpd_resp_send_chunk(req, len_str, strlen(len_str));
    
    // Enviar el contenido de la imagen
    httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    
    // Enviar un nuevo línea para finalizar el chunk
    httpd_resp_send_chunk(req, "\r\n", 2);

    esp_camera_fb_return(fb);  // Liberar la imagen
    delay(10);  // Ajusta el tiempo de espera para el flujo (30 ms = aproximadamente 33 fps)
  }
  
  return ESP_OK; // Aunque este código nunca se ejecuta debido al bucle infinito
}

// Configuración del servidor de la cámara
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  httpd_uri_t stream_uri = {
    .uri      = "/stream",
    .method   = HTTP_GET,
    .handler  = stream_handler,
    .user_ctx = NULL
  };

  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_register_uri_handler(server, &stream_uri);
  }
}

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
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_JPEG;  // Para streaming en formato JPEG
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12; // Calidad del JPEG
  config.fb_count = 3;

  if (psramFound()) {
    config.jpeg_quality = 12;
    config.fb_count = 3;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("¡Error al inicializar la cámara! Código de error: 0x%x", err);
    return;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }

  WiFi.begin(ssid);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("¡WiFi conectado!");

  startCameraServer();

  Serial.print("¡Cámara lista! Conéctate a 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("/stream' para ver el stream.");
}

void loop() {
  delay(10);
}
