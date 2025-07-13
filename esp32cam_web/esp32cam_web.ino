// Archivo: esp32cam_webserver.ino
#include "esp_camera.h"
#include <WiFi.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <ESPmDNS.h>


//http://esp32cam.local
// ================== CONFIGURACIÓN WI-FI ==================
//const char* ssid = "PIC16F628A de natan";
//const char* password = "IPHONERICKPROMAX";
const char* ssid = "MovistarFibra-2023";
const char* password = "30882709";
// ================== CONFIGURACIÓN PINES PARA AI-THINKER ==================
//--
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27

#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM    5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

// 4 for flash led or 33 for normal led
#define LED_GPIO_NUM   4


WebServer server(80);

// ================== FUNCIONES DE SERVIDOR ==================
void handleRoot() {
  File file = SPIFFS.open("/index.html", "r");
  if (!file) {
    server.send(500, "text/plain", "Error al abrir index.html");
    return;
  }
  server.streamFile(file, "text/html");
  file.close();
}
void handleStream() {
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  client.print(response);

  while (client.connected()) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) continue;

    client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.print("\r\n");

    esp_camera_fb_return(fb);
    delay(100);
  }
}

void handleCommand() {
  if (!server.hasArg("var") || !server.hasArg("val")) {
    server.send(400, "text/plain", "Faltan argumentos");
    return;
  }

  String var = server.arg("var");
  int val = server.arg("val").toInt();

  sensor_t * s = esp_camera_sensor_get();
  int res = 0;

  if (var == "framesize") {
    if (s->pixformat == PIXFORMAT_JPEG) {
      res = s->set_framesize(s, (framesize_t)val);
    }
  } else if (var == "quality") {
    res = s->set_quality(s, val);
  } else if (var == "contrast") {
    res = s->set_contrast(s, val);
  } else if (var == "brightness") {
    res = s->set_brightness(s, val);
  } else if (var == "saturation") {
    res = s->set_saturation(s, val);
  } else if (var == "gainceiling") {
    res = s->set_gainceiling(s, (gainceiling_t)val);
  } else if (var == "colorbar") {
    res = s->set_colorbar(s, val);
  } else if (var == "awb") {
    res = s->set_whitebal(s, val);
  } else if (var == "agc") {
    res = s->set_gain_ctrl(s, val);
  } else if (var == "aec") {
    res = s->set_exposure_ctrl(s, val);
  } else if (var == "hmirror") {
    res = s->set_hmirror(s, val);
  } else if (var == "vflip") {
    res = s->set_vflip(s, val);
  } else if (var == "awb_gain") {
    res = s->set_awb_gain(s, val);
  } else if (var == "agc_gain") {
    res = s->set_agc_gain(s, val);
  } else if (var == "aec_value") {
    res = s->set_aec_value(s, val);
  } else if (var == "aec2") {
    res = s->set_aec2(s, val);
  } else if (var == "dcw") {
    res = s->set_dcw(s, val);
  } else if (var == "bpc") {
    res = s->set_bpc(s, val);
  } else if (var == "wpc") {
    res = s->set_wpc(s, val);
  } else if (var == "raw_gma") {
    res = s->set_raw_gma(s, val);
  } else if (var == "lenc") {
    res = s->set_lenc(s, val);
  } else if (var == "special_effect") {
    res = s->set_special_effect(s, val);
  } else if (var == "wb_mode") {
    res = s->set_wb_mode(s, val);
  } else if (var == "ae_level") {
    res = s->set_ae_level(s, val);
  }else if (var == "led_intensity") {
    digitalWrite(LED_GPIO_NUM, val);
  }
  else {
    server.send(400, "text/plain", "Comando no válido");
    return;
  }

  if (res == 0) server.send(200, "text/plain", "OK");
  else server.send(500, "text/plain", "Error");
}

// ================== CONFIGURAR CÁMARA ==================
void startCamera() {
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x", err);
    while (true);
  }
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  Serial.println();
  pinMode(LED_GPIO_NUM, OUTPUT);

  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");

  unsigned long startAttemptTime = millis();
  const unsigned long timeout = 15000; // 15 segundos

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nError: no se pudo conectar a WiFi");
    return;
  }

  Serial.println("\nWiFi conectado");
  Serial.println("IP local: " + WiFi.localIP().toString());

  if (!SPIFFS.begin(true)) {
    Serial.println("Error montando SPIFFS");
    return;
  } else {
    Serial.println("SPIFFS iniciado correctamente");
  }

  // Inicializar la cámara ANTES de iniciar el servidor
  startCamera();

  // Handlers del servidor
  server.on("/", HTTP_GET, handleRoot);
  server.on("/stream", HTTP_GET, handleStream);
  server.on("/control", HTTP_GET, handleCommand);

  // mDNS (opcional)
  if (MDNS.begin("esp32cam")) {
    Serial.println("mDNS iniciado: http://esp32cam.local");
  } else {
    Serial.println("Error iniciando mDNS");
  }

  server.begin();
  Serial.println("Servidor HTTP iniciado");
}

void loop() {
  server.handleClient();
}
