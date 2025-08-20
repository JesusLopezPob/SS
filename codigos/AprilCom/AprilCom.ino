#define CAMERA_MODEL_AI_THINKER

#include "esp_camera.h"
#include "camera_pins.h"
#include "apriltag.h"
#include "tag36h11.h"
#include "common/image_u8.h"
#include "common/zarray.h"
#include "apriltag_pose.h"
#include "common/matd.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// Configuración Wi-Fi

const char *ssid = "mySSID";  // Nombre de la red Wi-Fi
const char *password = "myPass";


//const char *ssid = ".:PC Puma FI:.";  // Nombre de la red Wi-Fi

const char *hostIP = "192.168.79.7";  // Dirección IP del receptor (Master)
const int udpPort = 1234;            // Puerto UDP de la maestra

WiFiUDP udp;

// Parámetros de la cámara
#define TAG_SIZE 0.05
#define FX 370.1614
#define FY 379.2102
#define CX 160.5623
#define CY 128.9320

void setup() {
  Serial.begin(115200);

  // Inicializar PSRAM
  if (!psramInit()) {
    Serial.println("¡Error al inicializar PSRAM!");
  } else {
    Serial.println("PSRAM habilitada correctamente.");
  }

  // Conectar a Wi-Fi con configuración estática

  WiFi.begin(ssid, password);

  Serial.println("Conectando a Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConectado a Wi-Fi.");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());

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
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QVGA; // Resolución QVGA
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("¡Error al inicializar la cámara!");
    ESP.restart();
  }
  Serial.println("Cámara inicializada correctamente");
}

void loop() {
  // Captura de imagen
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Error al capturar imagen");
    return;
  }

  image_u8_t im = {
    .width = static_cast<int32_t>(fb->width),
    .height = static_cast<int32_t>(fb->height),
    .stride = static_cast<int32_t>(fb->width),
    .buf = fb->buf
  };

  apriltag_family_t *tf = tag36h11_create();
  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);

  td->quad_decimate = 2.0;
  td->nthreads = 1;

  zarray_t *detections = apriltag_detector_detect(td, &im);
  if (detections != nullptr) {
    int detections_count = zarray_size(detections);
    if (detections_count > 0) {
      for (int i = 0; i < detections_count; i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = TAG_SIZE;
        info.fx = FX;
        info.fy = FY;
        info.cx = CX;
        info.cy = CY;

        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);

        double posX = MATD_EL(pose.t, 0, 0);
        double posY = MATD_EL(pose.t, 1, 0);
        double posZ = MATD_EL(pose.t, 2, 0);

        // Construir mensaje con los datos de la etiqueta
        String message = "ID: " + String(det->id) +
                         ", X: " + String(posX, 4) +
                         ", Y: " + String(posY, 4) +
                         ", Z: " + String(posZ, 4);

        // Depuración
        Serial.println("Enviando mensaje: " + message);

        // Enviar el mensaje a través de UDP
        udp.beginPacket(hostIP, udpPort);
        udp.print(message.c_str());
        int success = udp.endPacket();

        if (success == 1) {
          Serial.println("Paquete enviado correctamente");
        } else {
          Serial.println("Error al enviar el paquete");
        }

        // Mostrar en el monitor serial
        Serial.println("Datos enviados: " + message);
      }
    } else {
      Serial.println("No se detectaron etiquetas.");
    }
    apriltag_detections_destroy(detections);
  }

  esp_camera_fb_return(fb);
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);

  delay(10);  // Ajusta la velocidad de captura
}
