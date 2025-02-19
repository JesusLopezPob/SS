#include <WiFi.h>
#include <Dynamixel2Arduino.h>

// =========================
// Configuración de WiFi
// =========================
const char* ssid = "ESP32_Dynamixel";
const char* password = "12345678";
WiFiServer server(80);

// =========================
// Pines y configuración común
// =========================
#define RX_PIN 16
#define TX_PIN 17
#define DXL_DIR_PIN 5

HardwareSerial& DXL_SERIAL = Serial2;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
//This namespace is required to use Control table item names
using namespace ControlTableItem;

const int DXL_Baud = 1000000;

// -------------------------
// Para servos que usan protocolo 2.0 (Servo 1 y 2)
// -------------------------
const uint8_t SERVO_IDS[] = {1, 2};
const int NUM_SERVOS = sizeof(SERVO_IDS);
const float PROTOCOL_VERSION_2 = 2.0;
const int MAX_UNIT_2 = 4095;
const int MAX_ANGLE_2 = 360;

// -------------------------
// Para el servo 3 (control individual, protocolo 1.0)
// -------------------------
const uint8_t SERVO3_ID = 3;
const float PROTOCOL_VERSION_1 = 1.0;
const int MAX_UNIT_1 = 1023;     // Rango interno del servo (AX-18)
const int MAX_ANGLE_1 = 300;     // Rango en grados (por ejemplo, 0-300°)

// Definición de direcciones y longitudes (protocolo 1.0)
#define TORQUE_ENABLE_ADDR      24
#define CW_ANGLE_LIMIT_ADDR     6
#define CCW_ANGLE_LIMIT_ADDR    8
#define GOAL_POSITION_ADDR      30
#define LED_ADDR                25
#define MOVING_SPEED_ADDR       32

#define TORQUE_ENABLE_ADDR_LEN  1
#define ANGLE_LIMIT_ADDR_LEN    2
#define GOAL_POSITION_ADDR_LEN  2
#define LED_ADDR_LEN            1
#define MOVING_SPEED_ADDR_LEN   2

// Valores de configuración para el servo 3 (ajusta según tu necesidad)
uint16_t servo3_CW_limit  = 300;   // Límite inferior
uint16_t servo3_CCW_limit = 450;   // Límite superior
uint16_t servo3_speed     = 50;    // Velocidad de movimiento

// Constantes para torque
const uint8_t TURN_ON  = 1;
const uint8_t TURN_OFF = 0;

// Tiempo de espera para las operaciones (ms)
#define TIMEOUT 1000

// =========================
// Función auxiliar para extraer parámetros de la URL
// =========================
String getValue(String data, String parameter) {
  int startIndex = data.indexOf(parameter + "=");
  if (startIndex == -1) return "";
  startIndex += parameter.length() + 1;
  int endIndex = data.indexOf("&", startIndex);
  if (endIndex == -1) {
    return data.substring(startIndex);
  } else {
    return data.substring(startIndex, endIndex);
  }
}

// =========================
// setup()
// =========================
void setup() {
  Serial.begin(115200);
  // Inicializa el puerto serial para Dynamixel
  DXL_SERIAL.begin(DXL_Baud, SERIAL_8N1, RX_PIN, TX_PIN);
  dxl.begin(DXL_Baud);

  // --- Configurar servos 1 y 2 (Protocolo 2.0) ---
  dxl.setPortProtocolVersion(PROTOCOL_VERSION_2);
  for (int i = 0; i < NUM_SERVOS; i++) {
    uint8_t id = SERVO_IDS[i];
    dxl.ping(id);
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_POSITION);
    dxl.torqueOn(id);
    Serial.println("Servo " + String(id) + " configurado en posición (Protocolo 2.0).");
  }

  // --- Configurar el servo 3 (Protocolo 1.0) ---
  dxl.setPortProtocolVersion(PROTOCOL_VERSION_1);
  if (dxl.ping(SERVO3_ID)) {
    Serial.println("Servo 3 encontrado (Protocolo 1.0).");
      // Apagar torque para configurar
    if (dxl.write(SERVO3_ID, TORQUE_ENABLE_ADDR, (uint8_t*)&TURN_OFF, TORQUE_ENABLE_ADDR_LEN, TIMEOUT)) {
      Serial.println("Servo 3: Torque apagado.");
    } else {
      Serial.println("Error: No se pudo apagar el torque del servo 3.");
    }
    // Configurar límites de ángulo
    if (dxl.write(SERVO3_ID, CW_ANGLE_LIMIT_ADDR, (uint8_t*)&servo3_CW_limit, ANGLE_LIMIT_ADDR_LEN, TIMEOUT) &&
        dxl.write(SERVO3_ID, CCW_ANGLE_LIMIT_ADDR, (uint8_t*)&servo3_CCW_limit, ANGLE_LIMIT_ADDR_LEN, TIMEOUT)) {
      Serial.println("Servo 3: Límites de ángulo configurados.");
    } else {
      Serial.println("Error: No se pudieron configurar los límites de ángulo del servo 3.");
    }
    // Configurar velocidad de movimiento
    if (dxl.write(SERVO3_ID, MOVING_SPEED_ADDR, (uint8_t*)&servo3_speed, MOVING_SPEED_ADDR_LEN, TIMEOUT)) {
      Serial.println("Servo 3: Velocidad configurada.");
    } else {
      Serial.println("Error: No se pudo configurar la velocidad del servo 3.");
    }
    // Activar torque
    if (dxl.write(SERVO3_ID, TORQUE_ENABLE_ADDR, (uint8_t*)&TURN_ON, TORQUE_ENABLE_ADDR_LEN, TIMEOUT)) {
      Serial.println("Servo 3: Torque activado.");
    } else {
      Serial.println("Error: No se pudo activar el torque del servo 3.");
    }
    
  } else {
    Serial.println("Error: No se encontró el servo 3.");
    while (true);  // Detiene el programa si no se encuentra el servo
  }

  // Regresar a protocolo 2.0 para servos 1 y 2
  dxl.setPortProtocolVersion(PROTOCOL_VERSION_2);

  // --- Inicializar el punto de acceso WiFi ---
  WiFi.softAP(ssid, password);
  Serial.print("Punto de acceso iniciado. IP: ");
  Serial.println(WiFi.softAPIP());
  server.begin();
}

// =========================
// loop()
// =========================
void loop() {
  WiFiClient client = server.available();
  if (client) {
    String request = client.readStringUntil('\r');
    Serial.println("Solicitud: " + request);

    // Procesa el control de posición (para los 3 servos)
    if (request.indexOf("/set_position") != -1) {
      String servoStr = getValue(request, "servo");
      String valueStr = getValue(request, "value");
      String typeStr  = getValue(request, "type");
      int servoID = servoStr.toInt();
      int posValue = valueStr.toInt();

      // Si es servo 3, se usa protocolo 1.0 y sus direcciones
      if (servoID == SERVO3_ID) {
        // Para servo 3 asumimos:
        // - Si se usa "angle", se mapea de 0 a MAX_ANGLE_1 (por ejemplo, 0-300°) al rango interno 0-MAX_UNIT_1.
        if (typeStr == "angle") {
          posValue = map(posValue, 0, MAX_ANGLE_1, 0, MAX_UNIT_1);
        }
        posValue = constrain(posValue, 0, MAX_UNIT_1);
        // Cambiar a protocolo 1.0 para enviar el comando
        dxl.setPortProtocolVersion(PROTOCOL_VERSION_1);
        uint16_t pos = posValue;
        if (dxl.write(SERVO3_ID, GOAL_POSITION_ADDR, (uint8_t*)&pos, GOAL_POSITION_ADDR_LEN, TIMEOUT)) {
          Serial.println("Servo 3 posicionado a: " + String(pos));
        } else {
          Serial.println("Error moviendo el servo 3.");
        }
        // Regresar a protocolo 2.0 para los demás servos
        dxl.setPortProtocolVersion(PROTOCOL_VERSION_2);
      }
      // Para servo 1 y 2 (protocolo 2.0)
      else {
        if (typeStr == "angle") {
          posValue = map(posValue, 0, MAX_ANGLE_2, 0, MAX_UNIT_2);
        }
        posValue = constrain(posValue, 0, MAX_UNIT_2);
        dxl.setGoalPosition(servoID, posValue);
        Serial.println("Servo " + String(servoID) + " posicionado a: " + String(posValue));
      }
    }

    // Procesa el ajuste de PID (solo para servos 1 y 2)
    if (request.indexOf("/set_pid") != -1) {
      String servoStr = getValue(request, "servo");
      int servoID = servoStr.toInt();
      if (servoID == SERVO3_ID) {
        Serial.println("El servo 3 solo se controla en posición.");
      } else {
        int pVal = getValue(request, "p").toInt();
        int iVal = getValue(request, "i").toInt();
        int dVal = getValue(request, "d").toInt();
        dxl.writeControlTableItem(POSITION_P_GAIN, servoID, pVal);
        dxl.writeControlTableItem(POSITION_I_GAIN, servoID, iVal);
        dxl.writeControlTableItem(POSITION_D_GAIN, servoID, dVal);
        Serial.println("Servo " + String(servoID) + " PID -> P:" + String(pVal) + " I:" + String(iVal) + " D:" + String(dVal));
      }
    }

    // Procesa el ajuste de velocidad (solo para servos 1 y 2)
    if (request.indexOf("/set_velocity") != -1) {
      String servoStr = getValue(request, "servo");
      int servoID = servoStr.toInt();
      if (servoID == SERVO3_ID) {
        Serial.println("El servo 3 solo se controla en posición.");
      } else {
        int vel = getValue(request, "v").toInt();
        dxl.writeControlTableItem(PROFILE_VELOCITY, servoID, vel);
        Serial.println("Servo " + String(servoID) + " velocidad -> " + String(vel));
      }
    }

    // Procesa el ajuste de aceleración (solo para servos 1 y 2)
    if (request.indexOf("/set_acceleration") != -1) {
      String servoStr = getValue(request, "servo");
      int servoID = servoStr.toInt();
      if (servoID == SERVO3_ID) {
        Serial.println("El servo 3 solo se controla en posición.");
      } else {
        int acc = getValue(request, "a").toInt();
        dxl.writeControlTableItem(PROFILE_ACCELERATION, servoID, acc);
        Serial.println("Servo " + String(servoID) + " aceleración -> " + String(acc));
      }
    }

    // --- Enviar página HTML ---
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println();
    client.println("<!DOCTYPE html>");
    client.println("<html lang=\"es\">");
    client.println("<head>");
    client.println("  <meta charset=\"UTF-8\">");
    client.println("  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">");
    client.println("  <title>Control de Servos</title>");
    client.println("  <style>");
    client.println("    .tab-container { display: flex; border-bottom: 2px solid #ccc; }");
    client.println("    .tab { padding: 10px 20px; cursor: pointer; border: 1px solid #ccc; border-bottom: none; background-color: #f1f1f1; }");
    client.println("    .tab:hover { background-color: #ddd; }");
    client.println("    .tab.active { background-color: white; font-weight: bold; }");
    client.println("    .tab-content { display: none; padding: 20px; border: 1px solid #ccc; }");
    client.println("    .tab-content.active { display: block; }");
    client.println("  </style>");
    client.println("</head>");
    client.println("<body>");
    client.println("  <header><h1>Control de Dynamixel</h1></header>");
    client.println("  <div class=\"tab-container\">");
    client.println("    <div class=\"tab active\" onclick=\"openTab(event, 'pestana1')\">Servo 1</div>");
    client.println("    <div class=\"tab\" onclick=\"openTab(event, 'pestana2')\">Servo 2</div>");
    client.println("    <div class=\"tab\" onclick=\"openTab(event, 'pestana3')\">Servo 3</div>");
    client.println("  </div>");

    // Pestaña Servo 1
    client.println("  <div id=\"pestana1\" class=\"tab-content active\">");
    client.println("    <h2>Control del Servo 1</h2>");
    client.println("    <h3>Control de Posición</h3>");
    client.println("    <form action=\"/set_position\" method=\"get\">");
    client.println("      <input type=\"hidden\" name=\"servo\" value=\"1\">");
    client.println("      <label>Posición (0-4095 o 0-360 si es 'angle'):</label><br>");
    client.println("      <input type=\"number\" name=\"value\" min=\"0\" max=\"4095\" required><br>");
    client.println("      <input type=\"radio\" name=\"type\" value=\"unit\" checked> Unidad interna<br>");
    client.println("      <input type=\"radio\" name=\"type\" value=\"angle\"> Ángulo (0-360)<br>");
    client.println("      <button type=\"submit\">Mover</button>");
    client.println("    </form>");
    client.println("    <h3>Ajustar PID</h3>");
    client.println("    <form action=\"/set_pid\" method=\"get\">");
    client.println("      <input type=\"hidden\" name=\"servo\" value=\"1\">");
    client.println("      <label>P:</label><input type=\"number\" name=\"p\" min=\"0\" max=\"32767\" required><br>");
    client.println("      <label>I:</label><input type=\"number\" name=\"i\" min=\"0\" max=\"32767\" required><br>");
    client.println("      <label>D:</label><input type=\"number\" name=\"d\" min=\"0\" max=\"32767\" required><br>");
    client.println("      <button type=\"submit\">Actualizar PID</button>");
    client.println("    </form>");
    client.println("    <h3>Ajustar Velocidad</h3>");
    client.println("    <form action=\"/set_velocity\" method=\"get\">");
    client.println("      <input type=\"hidden\" name=\"servo\" value=\"1\">");
    client.println("      <label>Velocidad:</label><input type=\"number\" name=\"v\" min=\"0\" max=\"1023\" required><br>");
    client.println("      <button type=\"submit\">Actualizar Velocidad</button>");
    client.println("    </form>");
    client.println("    <h3>Ajustar Aceleración</h3>");
    client.println("    <form action=\"/set_acceleration\" method=\"get\">");
    client.println("      <input type=\"hidden\" name=\"servo\" value=\"1\">");
    client.println("      <label>Aceleración:</label><input type=\"number\" name=\"a\" min=\"0\" max=\"1023\" required><br>");
    client.println("      <button type=\"submit\">Actualizar Aceleración</button>");
    client.println("    </form>");
    client.println("  </div>");

    // Pestaña Servo 2
    client.println("  <div id=\"pestana2\" class=\"tab-content\">");
    client.println("    <h2>Control del Servo 2</h2>");
    client.println("    <h3>Control de Posición</h3>");
    client.println("    <form action=\"/set_position\" method=\"get\">");
    client.println("      <input type=\"hidden\" name=\"servo\" value=\"2\">");
    client.println("      <label>Posición (0-4095 o 0-360 si es 'angle'):</label><br>");
    client.println("      <input type=\"number\" name=\"value\" min=\"0\" max=\"4095\" required><br>");
    client.println("      <input type=\"radio\" name=\"type\" value=\"unit\" checked> Unidad interna<br>");
    client.println("      <input type=\"radio\" name=\"type\" value=\"angle\"> Ángulo (0-360)<br>");
    client.println("      <button type=\"submit\">Mover</button>");
    client.println("    </form>");
    client.println("    <h3>Ajustar PID</h3>");
    client.println("    <form action=\"/set_pid\" method=\"get\">");
    client.println("      <input type=\"hidden\" name=\"servo\" value=\"2\">");
    client.println("      <label>P:</label><input type=\"number\" name=\"p\" min=\"0\" max=\"32767\" required><br>");
    client.println("      <label>I:</label><input type=\"number\" name=\"i\" min=\"0\" max=\"32767\" required><br>");
    client.println("      <label>D:</label><input type=\"number\" name=\"d\" min=\"0\" max=\"32767\" required><br>");
    client.println("      <button type=\"submit\">Actualizar PID</button>");
    client.println("    </form>");
    client.println("    <h3>Ajustar Velocidad</h3>");
    client.println("    <form action=\"/set_velocity\" method=\"get\">");
    client.println("      <input type=\"hidden\" name=\"servo\" value=\"2\">");
    client.println("      <label>Velocidad:</label><input type=\"number\" name=\"v\" min=\"0\" max=\"1023\" required><br>");
    client.println("      <button type=\"submit\">Actualizar Velocidad</button>");
    client.println("    </form>");
    client.println("    <h3>Ajustar Aceleración</h3>");
    client.println("    <form action=\"/set_acceleration\" method=\"get\">");
    client.println("      <input type=\"hidden\" name=\"servo\" value=\"2\">");
    client.println("      <label>Aceleración:</label><input type=\"number\" name=\"a\" min=\"0\" max=\"1023\" required><br>");
    client.println("      <button type=\"submit\">Actualizar Aceleración</button>");
    client.println("    </form>");
    client.println("  </div>");

    // Pestaña Servo 3 (solo posición)
    client.println("  <div id=\"pestana3\" class=\"tab-content\">");
    client.println("    <h2>Control del Servo 3 (Solo Posición)</h2>");
    client.println("    <h3>Control de Posición</h3>");
    client.println("    <form action=\"/set_position\" method=\"get\">");
    client.println("      <input type=\"hidden\" name=\"servo\" value=\"3\">");
    client.println("      <label>Posición (0-1023 o 0-300 si es 'angle'):</label><br>");
    client.println("      <input type=\"number\" name=\"value\" min=\"0\" max=\"1023\" required><br>");
    client.println("      <input type=\"radio\" name=\"type\" value=\"unit\" checked> Unidad interna<br>");
    client.println("      <input type=\"radio\" name=\"type\" value=\"angle\"> Ángulo (0-300)<br>");
    client.println("      <button type=\"submit\">Mover</button>");
    client.println("    </form>");
    client.println("  </div>");

    // JavaScript para el manejo de pestañas
    client.println("  <script>");
    client.println("    function openTab(evt, tabId) {");
    client.println("      var i, tabcontent, tablinks;");
    client.println("      tabcontent = document.getElementsByClassName('tab-content');");
    client.println("      for (i = 0; i < tabcontent.length; i++) {");
    client.println("        tabcontent[i].classList.remove('active');");
    client.println("      }");
    client.println("      tablinks = document.getElementsByClassName('tab');");
    client.println("      for (i = 0; i < tablinks.length; i++) {");
    client.println("        tablinks[i].classList.remove('active');");
    client.println("      }");
    client.println("      document.getElementById(tabId).classList.add('active');");
    client.println("      evt.currentTarget.classList.add('active');");
    client.println("    }");
    client.println("  </script>");
    client.println("</body>");
    client.println("</html>");

    client.stop();
  }
}
