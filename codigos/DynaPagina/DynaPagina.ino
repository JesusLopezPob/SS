#include <WiFi.h>
#include <Dynamixel2Arduino.h>

// Configuración de la red WiFi
const char* ssid = "ESP32_Dynamixel";
const char* password = "12345678";

// Configuración del servidor web
WiFiServer server(80);

// Configuración de los pines Dynamixel
#define RX_PIN 16
#define TX_PIN 17
#define DXL_DIR_PIN 5

// Inicialización del puerto serial para Dynamixel
HardwareSerial& DXL_SERIAL = Serial2;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// Parámetros del Dynamixel

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;
const int DXL_Baud = 1000000;

/*
const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 1.0;
const int DXL_Baud = 1000000;
*/

// Rango máximo del Dynamixel
const int MAX_UNIT = 4095; // Rango de la unidad interna
const int MAX_ANGLE = 360; // Rango de ángulos

int32_t goal_position = 1500; // Posición inicial
bool use_angle = false;       // Bandera para indicar si se usa ángulo

// PID inicial
uint16_t position_p_gain = 120;
uint16_t position_i_gain = 15;
uint16_t position_d_gain = 1;

// Parámetros de velocidad y aceleración
uint16_t velocity = 100;   // Velocidad inicial
uint16_t acceleration = 50; // Aceleración inicial

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // Configuración del Dynamixel
  Serial.begin(115200);
  dxl.begin(DXL_Baud);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID);
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);

  // Aplicar valores iniciales del PID
  dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID, position_p_gain);
  dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID, position_i_gain);
  dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID, position_d_gain);

  // Configuración de WiFi como Access Point
  WiFi.softAP(ssid, password);
  Serial.print("Punto de acceso iniciado. Dirección IP: ");
  Serial.println(WiFi.softAPIP());

  // Iniciar servidor web
  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    String request = client.readStringUntil('\r');
    Serial.println(request);

    // Procesar comandos desde la página web
    if (request.indexOf("/set_position?value=") >= 0) {
      int startIndex = request.indexOf("=") + 1;
      int endIndex = request.indexOf("&", startIndex);
      String valueStr = request.substring(startIndex, endIndex);

      // Determinar si es un ángulo o una unidad interna
      use_angle = (request.indexOf("&type=angle") >= 0);
      if (use_angle) {
        int angle = valueStr.toInt();
        goal_position = map(angle, 0, MAX_ANGLE, 0, MAX_UNIT); // Convertir ángulo a unidad interna
      } else {
        goal_position = valueStr.toInt();
      }

      goal_position = constrain(goal_position, 0, MAX_UNIT); // Limitar a rango válido
      dxl.setGoalPosition(DXL_ID, goal_position);
      Serial.println("Posición ajustada a: " + String(goal_position) +
                     (use_angle ? " (por ángulo)" : " (por unidad interna)"));
    }

    if (request.indexOf("/set_pid?p=") >= 0) {
      int pStart = request.indexOf("p=") + 2;
      int pEnd = request.indexOf("&", pStart);
      position_p_gain = request.substring(pStart, pEnd).toInt();

      int iStart = request.indexOf("i=") + 2;
      int iEnd = request.indexOf("&", iStart);
      position_i_gain = request.substring(iStart, iEnd).toInt();

      int dStart = request.indexOf("d=") + 2;
      int dEnd = request.indexOf(" ", dStart);
      position_d_gain = request.substring(dStart, dEnd).toInt();

      // Aplicar nuevos valores del PID
      dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID, position_p_gain);
      dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID, position_i_gain);
      dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID, position_d_gain);

      Serial.println("PID ajustado a: P=" + String(position_p_gain) +
                     ", I=" + String(position_i_gain) +
                     ", D=" + String(position_d_gain));
    }

    // Procesar ajustes de velocidad y aceleración
    
    if (request.indexOf("/set_velocity?v=") >= 0) {
      int vStart = request.indexOf("v=") + 2;
      int vEnd = request.indexOf("&", vStart);
      velocity = request.substring(vStart, vEnd).toInt();
      dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, velocity);
      Serial.println("Velocidad ajustada a: " + String(velocity));
    }

    if (request.indexOf("/set_acceleration?a=") >= 0) {
      int aStart = request.indexOf("a=") + 2;
      int aEnd = request.indexOf("&", aStart);
      acceleration = request.substring(aStart, aEnd).toInt();
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, acceleration);
      Serial.println("Aceleración ajustada a: " + String(acceleration));
      
    }

    // Página web
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println();
    client.println("<!DOCTYPE html>");
    client.println("<html>");
    client.println("<head><title>Control Dynamixel</title></head>");
    client.println("<body>");
    client.println("<h1>Control del Dynamixel</h1>");

    // Formulario de control de posición
    client.println("<h2>Control de Posición</h2>");
    client.println("<form action='/set_position'>");
    client.println("<label for='value'>Valor (ángulo o unidad interna):</label>");
    client.println("<input type='number' id='value' name='value' min='0' max='4095'>");
    client.println("<input type='radio' id='type_angle' name='type' value='angle'>");
    client.println("<label for='type_angle'>Ángulo (0-360)</label>");
    client.println("<input type='radio' id='type_unit' name='type' value='unit' checked>");
    client.println("<label for='type_unit'>Unidad interna</label><br>");
    client.println("<button type='submit'>Mover</button>");
    client.println("</form>");

    // Formulario de control PID
    client.println("<h2>Ajustar PID</h2>");
    client.println("<form action='/set_pid'>");
    client.println("<label for='p'>P:</label>");
    client.println("<input type='number' id='p' name='p' min='0' max='32767' value='" + String(position_p_gain) + "'><br>");
    client.println("<label for='i'>I:</label>");
    client.println("<input type='number' id='i' name='i' min='0' max='32767' value='" + String(position_i_gain) + "'><br>");
    client.println("<label for='d'>D:</label>");
    client.println("<input type='number' id='d' name='d' min='0' max='32767' value='" + String(position_d_gain) + "'><br>");
    client.println("<button type='submit'>Actualizar PID</button>");
    client.println("</form>");

    // Formulario de control de velocidad
    client.println("<h2>Ajustar Velocidad</h2>");
    client.println("<form action='/set_velocity'>");
    client.println("<label for='v'>Velocidad:</label>");
    client.println("<input type='number' id='v' name='v' min='0' max='1023' value='" + String(velocity) + "'><br>");
    client.println("<button type='submit'>Actualizar Velocidad</button>");
    client.println("</form>");

    // Formulario de control de aceleración
    client.println("<h2>Ajustar Aceleración</h2>");
    client.println("<form action='/set_acceleration'>");
    client.println("<label for='a'>Aceleración:</label>");
    client.println("<input type='number' id='a' name='a' min='0' max='1023' value='" + String(acceleration) + "'><br>");
    client.println("<button type='submit'>Actualizar Aceleración</button>");
    client.println("</form>");

    client.println("</body>");
    client.println("</html>");
    client.stop();
  }
}
