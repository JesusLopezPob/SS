#include <WiFi.h>
#include <Dynamixel2Arduino.h>

// =========================
// Configuración de WiFi
// =========================
const char* ssid = "ESP32_Dynamixel";
const char* password = "12345678";
WiFiServer server(80);

// =========================
// Configuración Dynamixel
// =========================
#define RX_PIN 16
#define TX_PIN 17
#define DXL_DIR_PIN 5

HardwareSerial& DXL_SERIAL = Serial2;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

const int DXL_Baud = 1000000;

// Servos Protocolo 2.0 (Servo 1 y 2)
const uint8_t SERVO_IDS[] = {1, 2};
const int NUM_SERVOS = sizeof(SERVO_IDS) / sizeof(SERVO_IDS[0]);
const float PROTOCOL_VERSION_2 = 2.0;
const int MAX_UNIT_2 = 4095;
const int MAX_ANGLE_2 = 360;

// Servo 3 Protocolo 1.0
const uint8_t SERVO3_ID = 3;
const float PROTOCOL_VERSION_1 = 1.0;
const int MAX_UNIT_1 = 1023;
const int MAX_ANGLE_1 = 300;

// Direcciones para Protocolo 1.0
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

// Nueva definición para leer la posición actual en Protocolo 1.0
#define PRESENT_POSITION_ADDR     36
#define PRESENT_POSITION_ADDR_LEN  2

// Configuración Servo 3
uint16_t servo3_CW_limit = 300;
uint16_t servo3_CCW_limit = 500;
uint16_t servo3_speed = 50;

// Constantes
const uint8_t TURN_ON = 1;
const uint8_t TURN_OFF = 0;
#define TIMEOUT 1000

// Parámetros iniciales para servos 1 y 2
uint16_t position_p_gain[] = {120, 120};
uint16_t position_i_gain[] = {15, 15};
uint16_t position_d_gain[] = {1, 1};
uint16_t velocity[] = {100, 100};
uint16_t acceleration[] = {50, 50};

// =========================
// Estructura para trayectorias
// =========================
struct SimulCommand {
  bool valid;
  int servoID;
  int positions[10];  // Almacena hasta 10 posiciones
  int positionCount;  // Contador actual
  String type;        // "unit" o "angle"
  int p;
  int i;
  int d;
  int velocity;
  int acceleration;
};

SimulCommand simulCommands[4];  // Usaremos índices 1 a 3

// Variables de ejecución
bool isExecuting = false;
int currentStep = 0;

// =========================
// Función auxiliar para extraer parámetros de la petición
// =========================
String getValue(String data, String parameter) {
  int startIndex = data.indexOf(parameter + "=");
  if (startIndex == -1) return "";
  startIndex += parameter.length() + 1;
  int endIndex = data.indexOf("&", startIndex);
  if (endIndex == -1) endIndex = data.indexOf(" ", startIndex);
  if (endIndex == -1) endIndex = data.length();
  return data.substring(startIndex, endIndex);
}

// =========================
// Función auxiliar para esperar a que el servo alcance la posición objetivo
// =========================
bool waitUntilPositionReached(uint8_t servoID, int target, float protocolVersion) {
  const int tolerance = 10;              // Tolerancia en unidades
  const unsigned long maxWait = 5000;      // Tiempo máximo de espera en ms
  unsigned long startTime = millis();

  while (millis() - startTime < maxWait) {
    int currentPos;
    if (protocolVersion == PROTOCOL_VERSION_2) {
      currentPos = dxl.getPresentPosition(servoID);
    } else {
      uint8_t pos;
      dxl.read(servoID, PRESENT_POSITION_ADDR, PRESENT_POSITION_ADDR_LEN,(uint8_t*)&pos,sizeof(pos) ,TIMEOUT);
      currentPos = pos;
    }
    if (abs(currentPos - target) < tolerance) {
      return true; // Se alcanzó la posición objetivo
    }
    delay(50);
  }
  return false; // Tiempo máximo de espera alcanzado sin llegar a la posición
}

// =========================
// Función para ejecutar la secuencia alternada de trayectorias
// =========================
void executeAlternatingSequence() {
  int maxPositions = max(
    max(simulCommands[1].positionCount, simulCommands[2].positionCount),
    simulCommands[3].positionCount
  );

  for (currentStep = 0; currentStep < maxPositions; currentStep++) {
    // Servo 1 (Protocolo 2.0)
    if (currentStep < simulCommands[1].positionCount) {
      dxl.setPortProtocolVersion(PROTOCOL_VERSION_2);
      int target = simulCommands[1].positions[currentStep];
      dxl.setGoalPosition(1, target);
      waitUntilPositionReached(1, target, PROTOCOL_VERSION_2);
    }
    
    // Servo 2 (Protocolo 2.0)
    if (currentStep < simulCommands[2].positionCount) {
      dxl.setPortProtocolVersion(PROTOCOL_VERSION_2);
      int target = simulCommands[2].positions[currentStep];
      dxl.setGoalPosition(2, target);
      waitUntilPositionReached(2, target, PROTOCOL_VERSION_2);
    }
    
    // Servo 3 (Protocolo 1.0)
    if (currentStep < simulCommands[3].positionCount) {
      dxl.setPortProtocolVersion(PROTOCOL_VERSION_1);
      uint16_t target = simulCommands[3].positions[currentStep];
      dxl.write(SERVO3_ID, GOAL_POSITION_ADDR, (uint8_t*)&target, GOAL_POSITION_ADDR_LEN, TIMEOUT);
      waitUntilPositionReached(SERVO3_ID, target, PROTOCOL_VERSION_1);
    }
  }
  
  // Reiniciar las estructuras de comandos
  for (int i = 1; i < 4; i++) {
    simulCommands[i].positionCount = 0;
    simulCommands[i].valid = false;
  }
  isExecuting = false;
}

// =========================
// Setup()
// =========================
void setup() {
  Serial.begin(115200);
  DXL_SERIAL.begin(DXL_Baud, SERIAL_8N1, RX_PIN, TX_PIN);
  dxl.begin(DXL_Baud);

  // Configurar servos 1 y 2 (Protocolo 2.0)
  dxl.setPortProtocolVersion(PROTOCOL_VERSION_2);
  for (int i = 0; i < NUM_SERVOS; i++) {
    uint8_t id = SERVO_IDS[i];
    dxl.ping(id);
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_POSITION);
    dxl.writeControlTableItem(POSITION_P_GAIN, id, position_p_gain[i]);
    dxl.writeControlTableItem(POSITION_I_GAIN, id, position_i_gain[i]);
    dxl.writeControlTableItem(POSITION_D_GAIN, id, position_d_gain[i]);
    dxl.writeControlTableItem(PROFILE_VELOCITY, id, velocity[i]);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, id, acceleration[i]);
    dxl.torqueOn(id);
    Serial.println("Servo " + String(id) + " configurado (Protocolo 2.0)");
  }

  // Configurar servo 3 (Protocolo 1.0)
  dxl.setPortProtocolVersion(PROTOCOL_VERSION_1);
  if (dxl.ping(SERVO3_ID)) {
    dxl.write(SERVO3_ID, TORQUE_ENABLE_ADDR, (uint8_t*)&TURN_OFF, TORQUE_ENABLE_ADDR_LEN, TIMEOUT);
    dxl.write(SERVO3_ID, CW_ANGLE_LIMIT_ADDR, (uint8_t*)&servo3_CW_limit, ANGLE_LIMIT_ADDR_LEN, TIMEOUT);
    dxl.write(SERVO3_ID, CCW_ANGLE_LIMIT_ADDR, (uint8_t*)&servo3_CCW_limit, ANGLE_LIMIT_ADDR_LEN, TIMEOUT);
    dxl.write(SERVO3_ID, MOVING_SPEED_ADDR, (uint8_t*)&servo3_speed, MOVING_SPEED_ADDR_LEN, TIMEOUT);
    dxl.write(SERVO3_ID, TORQUE_ENABLE_ADDR, (uint8_t*)&TURN_ON, TORQUE_ENABLE_ADDR_LEN, TIMEOUT);
    Serial.println("Servo 3 configurado (Protocolo 1.0)");
  }

  dxl.setPortProtocolVersion(PROTOCOL_VERSION_2);

  // Inicializar estructuras de trayectorias
  for (int i = 1; i < 4; i++) {
    simulCommands[i].valid = false;
    simulCommands[i].positionCount = 0;
  }

  // Iniciar WiFi en modo AP
  WiFi.softAP(ssid, password);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  server.begin();
}

// =========================
// Loop()
// =========================
void loop() {
  WiFiClient client = server.available();
  if (client) {
    String request = client.readStringUntil('\r');
    Serial.println("Request: " + request);
    String activeTab = getValue(request, "tab");
    if (activeTab == "") activeTab = "pestana1";

    // Ruta para movimiento inmediato
    if (request.indexOf("/set_position") != -1) {
      String servoStr = getValue(request, "servo");
      int servoID = servoStr.toInt();
      String valueStr = getValue(request, "value");
      int posValue = valueStr.toInt();
      String typeStr = getValue(request, "type");
      
      if (servoID == SERVO3_ID) {
        // Si se indica ángulo, se convierte a unidades internas
        if (typeStr == "angle") posValue = map(posValue, 0, MAX_ANGLE_1, 0, MAX_UNIT_1);
        posValue = constrain(posValue, 0, MAX_UNIT_1);
        dxl.setPortProtocolVersion(PROTOCOL_VERSION_1);
        dxl.write(SERVO3_ID, GOAL_POSITION_ADDR, (uint8_t*)&posValue, GOAL_POSITION_ADDR_LEN, TIMEOUT);
      } else {
        if (typeStr == "angle") posValue = map(posValue, 0, MAX_ANGLE_2, 0, MAX_UNIT_2);
        posValue = constrain(posValue, 0, MAX_UNIT_2);
        dxl.setPortProtocolVersion(PROTOCOL_VERSION_2);
        dxl.setGoalPosition(servoID, posValue);
      }
      
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/html");
      client.println();
      client.println("<html><head><meta charset='UTF-8'>");
      client.println("<script>setTimeout(()=>window.history.back(), 1000)</script>");
      client.println("</head><body>");
      client.println("<h1>Movimiento inmediato enviado a Servo " + String(servoID) + "</h1>");
      client.println("</body></html>");
      client.stop();
      return;
    }
    
    // Ruta para guardar punto de trayectoria
    if (request.indexOf("/guardar_simul") != -1) {
      String servoStr = getValue(request, "servo");
      int servoID = servoStr.toInt();
      String posStr = getValue(request, "position");
      String typeStr = getValue(request, "type");
      
      int posValue = posStr.toInt();
      
      // Conversión de ángulo a unidades
      if (servoID == SERVO3_ID) {
        if (typeStr == "angle") posValue = map(posValue, 0, MAX_ANGLE_1, 0, MAX_UNIT_1);
        posValue = constrain(posValue, 0, MAX_UNIT_1);
      } else {
        if (typeStr == "angle") posValue = map(posValue, 0, MAX_ANGLE_2, 0, MAX_UNIT_2);
        posValue = constrain(posValue, 0, MAX_UNIT_2);
        // Actualizar parámetros PID y de movimiento solo en el primer punto
        if (simulCommands[servoID].positionCount == 0) {
          simulCommands[servoID].p = getValue(request, "p").toInt();
          simulCommands[servoID].i = getValue(request, "i").toInt();
          simulCommands[servoID].d = getValue(request, "d").toInt();
          simulCommands[servoID].velocity = getValue(request, "v").toInt();
          simulCommands[servoID].acceleration = getValue(request, "a").toInt();
        }
      }
      
      // Guardar posición (máximo 10 puntos)
      if (simulCommands[servoID].positionCount < 10) {
        simulCommands[servoID].positions[simulCommands[servoID].positionCount] = posValue;
        simulCommands[servoID].positionCount++;
        simulCommands[servoID].valid = true;
        simulCommands[servoID].type = typeStr;
        simulCommands[servoID].servoID = servoID;
      }
      
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/html");
      client.println();
      client.println("<html><head><meta charset='UTF-8'>");
      client.println("<script>setTimeout(()=>window.history.back(), 1000)</script>");
      client.println("</head><body>");
      client.println("<h1>Punto agregado a Servo " + String(servoID) + "</h1>");
      client.println("<p>Puntos almacenados: " + String(simulCommands[servoID].positionCount) + "/10</p>");
      client.println("</body></html>");
      client.stop();
      return;
    }
    
    // Ruta para ejecutar la secuencia alternada de trayectorias
    if (request.indexOf("/confirm_simul") != -1) {
      if (!isExecuting) {
        isExecuting = true;
        
        // Configurar parámetros PID y de movimiento para servos 1 y 2 (Protocolo 2.0)
        dxl.setPortProtocolVersion(PROTOCOL_VERSION_2);
        for (int id = 1; id <= 2; id++) {
          if (simulCommands[id].valid) {
            dxl.writeControlTableItem(POSITION_P_GAIN, id, simulCommands[id].p);
            dxl.writeControlTableItem(POSITION_I_GAIN, id, simulCommands[id].i);
            dxl.writeControlTableItem(POSITION_D_GAIN, id, simulCommands[id].d);
            dxl.writeControlTableItem(PROFILE_VELOCITY, id, simulCommands[id].velocity);
            dxl.writeControlTableItem(PROFILE_ACCELERATION, id, simulCommands[id].acceleration);
          }
        }
        
        executeAlternatingSequence();
      }
      
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/html");
      client.println();
      client.println("<html><head><meta charset='UTF-8'>");
      client.println("<script>setTimeout(()=>window.history.back(), 500);</script>");
      client.println("</head><body><h1>Ejecutando secuencia alternada...</h1>");
      client.println("</body></html>");
      client.stop();
      return;
    }
    
    // Ruta para enviar el estado (usada por el JS de la página)
    if (request.indexOf("/status") != -1) {
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/plain");
      client.println();
      String status = isExecuting ? "Ejecutando" : "Listo";
      client.println(status + "," + currentStep);
      client.stop();
      return;
    }
    
    // Generar la página web principal
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println();
    client.println(R"=====( 
      <!DOCTYPE html><html lang="es">
      <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Control Servos</title>
        <style>
          .tab-container { display: flex; border-bottom: 2px solid #ccc; }
          .tab { padding: 10px 20px; cursor: pointer; border: 1px solid #ccc; background-color: #f1f1f1; }
          .tab.active { background-color: white; font-weight: bold; }
          .tab-content { display: none; padding: 20px; border: 1px solid #ccc; }
          .tab-content.active { display: block; }
          form { margin-bottom: 20px; padding: 10px; border: 1px solid #aaa; }
          .contador { color: #666; margin-top: 10px; }
          .status-box { padding: 15px; margin: 20px; border: 1px solid #4CAF50; background-color: #e8f5e9; }
        </style>
      </head>
      <body>
        <h1>Control de Servos</h1>
        <div class="status-box">
          <h3>Estado de Ejecución: <span id="estado-ejecucion">Listo</span></h3>
          <p>Paso actual: <span id="paso-actual">0</span></p>
        </div>
        <div class="tab-container">
          <div class="tab active" onclick="openTab(event, 'pestana1')">Servo 1</div>
          <div class="tab" onclick="openTab(event, 'pestana2')">Servo 2</div>
          <div class="tab" onclick="openTab(event, 'pestana3')">Servo 3</div>
        </div>
    )=====");
    
    // Generar contenido para cada pestaña
    for (int servo = 1; servo <= 3; servo++) {
      client.println("<div id='pestana" + String(servo) + "' class='tab-content'" + 
                    (servo == 1 ? " style='display:block'>" : ">"));
      client.println("<h2>Servo " + String(servo) + "</h2>");
      
      // Formulario para movimiento inmediato
      client.println("<h3>Movimiento Inmediato</h3>");
      client.println("<form action='/set_position' method='get'>");
      client.println("<input type='hidden' name='servo' value='" + String(servo) + "'>");
      client.println("<input type='hidden' name='tab' value='pestana" + String(servo) + "'>");
      
      if (servo == 3) {
        client.println("<label>Posición (0-300° o 0-1023):</label><br>");
        client.println("<input type='number' name='value' min='0' max='1023' required><br>");
      } else {
        client.println("<label>Posición (0-360° o 0-4095):</label><br>");
        client.println("<input type='number' name='value' min='0' max='4095' required><br>");
      }
      
      client.println("<input type='radio' name='type' value='unit' checked> Unidad Interna<br>");
      client.println("<input type='radio' name='type' value='angle'> Ángulo<br>");
      client.println("<button type='submit'>Mover</button>");
      client.println("</form>");
      
      // Formulario para guardar puntos de trayectoria
      if (servo != 3) {
        client.println("<h3>Configuración Trayectoria</h3>");
        client.println("<form action='/guardar_simul' method='get'>");
        client.println("<input type='hidden' name='servo' value='" + String(servo) + "'>");
        client.println("<input type='hidden' name='tab' value='pestana" + String(servo) + "'>");
        client.println("<label>Posición:</label><br>");
        client.println("<input type='number' name='position' required><br>");
        client.println("<input type='radio' name='type' value='unit' checked> Unidad Interna<br>");
        client.println("<input type='radio' name='type' value='angle'> Ángulo<br>");
        client.println("<h4>Parámetros PID</h4>");
        client.println("<label>P: <input type='number' name='p' value='120' min='0' max='32767' required></label><br>");
        client.println("<label>I: <input type='number' name='i' value='15' min='0' max='32767' required></label><br>");
        client.println("<label>D: <input type='number' name='d' value='1' min='0' max='32767' required></label><br>");
        client.println("<h4>Movimiento</h4>");
        client.println("<label>Velocidad: <input type='number' name='v' value='100' min='0' max='1023' required></label><br>");
        client.println("<label>Aceleración: <input type='number' name='a' value='50' min='0' max='1023' required></label><br>");
        client.println("<button type='submit'>Agregar Punto</button>");
        client.println("</form>");
      } else {
        client.println("<h3>Configuración Trayectoria</h3>");
        client.println("<form action='/guardar_simul' method='get'>");
        client.println("<input type='hidden' name='servo' value='3'>");
        client.println("<input type='hidden' name='tab' value='pestana3'>");
        client.println("<label>Posición:</label><br>");
        client.println("<input type='number' name='position' required><br>");
        client.println("<input type='radio' name='type' value='unit' checked> Unidad Interna<br>");
        client.println("<input type='radio' name='type' value='angle'> Ángulo<br>");
        client.println("<button type='submit'>Agregar Punto</button>");
        client.println("</form>");
      }
      
      client.println("<div class='contador'>Puntos almacenados: " + 
                    String(simulCommands[servo].positionCount) + "/10</div>");
      client.println("</div>"); // Fin de pestaña
    }
    
    // Footer y JavaScript para cambio de pestañas y actualización de estado
    client.println(R"=====( 
        <div style="margin:20px; padding:10px; border:1px solid #555;">
          <form action="/confirm_simul" method="get">
            <button type="submit">Ejecutar Secuencia Alternada</button>
          </form>
        </div>
        <script>
          function openTab(evt, tabName) {
            document.querySelectorAll('.tab-content').forEach(tab => tab.style.display = 'none');
            document.querySelectorAll('.tab').forEach(tab => tab.classList.remove('active'));
            document.getElementById(tabName).style.display = 'block';
            evt.currentTarget.classList.add('active');
          }
          
          // Actualizar estado cada segundo
          setInterval(() => {
            fetch('/status')
              .then(response => response.text())
              .then(data => {
                const [status, step] = data.split(',');
                document.getElementById('estado-ejecucion').textContent = status;
                document.getElementById('paso-actual').textContent = step;
              });
          }, 1000);
        </script>
      </body>
      </html>
    )=====");
    
    client.stop();
  }
}
