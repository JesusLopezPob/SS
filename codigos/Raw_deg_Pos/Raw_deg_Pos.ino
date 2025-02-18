#include <Dynamixel2Arduino.h>

// Definición de los pines RX y TX
const int RX_PIN = 16; // Ajusta según tu configuración
const int TX_PIN = 17; // Ajusta según tu configuración

// Definición del pin de dirección para el bus half-duplex
const int DXL_DIR_PIN = 5; // Ajusta según tu configuración

// Inicialización del puerto serial para Dynamixel
HardwareSerial& DXL_SERIAL = Serial2;

// Creación de la instancia de Dynamixel2Arduino
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//declaracion de parametros del Dynamixel
const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;
const int DXL_Baud=1000000;

//variables 
float pos;

void setup() {
  // Inicialización del puerto serial para depuración
  Serial.begin(115200);
  while (!Serial);

  // Configuración de los pines RX y TX
  DXL_SERIAL.begin(DXL_Baud, SERIAL_8N1, RX_PIN, TX_PIN);
  dxl.begin(DXL_Baud); // Establece el pin de dirección

  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);


  // Configuración adicional si se encuentra un Dynamixel
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);

}

void loop() {
  // Ejemplo de movimiento del servomotor
    dxl.setGoalPosition(DXL_ID, 300); // Posición en formato raw
    delay(1000);

    Serial.print("Posición actual (raw): ");
    Serial.println(dxl.getPresentPosition(DXL_ID));
    delay(1000);

    dxl.setGoalPosition(DXL_ID, 90.0, UNIT_DEGREE); // Posición en grados
    delay(1000);
    Serial.print("Posición actual (grados): ");
    pos=dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    Serial.println(pos);
    delay(1000);

    dxl.setGoalPosition(DXL_ID, 120.0, UNIT_DEGREE); // Posición en grados
    delay(1000);
    Serial.print("Posición actual (grados): ");
    pos=dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    Serial.println(pos);
    delay(1000);

    dxl.setGoalPosition(DXL_ID, 150.0, UNIT_DEGREE); // Posición en grados
    delay(1000);
    Serial.print("Posición actual (grados): ");
    pos=dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    Serial.println(pos);
    delay(1000);
}
