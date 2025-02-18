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

// Declaración de parámetros del Dynamixel
const uint8_t DXL_ID_AX = 2;
const float DXL_PROTOCOL_VERSION_AX = 1.0;

const uint8_t DXL_ID_XM = 1;
const float DXL_PROTOCOL_VERSION_XM = 2.0;

const int DXL_Baud = 1000000;

// Definición de direcciones de control
#define TORQUE_ENABLE_ADDR 24
#define CW_ANGLE_LIMIT_ADDR 6
#define CCW_ANGLE_LIMIT_ADDR 8
#define GOAL_POSITION_ADDR 30
#define LED_ADDR 25
#define MOVING_SPEED_ADDR 32  // Registro para la velocidad de movimiento

// Definición de longitudes de los registros
#define TORQUE_ENABLE_ADDR_LEN 1
#define ANGLE_LIMIT_ADDR_LEN 2
#define GOAL_POSITION_ADDR_LEN 2
#define LED_ADDR_LEN 1
#define MOVING_SPEED_ADDR_LEN 2  // Longitud del registro de velocidad

// Variables
const uint8_t turn_on = 1;
const uint8_t turn_off = 0;
uint16_t goalPosition1 = 300;
uint16_t goalPosition2 = 450;
float pos;

// Tiempo de espera
#define TIMEOUT 1000

void setup() {
  // Inicialización del puerto serial para depuración
  Serial.begin(115200);
  while (!Serial);

  // Configuración de los pines RX y TX
  DXL_SERIAL.begin(DXL_Baud, SERIAL_8N1, RX_PIN, TX_PIN);
  dxl.begin(DXL_Baud); // Establece el pin de dirección



}

void loop() {
  iniciarProtUno();
  dxl.write(DXL_ID_AX, GOAL_POSITION_ADDR, (uint8_t*)&goalPosition1, GOAL_POSITION_ADDR_LEN, TIMEOUT);
  Serial.print("Posición objetivo: ");
  Serial.println(goalPosition1);


  iniciarProtDos();
  dxl.setGoalPosition(DXL_ID_XM, 500);

  delay(2000);

  iniciarProtUno();
  dxl.write(DXL_ID_AX, GOAL_POSITION_ADDR, (uint8_t*)&goalPosition2, GOAL_POSITION_ADDR_LEN, TIMEOUT);
  Serial.print("Posición objetivo: ");
  Serial.println(goalPosition2);


  iniciarProtDos();
  dxl.setGoalPosition(DXL_ID_XM, 270.0, UNIT_DEGREE); // Posición en grados

  delay(2000);

}



void iniciarProtUno(){
//---------AX18A---------
  // Configuración de la versión del protocolo
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION_AX);

  // Verificar si el servo responde
  if (dxl.ping(DXL_ID_AX)) {
    Serial.println("Dynamixel AX18A encontrado y listo.");
  } else {
    Serial.println("Error: No se pudo encontrar el Dynamixel.");
    while (true);  // Detiene el programa si no se encuentra el servo
  }

  // Apagar el torque antes de realizar configuraciones
  if (dxl.write(DXL_ID_AX, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_off , TORQUE_ENABLE_ADDR_LEN, TIMEOUT)) {
    Serial.println("Torque apagado");
  } else {
    Serial.println("Error: No se pudo apagar el torque");
  }

  // Configuración de los límites de ángulo
  if (dxl.write(DXL_ID_AX, CW_ANGLE_LIMIT_ADDR, (uint8_t*)&goalPosition1, ANGLE_LIMIT_ADDR_LEN, TIMEOUT) &&
      dxl.write(DXL_ID_AX, CCW_ANGLE_LIMIT_ADDR, (uint8_t*)&goalPosition2, ANGLE_LIMIT_ADDR_LEN, TIMEOUT)) {
    Serial.println("Modo de operación configurado");
  } else {
    Serial.println("Error: No se pudo configurar el modo de operación");
  }

  // Configurar la velocidad de movimiento (ajustar según sea necesario)
  uint16_t speed = 100; // Ajusta la velocidad según sea necesario
  if (dxl.write(DXL_ID_AX, MOVING_SPEED_ADDR, (uint8_t*)&speed, MOVING_SPEED_ADDR_LEN, TIMEOUT)) {
    Serial.println("Velocidad de movimiento configurada");
  } else {
    Serial.println("Error: No se pudo configurar la velocidad");
  }

  // Activar el torque
  if (dxl.write(DXL_ID_AX, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_on, TORQUE_ENABLE_ADDR_LEN, TIMEOUT)) {
    Serial.println("Torque activado");
  } else {
    Serial.println("Error: No se pudo activar el torque");
  }

}

void iniciarProtDos(){
//------------------XM430--------------
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION_XM);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID_XM);


  // Configuración adicional si se encuentra un Dynamixel
  dxl.torqueOff(DXL_ID_XM);
  dxl.setOperatingMode(DXL_ID_XM, OP_POSITION);
  dxl.torqueOn(DXL_ID_XM);
  Serial.println("Dynamixel XM430 encontrado y listo.");

}


