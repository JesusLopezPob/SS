#include <Dynamixel2Arduino.h>

#define DEBUG_SERIAL Serial

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
int i_present_position = 0;
float f_present_position = 0.0;

using namespace ControlTableItem;

void setup() {
  // Inicialización del puerto serial para depuración
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL);

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

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 32);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
   
  // Set Goal Position in RAW value
  dxl.setGoalPosition(DXL_ID, 2000);

  int i_present_position = 0;
  float f_present_position = 0.0;

  while (abs(2000 - i_present_position) > 10)
  {
    i_present_position = dxl.getPresentPosition(DXL_ID);
    DEBUG_SERIAL.print("Present_Position(raw) : ");
    DEBUG_SERIAL.println(i_present_position);
  }
  delay(1000);

  // Set Goal Position in DEGREE value
  dxl.setGoalPosition(DXL_ID, 5.7, UNIT_DEGREE);
  
  while (abs(5.7 - f_present_position) > 1.0)
  {
    f_present_position = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    DEBUG_SERIAL.print("Present_Position(degree) : ");
    DEBUG_SERIAL.println(f_present_position);
  }
  delay(1000);
}
