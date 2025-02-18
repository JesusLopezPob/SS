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

//int32_t goal_position[2] = {1200, 1600};
int32_t goal_position[2] = {300, 450};
int8_t direction = 0;
unsigned long timer = 0;

// Position PID Gains
// Adjust these gains to tune the behavior of DYNAMIXEL


uint16_t position_p_gain = 125;//120
uint16_t position_i_gain = 15;
uint16_t position_d_gain = 1;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  // For Uno, Nano, Mini, and Mega, use the UART port of the DYNAMIXEL Shield to read debugging messages.
  DEBUG_SERIAL.begin(57600);
  while(!DEBUG_SERIAL);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(DXL_Baud);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  
  // Set Position PID Gains
  dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID, position_p_gain);
  dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID, position_i_gain);
  dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID, position_d_gain);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Read Present Position (Use the Serial Plotter)
  while(true) {
    DEBUG_SERIAL.print("Goal_Position:");
    DEBUG_SERIAL.print(dxl.readControlTableItem(GOAL_POSITION, DXL_ID));
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print("Present_Position:");
    DEBUG_SERIAL.print(dxl.getPresentPosition(DXL_ID));
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.println();
    delay(10);

    if (millis() - timer >= 2500) {
      dxl.setGoalPosition(DXL_ID, goal_position[direction]);
      timer = millis();
      break;
    }
  }

  if(direction >= 1) {
    direction = 0;
  } else {
    direction = 1;
  }
}
