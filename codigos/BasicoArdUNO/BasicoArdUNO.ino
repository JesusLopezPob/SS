#include <Dynamixel2Arduino.h>

// Configuración para Arduino Uno
#define DXL_SERIAL   Serial   // Usamos Serial para la comunicación (UART del Arduino Uno)
#define DEBUG_SERIAL Serial   // Puerto serial para depuración
const int DXL_DIR_PIN = 2;    // Pin de dirección para el Shield Dynamixel (puedes usar cualquier pin digital disponible)

// Inicialización de la biblioteca Dynamixel
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
//Dynamixel2Arduino dxl(DXL_SERIAL);

using namespace ControlTableItem;

uint8_t DXL_ID = 1;
float DXL_PROTOCOL_VERSION = 2.0;

#define MAX_BAUD  6
long buad[MAX_BAUD] = {9600, 57600, 115200, 1000000, 2000000, 3000000};  // Cambiado a 'long'

void setup() {
  int8_t index = 0;
  int8_t found_dynamixel = 0;

  // Inicialización del puerto serial para depuración
  DEBUG_SERIAL.begin(115200);

  // Inicialización del puerto serial para Dynamixel
  dxl.begin(57600); // Configura Dynamixel a 115200 bps
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  /*

  // Bucle de espera hasta que el servo sea encontrado
  while (!dxl.ping(DXL_ID)) {
    //DEBUG_SERIAL.println("No Dynamixel found, retrying...");
    for (int8_t protocol = 2; protocol < 3; protocol++) {
      dxl.setPortProtocolVersion((float)protocol);
      //DEBUG_SERIAL.print("SCAN PROTOCOL ");
      //DEBUG_SERIAL.println((float)protocol);

      delay(90);
      for (index = 1; index < 2; index++) {
        //DEBUG_SERIAL.print("SCAN BAUDRATE ");
        //DEBUG_SERIAL.println(buad[index]);
        dxl.begin(buad[index]); // Configura el baudrate
        for (int id = 0; id < DXL_BROADCAST_ID; id++) {
          if (dxl.ping(id)) {
            //DEBUG_SERIAL.print("ID : ");
            //DEBUG_SERIAL.print(id);
            //DEBUG_SERIAL.print(", Model Number: ");
            //DEBUG_SERIAL.println(dxl.getModelNumber(id));
            DXL_ID = id;
            dxl.ping(DXL_ID);
            dxl.ledOn(DXL_ID);
            delay(5000);
            dxl.ledOff(DXL_ID);
          }
          delay(50);
        }
      }
    }
  }

  //DEBUG_SERIAL.println("Dynamixel found.");

  */

  // Configuración del servo una vez encontrado
  dxl.ping(DXL_ID);
  dxl.ledOn(DXL_ID);
  delay(5000);
  dxl.ledOff(DXL_ID);
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
}

void loop() {
  // Lógica de control del servo
  dxl.setGoalPosition(DXL_ID, 512);  // Posición en formato raw
  delay(1000);
  DEBUG_SERIAL.print("Present Position (raw): ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID));
  delay(1000);

  dxl.setGoalPosition(DXL_ID, 5.7, UNIT_DEGREE);  // Posición en grados
  delay(1000);
  DEBUG_SERIAL.print("Present Position (degree): ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE));
  delay(1000);

  dxl.setGoalPosition(DXL_ID, 0, UNIT_DEGREE);  // Posición en grados
  delay(1000);
  DEBUG_SERIAL.print("Present Position (degree): ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE));
  delay(1000);
}
