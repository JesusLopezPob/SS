#include <Dynamixel2Arduino.h>


#define DEBUG_SERIAL Serial

// Definición de los pines RX y TX
const int RX_PIN = 16; // Ajusta según tu configuración
const int TX_PIN = 17; // Ajusta según tu configuración

// Definición del pin de dirección para el bus half-duplex
const int DXL_DIR_PIN = 5; // Ajusta según tu configuración

// Inicialización del puerto serial para Dynamixel
HardwareSerial& DXL_SERIAL = Serial2;


// Crea la instancia Dynamixel2Arduino
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;



uint8_t currentID = 1;   // ID actual del servo
uint8_t newID = 1;       // Nuevo ID deseado
uint8_t baudValue = 3;  // Valor para cambiar baudios
int Oldbaud = 57600;  // Valor de baudios actual

void setup() {
  DEBUG_SERIAL.begin(115200);
  DXL_SERIAL.begin(Oldbaud, SERIAL_8N1, RX_PIN, TX_PIN); // Baud rate actual del AX18A

  // Configura el protocolo (1.0 para AX18A)
  dxl.begin(Oldbaud);
  dxl.setPortProtocolVersion(float(2.0));
  DEBUG_SERIAL.println(dxl.scan());

  // Verifica si el servo responde al ID actual
  if (dxl.ping(currentID)) {
    DEBUG_SERIAL.println("Servo detectado correctamente.");

    // Cambia el ID AX
    /*
    if (dxl.write(currentID, 0x03, &newID, 1, 100)) {
      DEBUG_SERIAL.println("ID cambiado correctamente.");
    } else {
      DEBUG_SERIAL.println("Error al cambiar el ID.");
    }

    // Cambia el baud rate
    if (dxl.write(newID, 0x04, &baudValue, 1, 100)) {
      DEBUG_SERIAL.println("Baud rate cambiado correctamente.");
    } else {
      DEBUG_SERIAL.println("Error al cambiar el baud rate.");
    }
*/

    // Cambia el ID XM
    /*
    if (dxl.write(currentID, 0x07, &newID, 1, 100)) {
      DEBUG_SERIAL.println("ID cambiado correctamente.");
    } else {
      DEBUG_SERIAL.println("Error al cambiar el ID.");
    }
*/
    // Cambia el baud rate
    if (dxl.write(currentID, 0x08, &baudValue, 1, 100)) {
      DEBUG_SERIAL.println("Baud rate cambiado correctamente.");
    } else {
      DEBUG_SERIAL.println("Error al cambiar el baud rate.");
    }

  } else {
    DEBUG_SERIAL.println("No se detectó el servo. Verifica el ID o las conexiones.");
  }
}

void loop() {
  // Nada en el loop
}
