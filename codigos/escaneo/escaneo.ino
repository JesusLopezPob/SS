#include <Dynamixel2Arduino.h>

// Definición de los pines RX y TX
const int RX_PIN = 16; // Ajusta según tu configuración
const int TX_PIN = 17; // Ajusta según tu configuración

// Definición del pin de dirección para el bus half-duplex
const int DXL_DIR_PIN = 5; // Ajusta según tu configuración

bool found=false;

// Inicialización del puerto serial para Dynamixel
HardwareSerial& DXL_SERIAL = Serial2;

// Creación de la instancia de Dynamixel2Arduino
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// Definición de los baud rates a escanear
#define MAX_BAUD 6
int baud[MAX_BAUD] = {9600, 57600, 115200, 1000000, 2000000, 3000000};

// Variable global para almacenar el ID del Dynamixel encontrado
int found_id = -1;

void setup() {
  // Inicialización del puerto serial para depuración
  Serial.begin(115200);
  while (!Serial);

  // Configuración de los pines RX y TX
  DXL_SERIAL.begin(57600, SERIAL_8N1, RX_PIN, TX_PIN);
  dxl.begin(57600); // Establece el pin de dirección

  // Escaneo de baud rates y protocolos
  for (int protocol = 1; protocol <= 2; protocol++) {
    dxl.setPortProtocolVersion((float)protocol);
    Serial.print("Escaneando con protocolo ");
    Serial.println(protocol);

    for (int i = 0; i < MAX_BAUD; i++) {
      Serial.print("Escaneando a baud rate: ");
      Serial.println(baud[i]);
      DXL_SERIAL.updateBaudRate(baud[i]);
      dxl.begin(baud[i]);
      found=dxl.scan();
      Serial.print("inicio Scan : ");
      Serial.println(found);

      // Escaneo de IDs
      if (found==1){
              for (int id = 1; id < 253; id++) {
        if (dxl.ping(id)) {
          Serial.print("Dynamixel encontrado - ID: ");
          Serial.print(id);
          Serial.print(", Número de modelo: ");
          Serial.println(dxl.getModelNumber(id));
          found_id = id;
          break;
        }
      }
      }

      if (found_id != -1) break;
    }
    if (found_id != -1) break;
  }

  if (found_id == -1) {
    Serial.println("No se encontró ningún Dynamixel.");
  } else {
    // Configuración adicional si se encuentra un Dynamixel
    dxl.torqueOff(found_id);
    dxl.setOperatingMode(found_id, OP_POSITION);
    dxl.torqueOn(found_id);
    dxl.ledOn(1);
    delay(100);
    dxl.ledOff(1);
  }
}

void loop() {
  // Ejemplo de movimiento del servomotor
  /*
  if (found_id != -1 && dxl.ping(found_id)) {
    dxl.setGoalPosition(found_id, 512); // Posición en formato raw
    delay(1000);

    Serial.print("Posición actual (raw): ");
    Serial.println(dxl.getPresentPosition(found_id));
    delay(1000);

    dxl.setGoalPosition(found_id, 90.0, UNIT_DEGREE); // Posición en grados
    delay(1000);
    Serial.print("Posición actual (grados): ");
    Serial.println(dxl.getPresentPosition(found_id, UNIT_DEGREE));
    delay(1000);

    dxl.setGoalPosition(found_id, 180.0, UNIT_DEGREE); // Posición en grados
    delay(1000);
    Serial.print("Posición actual (grados): ");
    Serial.println(dxl.getPresentPosition(found_id, UNIT_DEGREE));
    delay(1000);

    dxl.setGoalPosition(found_id, 270.0, UNIT_DEGREE); // Posición en grados
    delay(1000);
    Serial.print("Posición actual (grados): ");
    Serial.println(dxl.getPresentPosition(found_id, UNIT_DEGREE));
    delay(1000);
  }
  */
  dxl.ledOn(1);
}
