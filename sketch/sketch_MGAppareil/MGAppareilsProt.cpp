#include "MGAppareilsProt.h"
#include <Arduino.h>

// Ecrit UUID a partir du program space vers un buffer
void MGProtocoleV7::ecrireUUID(byte* destination) {
  memcpy_P(destination, &_uuid, 16);
}

void MGProtocoleV7::transmettrePaquets() {
  // Preparer paquet0 et transmettre
  _mesh->write(_buffer, 'P', 24, MESH_MASTER_ID);

  // Transmettres autres paquets
  _mesh->write(_buffer, 'p', 24, MESH_MASTER_ID);
}

bool MGProtocoleV7::transmettrePaquet0(uint16_t typeMessage, uint16_t nombrePaquets) {
    uint8_t transmitBuffer[24];

    // Format message Paquet0:
    // Version (1 byte)
    // UUID 16 bytes
    // TYPE MESSAGE 2 bytes
    // Nombre paquets 2 bytes
    _buffer[0] = VERSION_PROTOCOLE;
    ecrireUUID(_buffer + 1);
    memcpy(_buffer + 17, &typeMessage, sizeof(typeMessage));
    memcpy(_buffer + 20, &nombrePaquets, sizeof(nombrePaquets));
    _buffer[22] = 0;
    _buffer[23] = 0;
    
    // printArray(transmitBuffer, sizeof(transmitBuffer));
    bool transmissionOk = _mesh->write(_buffer, 'P', sizeof(_buffer), MESH_MASTER_ID);
    if(!transmissionOk) {
      Serial.println("Erreur transmission paquet 0");
    } else {
      Serial.println("Paquet 0 ok");
    }

    return transmissionOk;
}

bool MGProtocoleV7::transmettrePaquet() {
  bool transmissionOk = _mesh->write(_buffer, 'p', 24, MESH_MASTER_ID);
  return transmissionOk;
}

bool MGProtocoleV7::transmettrePaquetLectureTHP(uint16_t noPaquet, int temperature, uint16_t humidite, uint16_t pression) {

  // Format message THP (Temperatures, Humidite, Pression Atmospherique)
  // noPaquet - 2 bytes
  // typeMessage - 2 bytes
  // temperature - 2 bytes
  // humidite - 2 bytes
  // pression - 2 bytes

  uint16_t typeMessage = 0x102;

  memcpy(_buffer + 0, &noPaquet, sizeof(noPaquet));
  memcpy(_buffer + 2, &typeMessage, sizeof(typeMessage));
  memcpy(_buffer + 4, &temperature, sizeof(temperature));
  memcpy(_buffer + 6, &humidite, sizeof(humidite));
  memcpy(_buffer + 8, &pression, sizeof(pression));

  return transmettrePaquet();
}

bool MGProtocoleV7::transmettrePaquetLectureMillivolt(uint16_t noPaquet, uint32_t millivolt1, uint32_t millivolt2, uint32_t millivolt3, uint32_t millivolt4) {

  // Format message millivol - supporte jusqu'a 4 milliards de volts
  // noPaquet - 2 bytes
  // typeMessage - 2 bytes
  // millivolt 1 - 4 bytes
  // millivolt 2 - 4 bytes
  // millivolt 3 - 4 bytes
  // millivolt 4 - 4 bytes

  uint16_t typeMessage = 0x103;

  memcpy(_buffer + 0, &noPaquet, sizeof(noPaquet));
  memcpy(_buffer + 2, &typeMessage, sizeof(typeMessage));
  memcpy(_buffer + 4, &millivolt1, sizeof(millivolt1));
  memcpy(_buffer + 8, &millivolt2, sizeof(millivolt2));
  memcpy(_buffer + 12, &millivolt3, sizeof(millivolt3));
  memcpy(_buffer + 16, &millivolt4, sizeof(millivolt4));

  return transmettrePaquet();
}
