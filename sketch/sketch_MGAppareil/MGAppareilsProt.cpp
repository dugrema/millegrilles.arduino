#include "MGAppareilsProt.h"
#include <Arduino.h>

byte MGProtocoleV7::lireReponseDhcp(byte* data) {
  byte nodeId_reserve = data[3];
  return nodeId_reserve;
}

// Ecrit UUID a partir du program space vers un buffer
void MGProtocoleV7::ecrireUUID(byte* destination) {
  // memcpy_P(destination, _uuid, 16);  // Utiliser si UUID est dans PROGMEM
  memcpy(destination, _uuid, 16);
}

bool MGProtocoleV7::transmettrePaquet0(uint16_t typeMessage, uint16_t nombrePaquets) {
    uint8_t transmitBuffer[24];

    // Format message Paquet0:
    // Version (1 byte)
    // TYPE MESSAGE 2 bytes
    // Nombre paquets 2 bytes
    // UUID 16 bytes
    _buffer[0] = VERSION_PROTOCOLE;
    memcpy(_buffer + 1, &typeMessage, sizeof(typeMessage));
    memcpy(_buffer + 3, &nombrePaquets, sizeof(nombrePaquets));
    ecrireUUID(_buffer + 5);

    bool transmissionOk = false;
    for(byte essai=0; !transmissionOk && essai<5; essai++) { 
      transmissionOk = _mesh->write(_buffer, 'P', PAYLOAD_TAILLE_SIMPLE, MESH_MASTER_ID);
      if(!transmissionOk) {
        if( ! _mesh->checkConnection() ) {
          _mesh->renewAddress(2000);
        }
      }
    }

    return transmissionOk;
}

bool MGProtocoleV7::transmettreRequeteDhcp() {
    uint8_t transmitBuffer[24];

    // Format message Paquet0:
    // Version (1 byte)
    // TYPE MESSAGE 2 bytes
    // Nombre paquets 2 bytes
    // UUID 16 bytes

    uint16_t typeMessage = MSG_TYPE_REQUETE_DHCP;
    
    _buffer[0] = VERSION_PROTOCOLE;
    memcpy(_buffer + 1, &typeMessage, sizeof(typeMessage));
    ecrireUUID(_buffer + 3);

    bool transmissionOk = false;
    for(byte essai=0; !transmissionOk && essai<15; essai++) {
      transmissionOk = _mesh->write(_buffer, 'D', PAYLOAD_TAILLE_SIMPLE, MESH_MASTER_ID);
      if(!transmissionOk) {
        if( ! _mesh->checkConnection() ) {
          _mesh->renewAddress(2000);
        }
      }
    }

    return transmissionOk;
}

bool MGProtocoleV7::transmettrePaquet(byte taillePayload) {
  bool transmissionOk = false;
  
  char typePaquet = 'p';
  if(taillePayload == PAYLOAD_TAILLE_DOUBLE) typePaquet = '2';
  
  for(byte essai=0; !transmissionOk && essai<20; essai++) { 
    transmissionOk = _mesh->write(_buffer, typePaquet, taillePayload, MESH_MASTER_ID);
    if(!transmissionOk) {
      if( ! _mesh->checkConnection() ) {
        _mesh->renewAddress(2000);
      }
    }
  }
  
  return transmissionOk;
}

bool MGProtocoleV7::transmettrePaquetLectureTH(uint16_t noPaquet, FournisseurLectureTH* fournisseur) {

  // Format message THP (Temperatures, Humidite, Pression Atmospherique)
  // Version - 1 byte
  // typeMessage - 2 bytes
  // noPaquet - 2 bytes
  // temperature - 2 bytes
  // humidite - 2 bytes

  uint16_t typeMessage = MSG_TYPE_LECTURE_TH;

  int temperature = fournisseur->temperature();
  uint16_t humidite = fournisseur->humidite();

  _buffer[0] = VERSION_PROTOCOLE;
  memcpy(_buffer + 1, &typeMessage, sizeof(typeMessage));
  memcpy(_buffer + 3, &noPaquet, sizeof(noPaquet));
  memcpy(_buffer + 5, &temperature, sizeof(temperature));
  memcpy(_buffer + 7, &humidite, sizeof(humidite));

  return transmettrePaquet(PAYLOAD_TAILLE_SIMPLE);
}

bool MGProtocoleV7::transmettrePaquetLectureTP(uint16_t noPaquet, FournisseurLectureTP* fournisseur) {

  // Format message TP (Temperatures,  Pression Atmospherique)
  // Version - 1 byte
  // typeMessage - 2 bytes
  // noPaquet - 2 bytes
  // typeMessage - 2 bytes
  // temperature - 2 bytes
  // pression - 2 bytes

  uint16_t typeMessage = MSG_TYPE_LECTURE_TP;

  int temperature = fournisseur->temperature();
  uint16_t pression = fournisseur->pression();

  _buffer[0] = VERSION_PROTOCOLE;
  memcpy(_buffer + 1, &typeMessage, sizeof(typeMessage));
  memcpy(_buffer + 3, &noPaquet, sizeof(noPaquet));
  memcpy(_buffer + 5, &temperature, sizeof(temperature));
  memcpy(_buffer + 7, &pression, sizeof(pression));

  return transmettrePaquet(PAYLOAD_TAILLE_SIMPLE);
}

bool MGProtocoleV7::transmettrePaquetLecturePower(uint16_t noPaquet, FournisseurLecturePower* fournisseur) {

  // Format message Power
  // Version - 1 byte
  // typeMessage - 2 bytes
  // noPaquet - 2 bytes
  // millivolt - 4 bytes
  // reservePct - 1 bytes
  // alerte - 1 bytes

  uint16_t typeMessage = MSG_TYPE_LECTURE_POWER;

  uint32_t millivolt = fournisseur->millivolt();
  byte reservePct = fournisseur->reservePct();
  byte alerte = fournisseur->alerte();

  _buffer[0] = VERSION_PROTOCOLE;
  memcpy(_buffer + 1, &typeMessage, sizeof(typeMessage));
  memcpy(_buffer + 3, &noPaquet, sizeof(noPaquet));
  memcpy(_buffer + 5, &millivolt, sizeof(millivolt));
  memcpy(_buffer + 9, &reservePct, sizeof(reservePct));
  memcpy(_buffer + 10, &alerte, sizeof(alerte));

  return transmettrePaquet(PAYLOAD_TAILLE_SIMPLE);
}

bool MGProtocoleV7::transmettrePaquetLectureOneWire(uint16_t noPaquet, FournisseurLectureOneWire* fournisseur) {
  // Format message Power
  // Version - 1 byte
  // typeMessage - 2 bytes
  // noPaquet - 2 bytes
  // adresse - 8 bytes
  // data - 12 bytes

  uint16_t typeMessage = MSG_TYPE_LECTURE_ONEWIRE;

  _buffer[0] = VERSION_PROTOCOLE;
  memcpy(_buffer + 1, &typeMessage, sizeof(typeMessage));
  memcpy(_buffer + 3, &noPaquet, sizeof(noPaquet));
  memcpy(_buffer + 5, fournisseur->adresse(), 8);
  memcpy(_buffer + 13, fournisseur->data(), 12);

  return transmettrePaquet(PAYLOAD_TAILLE_DOUBLE);
}

//bool MGProtocoleV7::transmettrePaquetLectureMillivolt(uint16_t noPaquet, uint32_t millivolt1, uint32_t millivolt2, uint32_t millivolt3, uint32_t millivolt4) {
//
//  // Format message millivol - supporte jusqu'a 4 milliards de volts
//  // noPaquet - 2 bytes
//  // typeMessage - 2 bytes
//  // millivolt 1 - 4 bytes
//  // millivolt 2 - 4 bytes
//  // millivolt 3 - 4 bytes
//  // millivolt 4 - 4 bytes
//
//  uint16_t typeMessage = 0x104;
//
//  memcpy(_buffer + 0, &noPaquet, sizeof(noPaquet));
//  memcpy(_buffer + 2, &typeMessage, sizeof(typeMessage));
//  memcpy(_buffer + 4, &millivolt1, sizeof(millivolt1));
//  memcpy(_buffer + 8, &millivolt2, sizeof(millivolt2));
//  memcpy(_buffer + 12, &millivolt3, sizeof(millivolt3));
//  memcpy(_buffer + 16, &millivolt4, sizeof(millivolt4));
//
//  return transmettrePaquet();
//}
