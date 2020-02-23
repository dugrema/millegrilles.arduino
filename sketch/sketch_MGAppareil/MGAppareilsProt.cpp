#include "MGAppareilsProt.h"
#include <Arduino.h>

void MGProtocoleV9::lireBeaconDhcp(byte* data, byte* adresseServeur) {

  // Init 2 premiers bytes a 0 (serveur a une adresse de 24 bits)
  adresseServeur[0] = 0;
  adresseServeur[1] = 0;

  // Copier 3 bytes reseau
  for(byte i=0; i<3; i++) {
    adresseServeur[i+2] = data[i+3];
  }
  
}

byte MGProtocoleV9::lireReponseDhcp(byte* data, byte* adresseNoeud) {
  // Copier l'adresse recue dans le buffer addresseNoeud
  // L'adresse est dans les bytes [3:7], le premier byte est le nodeId
  memcpy(adresseNoeud, data + 3, 5);
  byte nodeId_reserve = adresseNoeud[0];
  return nodeId_reserve;
}

// Ecrit UUID a partir du program space vers un buffer
void MGProtocoleV9::ecrireUUID(byte* destination) {
  // memcpy_P(destination, _uuid, 16);  // Utiliser si UUID est dans PROGMEM
  memcpy(destination, _uuid, 16);
}

byte* MGProtocoleV9::getCleBuffer() {
  return (byte*) &_cle;
}

byte* MGProtocoleV9::getIvBuffer() {
  return (byte*) &_iv;
}

byte* MGProtocoleV9::executerDh1() {
  // Creer un buffer temporaire pour sauvegarder la cle privee
  _bufferTemp = new byte[32];

  Curve25519::dh1(_cle, _bufferTemp);

  // Retourner la cle publique
  return (byte*)&_cle;
}

bool MGProtocoleV9::executerDh2() {
  // Note, copier la cle publique distante dans _cle (getCle()) avant d'appeler cette methode
  bool succes = false;
  
  if(_bufferTemp != 0x0) {
    // Calculer la cle secrete, sauvegarder
    succes = Curve25519::dh2(_cle, _bufferTemp);
  
    // Nettoyage, on n'a plus besoin du buffer avec la cle privee
    delete _bufferTemp;
    _bufferTemp = 0x0;  // Comment faire null en C++?
  }

  return succes;
}

bool MGProtocoleV9::transmettrePaquet0(uint16_t typeMessage) {
    uint8_t transmitBuffer[PAYLOAD_TAILLE_SIMPLE];

    // Format message :
    // Version (1 byte)
    // Node ID (1 byte)
    // TYPE MESSAGE 2 bytes
    // UUID 16 bytes

    uint16_t typePaquet0 = MSG_TYPE_PAQUET0;
    
    _buffer[0] = VERSION_PROTOCOLE;
    _buffer[1] = _nodeId[0];
    memcpy(_buffer + 2, &typePaquet0, sizeof(typePaquet0));
    memcpy(_buffer + 4, &typeMessage, sizeof(typeMessage));
    ecrireUUID(_buffer + 6);

    bool transmissionOk = false;
    byte compteurTransmissions = 0;
    _radio->stopListening();
    while(!transmissionOk && compteurTransmissions++ < LIMITE_RETRANSMISSION) {
      transmissionOk = _radio->write(_buffer, PAYLOAD_TAILLE_SIMPLE);
    }
    _radio->startListening();

    if(!transmissionOk) return false;  // Erreur transmission, abort
  
    // Transmettre IV utilise pour crypter/decrypter message
    transmissionOk = transmettrePaquetIv(1);

    return transmissionOk;
}

bool MGProtocoleV9::transmettrePaquetFin(byte noPaquet) {
  // Format message :
  // Version (1 byte)
  // Node ID (1 byte)
  // TYPE MESSAGE 2 bytes
  // Nombre paquets 2 bytes
  // Message Tag eax256 - 16 bytes (optionnel)

  bool transmissionOk = false;
  
//  // IV optionnel pour message non cryptes (e.g. DHCP)
//  transmissionOk = transmettrePaquetIv(noPaquet); // Transmettre IV
//  noPaquet++;  // Incrementer no paquet
  
//  if(transmissionOk) {
//    transmissionOk = false;

    uint16_t typeMessage = MSG_TYPE_PAQUET_FIN;
    // uint16_t noPaquetInt = noPaquet;
  
    _buffer[0] = VERSION_PROTOCOLE;
    _buffer[1] = _nodeId[0];
    memcpy(_buffer + 2, &typeMessage, sizeof(typeMessage));
    memcpy(_buffer + 4, &noPaquet, sizeof(noPaquet));

//    if(messageTag != 0x0) {
//      memcpy(_buffer + 6, messageTag, 16);
//    } else {
      memcpy(_buffer + 6, 0x0, 16);  // Vider le buffer
//    }
  
    byte compteurTransmissions = 0;
    _radio->stopListening();
    while(!transmissionOk && compteurTransmissions++ < LIMITE_RETRANSMISSION) {
      transmissionOk = _radio->write(_buffer, PAYLOAD_TAILLE_SIMPLE);
    }
    _radio->startListening();
    
//  }

  return transmissionOk;
}

bool MGProtocoleV9::transmettrePaquetIv(byte noPaquet) {
  // Format message :
  // Version (1 byte)
  // Node ID (1 byte)
  // TYPE MESSAGE 2 bytes
  // Nombre paquets 2 bytes
  // Message Tag eax256 - 16 bytes (optionnel)

  uint16_t typeMessage = MSG_TYPE_PAQUET_IV;
  uint16_t noPaquetInt = noPaquet;

  _buffer[0] = VERSION_PROTOCOLE;
  _buffer[1] = _nodeId[0];
  memcpy(_buffer + 2, &typeMessage, sizeof(typeMessage));
  memcpy(_buffer + 4, &noPaquetInt, sizeof(noPaquetInt));
  memcpy(_buffer + 6, &_iv, 16);

  bool transmissionOk = false;
  byte compteurTransmissions = 0;
  _radio->stopListening();
  while(!transmissionOk && compteurTransmissions++ < LIMITE_RETRANSMISSION) {
    transmissionOk = _radio->write(_buffer, PAYLOAD_TAILLE_SIMPLE);
  }
  _radio->startListening();
  
  return transmissionOk;
}

bool MGProtocoleV9::transmettrePaquetCrypte(uint16_t noPaquet) {
  return false;
}

bool MGProtocoleV9::transmettreRequeteDhcp() {
    uint8_t transmitBuffer[PAYLOAD_TAILLE_SIMPLE];

    // Format message :
    // Version - 1 byte
    // Node ID - 1 byte (placeholder)
    // TYPE MESSAGE - 2 bytes
    // No paquet - 2 bytes
    // UUID 16 bytes

    uint16_t typeMessage = MSG_TYPE_REQUETE_DHCP;
    
    _buffer[0] = VERSION_PROTOCOLE;
    _buffer[1] = 0;  // Blank, nodeId n'est pas connu
    memcpy(_buffer + 2, &typeMessage, sizeof(typeMessage));
    ecrireUUID(_buffer + 4);

    bool transmissionOk = false;
    byte compteurTransmissions = 0;
    _radio->stopListening();
    while(!transmissionOk && compteurTransmissions++ < LIMITE_RETRANSMISSION) {
      transmissionOk = _radio->write(_buffer, PAYLOAD_TAILLE_SIMPLE);
    }
    _radio->startListening();

    return transmissionOk;
}

bool MGProtocoleV9::transmettrePaquet(byte taillePayload) {
  bool transmissionOk = false;
  byte compteurTransmissions = 0;
  
  _radio->stopListening();
  while(!transmissionOk && compteurTransmissions++ < LIMITE_RETRANSMISSION) {
    transmissionOk = _radio->write(_buffer, taillePayload);
  }
  _radio->startListening();
  
  return transmissionOk;
}

bool MGProtocoleV9::transmettrePaquetsClePublique(uint16_t noPaquet) {
  // Transmet une cle publique de 32 bytes en 2 paquets

  // Format message THP (Temperatures, Humidite, Pression Atmospherique)
  // Version - 1 byte
  // Node ID - 1 byte
  // typeMessage - 2 bytes
  // noPaquet - 2 bytes
  // clePublique - 26 bytes / 6 bytes

  uint16_t typeMessage = MSG_TYPE_CLE_LOCALE_1;
  byte* clePublique = getCleBuffer();

  _buffer[0] = VERSION_PROTOCOLE;
  _buffer[1] = _nodeId[0];
  memcpy(_buffer + 2, &typeMessage, sizeof(typeMessage));
  memcpy(_buffer + 4, &noPaquet, sizeof(noPaquet));
  memcpy(_buffer + 6, clePublique, 26);

  bool transmissionOk = transmettrePaquet(PAYLOAD_TAILLE_SIMPLE);

  if(!transmissionOk) {
    // Abort
    return false;
  }

  // Tramsettre le reste de la cle publique
  uint16_t noPaquetSuivant = noPaquet + 1;
  typeMessage = MSG_TYPE_CLE_LOCALE_2;
  memcpy(_buffer + 2, &typeMessage, sizeof(typeMessage));
  memcpy(_buffer + 4, &noPaquetSuivant, sizeof(noPaquetSuivant));
  memcpy(_buffer + 6, clePublique + 26, 6);

  transmissionOk = transmettrePaquet(PAYLOAD_TAILLE_SIMPLE);

  return transmissionOk;
  
}

bool MGProtocoleV9::transmettrePaquetLectureTH(uint16_t noPaquet, FournisseurLectureTH* fournisseur) {

  // Format message THP (Temperatures, Humidite, Pression Atmospherique)
  // Version - 1 byte
  // Node ID - 1 byte
  // typeMessage - 2 bytes
  // noPaquet - 2 bytes
  // temperature - 2 bytes
  // humidite - 2 bytes

  uint16_t typeMessage = MSG_TYPE_LECTURE_TH;

  int temperature = fournisseur->temperature();
  uint16_t humidite = fournisseur->humidite();

  _buffer[0] = VERSION_PROTOCOLE;
  _buffer[1] = _nodeId[0];
  memcpy(_buffer + 2, &typeMessage, sizeof(typeMessage));
  memcpy(_buffer + 4, &noPaquet, sizeof(noPaquet));
  memcpy(_buffer + 6, &temperature, sizeof(temperature));
  memcpy(_buffer + 8, &humidite, sizeof(humidite));

  return transmettrePaquet(PAYLOAD_TAILLE_SIMPLE);
}

bool MGProtocoleV9::transmettrePaquetLectureTP(uint16_t noPaquet, FournisseurLectureTP* fournisseur) {

  // Format message TP (Temperatures,  Pression Atmospherique)
  // Version - 1 byte
  // Node ID - 1 byte
  // typeMessage - 2 bytes
  // noPaquet - 2 bytes
  // typeMessage - 2 bytes
  // temperature - 2 bytes
  // pression - 2 bytes

  uint16_t typeMessage = MSG_TYPE_LECTURE_TP;

  int temperature = fournisseur->temperature();
  uint16_t pression = fournisseur->pression();

  _buffer[0] = VERSION_PROTOCOLE;
  _buffer[1] = _nodeId[0];
  memcpy(_buffer + 2, &typeMessage, sizeof(typeMessage));
  memcpy(_buffer + 4, &noPaquet, sizeof(noPaquet));
  memcpy(_buffer + 6, &temperature, sizeof(temperature));
  memcpy(_buffer + 8, &pression, sizeof(pression));

  return transmettrePaquet(PAYLOAD_TAILLE_SIMPLE);
}

bool MGProtocoleV9::transmettrePaquetLecturePower(uint16_t noPaquet, FournisseurLecturePower* fournisseur) {

  // Format message Power
  // Version - 1 byte
  // Node ID - 1 byte
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
  _buffer[1] = _nodeId[0];
  memcpy(_buffer + 2, &typeMessage, sizeof(typeMessage));
  memcpy(_buffer + 4, &noPaquet, sizeof(noPaquet));
  memcpy(_buffer + 6, &millivolt, sizeof(millivolt));
  memcpy(_buffer + 10, &reservePct, sizeof(reservePct));
  memcpy(_buffer + 11, &alerte, sizeof(alerte));

  return transmettrePaquet(PAYLOAD_TAILLE_SIMPLE);
}

bool MGProtocoleV9::transmettrePaquetLectureOneWire(uint16_t noPaquet, FournisseurLectureOneWire* fournisseur) {
  // Format message 1W
  // Version - 1 byte
  // Node ID - 1 byte
  // typeMessage - 2 bytes
  // noPaquet - 2 bytes
  // adresse - 8 bytes
  // data - 12 bytes

  uint16_t typeMessage = MSG_TYPE_LECTURE_ONEWIRE;

  _buffer[0] = VERSION_PROTOCOLE;
  _buffer[1] = _nodeId[0];
  memcpy(_buffer + 2, &typeMessage, sizeof(typeMessage));
  memcpy(_buffer + 4, &noPaquet, sizeof(noPaquet));
  memcpy(_buffer + 6, fournisseur->adresse(), 8);
  memcpy(_buffer + 14, fournisseur->data(), 12);

  return transmettrePaquet(PAYLOAD_TAILLE_SIMPLE);
}

//bool MGProtocoleV9::transmettrePaquetLectureMillivolt(uint16_t noPaquet, uint32_t millivolt1, uint32_t millivolt2, uint32_t millivolt3, uint32_t millivolt4) {
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
