#include "MGAppareilsProt.h"
#include <Arduino.h>

void MGProtocoleV9::loop() {

  // Ajuster le niveau PA de la radio
  if( stats.nombrePaquets > 16 || stats.nombreTransmissions > 256 ) {
    
    uint8_t currentPALevel = _radio->getPALevel();

    #ifdef LOGGING_DEV
      Serial.print(F("Nb erreurs : "));
      Serial.print(stats.nombreErreurs);
      Serial.print(F(", nb transmissions : "));
      Serial.println(stats.nombreTransmissions);
    #endif
    
    float pctEchec = float(stats.nombreErreurs) / float(stats.nombreTransmissions);
    stats.forceSignalPct = 100 - byte(pctEchec * 100.0f);  // Inverser valeur (100 - pct erreur -> force signal)
    
    if( pctEchec > 0.85f && currentPALevel < RF24_PA_MAX ) {
      
      // Augmenter la puissance d'emission
      _radio->setPALevel(++currentPALevel);
      
      #ifdef LOGGING_DEV
        Serial.print(F("Augmentation puissance emission. "));
      #endif
      
    } else if( pctEchec < 0.15f && currentPALevel > RF24_PA_MIN ) { 
      
      // Diminuer la puissance d'emission
      _radio->setPALevel(--currentPALevel);

      #ifdef LOGGING_DEV
        Serial.print(F("Reduction puissance emission. "));
      #endif
      
    } 

    // Reset stats
    stats.nombreTransmissions = 0;
    stats.nombreErreurs = 0;
    stats.nombrePaquets = 0;

    #ifdef LOGGING_DEV
      Serial.print(F("Transmission signal "));
      Serial.print(stats.forceSignalPct);
      Serial.print(F("%, PA Level : "));
      Serial.println(currentPALevel);
    #endif
    
  }
  
}

byte MGProtocoleV9::pctSignal() {
  return stats.forceSignalPct;
}

byte MGProtocoleV9::forceEmetteur() {
  return _radio->getPALevel();
}

byte MGProtocoleV9::canal() {
  return _radio->getChannel();
}

byte MGProtocoleV9::nombreCyclesAbortConsecutifs() {
  return stats.nombreCyclesAbortConsecutifs;
}

void MGProtocoleV9::resetNombreCyclesAbortConsecutifs() {
  stats.nombreCyclesAbortConsecutifs = 0;
}


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

// Ecrit UUID a partir du EEPROM space vers un buffer
void MGProtocoleV9::ecrireUUID(byte* destination) {
  // memcpy_P(destination, _uuid, 16);  // Utiliser si UUID est dans PROGMEM
  byte uuid[16];
  EEPROM.get(EEPROM_UUID, uuid);
  memcpy(destination, uuid, 16);
}

byte* MGProtocoleV9::getCleBuffer() {
  return (byte*) &_cle;
}

byte* MGProtocoleV9::getIvBuffer() {
  return (byte*) &_iv;
}

bool MGProtocoleV9::isTransmissionOk() {
  return _transmissionOk;
}

bool MGProtocoleV9::isAckRecu() {
  return _ackRecu;
}

bool MGProtocoleV9::isClePriveePrete() {
  return _clePriveePrete;
}

byte* MGProtocoleV9::executerDh1() {
  // Generer la cle publique
  // Si la cle privee n'existe pas, on va aussi la generer
  
  byte bufferClePrivee[32];

  // Desactiver le cryptage de messages pour faire l'echange de cles publiques
  _cryptageActif = false;

  if( ! _clePriveePrete ) {
    Curve25519::dh1(_cle, bufferClePrivee);

    // Sauvegarder la cle privee dans le EEPROM
    EEPROM.put(EEPROM_CLE_PRIVEE, bufferClePrivee);

    // Sauvegarder flag pour indiquer que la cle privee est prete
    EEPROM.update(EEPROM_CLE_PRIVEE_ETAT, CLE_PRIVEE_ETAT_PRETE);

    _clePriveePrete = true;
  } else {
    // Regenerer la cle publique a partir de la cle privee
    EEPROM.get(EEPROM_CLE_PRIVEE, bufferClePrivee);
    Curve25519::eval(_cle, bufferClePrivee, 0);
  }

  // Retourner la cle publique
  return (byte*)&_cle;
}

bool MGProtocoleV9::executerDh2() {
  // Note, copier la cle publique distante dans _cle (getCle()) avant d'appeler cette methode
  bool succes = false;

  if( _clePriveePrete ) {

    // Recuperer la cle privee du EEPROM
    byte bufferClePrivee[32];
    EEPROM.get(EEPROM_CLE_PRIVEE, bufferClePrivee);
    
    // Calculer la cle secrete, sauvegarder
    succes = Curve25519::dh2(_cle, bufferClePrivee);
    
  }

  return succes;
}

void MGProtocoleV9::activerCryptage() {
  _cryptageActif = true;
}

bool MGProtocoleV9::initCipher(byte* authData, byte authDataLen, byte* iv) {

  if( ! _cryptageActif ) {
    return false;
  }

  cipher.clear();
    
  if (!cipher.setKey((byte*)&_cle, cipher.keySize())) {
      Serial.print("setKey ");
      return false;
  }

  if (!cipher.setIV(iv, 16)) {
      Serial.print("setIV ");
      return false;
  }

  cipher.addAuthData(authData, authDataLen);

  return true;
}

void MGProtocoleV9::encryptBuffer(byte* buffer, byte bufferLen) {
  cipher.encrypt(buffer, buffer, bufferLen);
}

void MGProtocoleV9::decryptBuffer(byte* buffer, byte bufferLen) {
  cipher.decrypt(buffer, buffer, bufferLen);
}

void MGProtocoleV9::computeTag(byte* outputTag) {
  if(_cryptageActif) {
    // Le cryptage est actif
    cipher.computeTag(outputTag, 16);
  } else {
    // Remplace le buffer par 16 bytes de 0
    memset(outputTag, 0x0, 16);
  }
}

byte MGProtocoleV9::transmettrePaquet0(uint16_t typeMessage) {

    // Format message :
    // Version       -  1 byte
    // Node ID       -  1 byte
    // Numero paquet -  2 bytes
    // TYPE Paquet0  -  2 bytes
    // TYPE MESSAGE  -  2 bytes
    // UUID          - 16 bytes

    stats.nombreCyclesAbortConsecutifs++;  // Va etre resette des qu'un paquet ACK est recu

    uint16_t typePaquet0 = MSG_TYPE_PAQUET0;
    byte buffer[32];
    
    buffer[0] = VERSION_PROTOCOLE;
    buffer[1] = _nodeId[0];
    memcpy(buffer + 2, &typePaquet0, sizeof(typePaquet0)); // Numero paquet = 0
    memcpy(buffer + 4, &typePaquet0, sizeof(typePaquet0));
    memcpy(buffer + 6, &typeMessage, sizeof(typeMessage));
    ecrireUUID(buffer + 8);

    transmettrePaquet(PAYLOAD_TAILLE_SIMPLE, (byte*)&buffer);

    byte nombrePaquets = 0;
    if(!_transmissionOk) return nombrePaquets;  // Erreur transmission, abort
    nombrePaquets++;
  
    if( _cryptageActif ) { // Verifier si cryptage actif
      byte iv[16];
      RNG.rand((byte*)&iv, 16);
      
      initCipher(buffer, 22, (byte*)&iv);
      
      // Transmettre IV utilise pour crypter/decrypter message
      // Ce message indique implicitement que le reste de la transmission est cryptee
      // Auth data est le paquet 0 au complet, garanti la provenance
      transmettrePaquetIv(1, (byte*)&iv);
      
      nombrePaquets++;
    }

    return nombrePaquets;
}

bool MGProtocoleV9::transmettrePaquetFin(byte noPaquet) {
  // Format message :
  // Version            -  1 byte
  // Node ID            -  1 byte
  // Numero paquet      -  2 bytes
  // TYPE MESSAGE       -  2 bytes
  // Nombre paquets     -  2 bytes
  // Message Tag eax256 - 16 bytes

  uint16_t typeMessage = MSG_TYPE_PAQUET_FIN;
  uint16_t nombrePaquets = noPaquet;
  byte buffer[32];

  buffer[0] = VERSION_PROTOCOLE;
  buffer[1] = _nodeId[0];
  memcpy(buffer + 2, &typeMessage, sizeof(typeMessage));  // Le numero de paquet est 0xFFFF, paquet de fin
  memcpy(buffer + 4, &typeMessage, sizeof(typeMessage));
  memcpy(buffer + 6, &nombrePaquets, sizeof(nombrePaquets));

  // Calculer le tag (hash) pour message crypte
  // Note : pour les message non cryptes, le buffer est mis a 0 sur 16 bytes.
  computeTag(buffer + 8);

  _ackRecu = false; // On attend un ACK pour cette transmission
  
  return transmettrePaquet(PAYLOAD_TAILLE_SIMPLE, (byte*)&buffer);
}

bool MGProtocoleV9::transmettrePaquetIv(byte noPaquet, byte* iv) {
  // Format message :
  // Version            -  1 byter
  // Node ID            -  1 byte
  // Numbero Paquet     -  2 bytes
  // TYPE MESSAGE       -  2 bytes
  // IV                 - 16 bytes

  uint16_t typeMessage = MSG_TYPE_PAQUET_IV;
  uint16_t noPaquetInt = noPaquet;
  byte buffer[32];

  buffer[0] = VERSION_PROTOCOLE;
  buffer[1] = _nodeId[0];
  memcpy(buffer + 2, &noPaquetInt, sizeof(noPaquetInt));
  memcpy(buffer + 4, &typeMessage, sizeof(typeMessage));
  memcpy(buffer + 6, iv, 16);

  return transmettrePaquet(PAYLOAD_TAILLE_SIMPLE, (byte*)&buffer);
}

bool MGProtocoleV9::transmettrePaquetCrypte(byte taillePayload, byte* buffer) {

  // Les 4 premiers bytes sont utilise pour routage, ils ne doivent pas etre crypte
  byte* bufferData = buffer + 4;

  // Crypter buffer
  cipher.encrypt(bufferData, bufferData, taillePayload - 4);

  return transmettrePaquet(taillePayload, buffer);

}

bool MGProtocoleV9::transmettreRequeteDhcp() {
    uint8_t transmitBuffer[PAYLOAD_TAILLE_SIMPLE];

    // Format message :
    // Version      -  1 byte
    // Node ID      -  1 byte (placeholder)
    // No Paquet    -  2 bytes
    // TYPE MESSAGE -  2 bytes
    // UUID         - 16 bytes

    uint16_t typeMessage = MSG_TYPE_REQUETE_DHCP;
    byte buffer[32];
    
    buffer[0] = VERSION_PROTOCOLE;
    buffer[1] = 0;  // Blank, nodeId n'est pas connu
    memset(buffer + 2, 0x0, 2);  // No Paquet est 0
    memcpy(buffer + 4, &typeMessage, sizeof(typeMessage));
    ecrireUUID(buffer + 6);

    return transmettrePaquet(PAYLOAD_TAILLE_SIMPLE, (byte*)&buffer);
}

bool MGProtocoleV9::transmettrePaquet(byte taillePayload, byte* buffer) {
  _transmissionOk = false;
  byte compteurTransmissions = 0;

  // Compter le nombre de paquets depuis depuis reset stats
  stats.nombrePaquets++;
  
  while(!_transmissionOk && compteurTransmissions++ < LIMITE_RETRANSMISSION) {
    _radio->stopListening();
    _transmissionOk = _radio->write(buffer, PAYLOAD_TAILLE_SIMPLE, (byte*)&buffer);

    uint8_t retransmissions = _radio->getARC();
    #ifdef LOGGING_DEV
      Serial.print("Retransmissions : "); Serial.println(retransmissions);
    #endif
    stats.nombreTransmissions += RF24_RETRANSMISSIONS;  // retransmissions + 1;
    stats.nombreErreurs += retransmissions;

//    // Cumuler stats de transmission pour diagnostic
//    stats.nombreTransmissions++;
//    if( ! _transmissionOk ) {
//      stats.nombreErreurs++;
//    }
    
    _radio->startListening();
    delayMicroseconds(150);
  }
  
  return _transmissionOk;
}

bool MGProtocoleV9::transmettrePaquetsClePublique(uint16_t noPaquet) {
  // Transmet une cle publique de 32 bytes en 2 paquets

  // Format message THP (Temperatures, Humidite, Pression Atmospherique)
  // Version - 1 byte
  // Node ID - 1 byte
  // noPaquet - 2 bytes
  // typeMessage - 2 bytes
  // clePublique - 26 bytes / 6 bytes

  uint16_t typeMessage = MSG_TYPE_CLE_LOCALE_1;
  byte* clePublique = getCleBuffer();
  byte buffer[32];

  buffer[0] = VERSION_PROTOCOLE;
  buffer[1] = _nodeId[0];
  memcpy(buffer + 2, &noPaquet, sizeof(noPaquet));
  memcpy(buffer + 4, &typeMessage, sizeof(typeMessage));
  memcpy(buffer + 6, clePublique, 26);

  bool transmissionOk = transmettrePaquet(PAYLOAD_TAILLE_SIMPLE, (byte*)&buffer);

  if(!transmissionOk) {
    // Abort
    return false;
  }

  // Tramsettre le reste de la cle publique
  uint16_t noPaquetSuivant = noPaquet + 1;
  typeMessage = MSG_TYPE_CLE_LOCALE_2;
  memcpy(buffer + 2, &noPaquetSuivant, sizeof(noPaquetSuivant));
  memcpy(buffer + 4, &typeMessage, sizeof(typeMessage));
  memcpy(buffer + 6, clePublique + 26, 6);

  transmissionOk = transmettrePaquet(PAYLOAD_TAILLE_SIMPLE, (byte*)&buffer);

  return transmissionOk;
  
}

bool MGProtocoleV9::transmettrePaquetLectureTH(uint16_t noPaquet, FournisseurLectureTH* fournisseur) {

  // Format message THP (Temperatures, Humidite, Pression Atmospherique)
  // Version - 1 byte
  // Node ID - 1 byte
  // noPaquet - 2 bytes
  // typeMessage - 2 bytes
  // temperature - 2 bytes
  // humidite - 2 bytes

  uint16_t typeMessage = MSG_TYPE_LECTURE_TH;
  byte buffer[32];

  int temperature = fournisseur->temperature();
  uint16_t humidite = fournisseur->humidite();

  buffer[0] = VERSION_PROTOCOLE;
  buffer[1] = _nodeId[0];
  memcpy(buffer + 2, &noPaquet, sizeof(noPaquet));
  memcpy(buffer + 4, &typeMessage, sizeof(typeMessage));
  memcpy(buffer + 6, &temperature, sizeof(temperature));
  memcpy(buffer + 8, &humidite, sizeof(humidite));

  return transmettrePaquetCrypte(PAYLOAD_TAILLE_SIMPLE, (byte*)&buffer);
}

bool MGProtocoleV9::transmettrePaquetLectureTP(uint16_t noPaquet, FournisseurLectureTP* fournisseur) {

  // Format message TP (Temperatures,  Pression Atmospherique)
  // Version - 1 byte
  // Node ID - 1 byte
  // noPaquet - 2 bytes
  // typeMessage - 2 bytes
  // temperature - 2 bytes
  // pression - 2 bytes

  uint16_t typeMessage = MSG_TYPE_LECTURE_TP;
  byte buffer[32];

  int temperature = fournisseur->temperature();
  uint16_t pression = fournisseur->pression();

  buffer[0] = VERSION_PROTOCOLE;
  buffer[1] = _nodeId[0];
  memcpy(buffer + 2, &noPaquet, sizeof(noPaquet));
  memcpy(buffer + 4, &typeMessage, sizeof(typeMessage));
  memcpy(buffer + 6, &temperature, sizeof(temperature));
  memcpy(buffer + 8, &pression, sizeof(pression));

  return transmettrePaquetCrypte(PAYLOAD_TAILLE_SIMPLE, (byte*)&buffer);
}

bool MGProtocoleV9::transmettrePaquetLecturePower(uint16_t noPaquet, FournisseurLecturePower* fournisseur) {

  // Format message Power
  // Version - 1 byte
  // Node ID - 1 byte
  // noPaquet - 2 bytes
  // typeMessage - 2 bytes
  // millivolt - 4 bytes
  // reservePct - 1 bytes
  // alerte - 1 bytes

  uint16_t typeMessage = MSG_TYPE_LECTURE_POWER;
  byte buffer[32];

  uint32_t millivolt = fournisseur->millivolt();
  byte reservePct = fournisseur->reservePct();
  byte alerte = fournisseur->alerte();

  buffer[0] = VERSION_PROTOCOLE;
  buffer[1] = _nodeId[0];
  memcpy(buffer + 2, &noPaquet, sizeof(noPaquet));
  memcpy(buffer + 4, &typeMessage, sizeof(typeMessage));
  memcpy(buffer + 6, &millivolt, sizeof(millivolt));
  memcpy(buffer + 10, &reservePct, sizeof(reservePct));
  memcpy(buffer + 11, &alerte, sizeof(alerte));

  return transmettrePaquetCrypte(PAYLOAD_TAILLE_SIMPLE, (byte*)&buffer);
}

bool MGProtocoleV9::transmettrePaquetLectureOneWire(uint16_t noPaquet, FournisseurLectureOneWire* fournisseur) {
  // Format message 1W
  // Version - 1 byte
  // Node ID - 1 byte
  // noPaquet - 2 bytes
  // typeMessage - 2 bytes
  // adresse - 8 bytes
  // data - 12 bytes

  uint16_t typeMessage = MSG_TYPE_LECTURE_ONEWIRE;
  byte buffer[32];

  buffer[0] = VERSION_PROTOCOLE;
  buffer[1] = _nodeId[0];
  memcpy(buffer + 2, &noPaquet, sizeof(noPaquet));
  memcpy(buffer + 4, &typeMessage, sizeof(typeMessage));
  memcpy(buffer + 6, fournisseur->adresse(), 8);
  memcpy(buffer + 14, fournisseur->data(), 12);

  return transmettrePaquetCrypte(PAYLOAD_TAILLE_SIMPLE, (byte*)&buffer);
}

bool MGProtocoleV9::transmettrePaquetLectureAntenne(uint16_t noPaquet, FournisseurLectureAntenne* fournisseur) {
  // Format message Antenne
  // Version       - 1 byte
  // Node ID       - 1 byte
  // noPaquet      - 2 bytes
  // typeMessage   - 2 bytes
  // pctSignal     - 1 byte
  // forceEmetteur - 1 byte
  // canal         - 1 byte

  uint16_t typeMessage = MSG_TYPE_LECTURE_ANTENNE;
  byte buffer[32];

  buffer[0] = VERSION_PROTOCOLE;
  buffer[1] = _nodeId[0];
  memcpy(buffer + 2, &noPaquet, sizeof(noPaquet));
  memcpy(buffer + 4, &typeMessage, sizeof(typeMessage));
  buffer[6] = fournisseur->pctSignal();
  buffer[7] = fournisseur->forceEmetteur();
  buffer[8] = fournisseur->canal();

  return transmettrePaquetCrypte(PAYLOAD_TAILLE_SIMPLE, (byte*)&buffer);
}

bool MGProtocoleV9::transmettreLectureTHAntennePower(FournisseurLectureTHAntennePower* fournisseur) {
  // Format message THP (Temperatures, Humidite)
  // Version       - 1 byte
  // Node ID       - 1 byte
  // noPaquet      - 2 bytes - 0 hard coded
  // typeMessage   - 2 bytes - MSG_TYPE_LECTURE_TH_ANTENNE_POWER
  // temperature   - 2 bytes
  // humidite      - 2 bytes
  // pctSignal     - 1 byte
  // forceEmetteur - 1 byte
  // canal         - 1 byte
  // batterie      - 2 bytes
  // compute tag   - 16 bytes

  uint16_t typeMessage = MSG_TYPE_LECTURE_TH_ANTENNE_POWER;
  byte buffer[32];

  int temperature = fournisseur->temperature();
  uint16_t humidite = fournisseur->humidite();
  uint16_t millivolt = fournisseur->millivolt();

  buffer[0] = VERSION_PROTOCOLE;
  buffer[1] = _nodeId[0];
  buffer[2] = 0; buffer[3] = 0;  // Numero paquet 0, hard coded
  memcpy(buffer + 4, &typeMessage, sizeof(typeMessage));

  // Payload
  memcpy(buffer + 6, &temperature, sizeof(temperature));
  memcpy(buffer + 8, &humidite, sizeof(humidite));
  buffer[10] = fournisseur->pctSignal();
  buffer[11] = fournisseur->forceEmetteur();
  buffer[12] = fournisseur->canal();
  memcpy(buffer + 13, &millivolt, sizeof(millivolt));

  return transmettreMessageCrypte(15, (byte*)&buffer);
}


bool MGProtocoleV9::transmettreLectureTPAntennePower(FournisseurLectureTPAntennePower* fournisseur) {
  
}


// Un message crypte est un payload complet qui contient les donnees et le compute tag
// Requiert que l'identification, preparation de la cle et du IV soit deja fait
bool MGProtocoleV9::transmettreMessageCrypte(byte taillePayload, byte* buffer) {

  // Initialiser chiffrage avec 6 bytes de routage et le IV
  initCipher(buffer, 6, getIvBuffer());

  // Les 6 premiers bytes sont utilise pour routage, ils ne doivent pas etre crypte
  byte* bufferData = buffer + 6;

  // Crypter buffer
  cipher.encrypt(bufferData, bufferData, taillePayload - 6);
  cipher.computeTag(buffer + taillePayload, 16);

  return transmettrePaquet(taillePayload, buffer);

}

//  Recevoir information
uint16_t MGProtocoleV9::recevoirPaquet(byte* buffer, byte bufferLen) {
  
  uint16_t typeMessage;
  
  _radio->read(buffer, bufferLen);
  if(buffer[0] != VERSION_PROTOCOLE) {
    // Version incorrecte, abort
    return MSG_TYPE_PAQUET_INCONNU;
  }
  
  memcpy((byte*)&typeMessage, buffer+1, 2);

  if( typeMessage == MSG_TYPE_REPONSE_ACK ) {
    _ackRecu = true;
    stats.nombreCyclesAbortConsecutifs = 0;  // Reset nombre de transmission abort
  }

  return typeMessage;
}
