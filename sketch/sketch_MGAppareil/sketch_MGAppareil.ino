#include <EEPROM.h>
#include "Config.h"
#include "Power.h"
#include "MGAppareilsProt.h"

#include <SPI.h>
#include <RF24.h>

#include <RNG.h>
#include <TransistorNoiseSource.h>

#if defined(defLOGGING_DEV) || defined(LOGGING_DEV_RADIO)
  #include <printf.h>
#endif


// ***********************************

// Configuration

// UUID, va etre charge a partir de la memoire EEPROM
byte modePairing = PAIRING_PAS_INIT;

// *****************
// Senseurs
// *****************

// DHT
#if defined(DHTPIN) && defined(DHTTYPE)
  #include "dht.h"
  MilleGrillesDHT dht(DHTPIN, DHTTYPE);
#endif

// Determiner le type de mode d'utilisation du bus de l'appareil.
// Supporte soit I2C, soit OneWire.
#ifdef BUS_MODE_ONEWIRE

  // OneWire
  #include "OneWireHandler.h"
  OneWireHandler oneWireHandler(A4);  // Pin A4 - meme que I2C data

#elif defined(BUS_MODE_I2C)

  // Adafruit BMP
  #include "AdafruitSensors.h"
  MilleGrillesAdafruitSensors bmp;

#endif

// *****************

// Utilise par RNG
TransistorNoiseSource noise1(A1);

// Radio nRF24L01 sur pins CE=7, CSN=8
RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);
volatile bool messageRecu = false;

struct {
  byte nodeId = NODE_ID_DEFAULT;

  // Utilise pour indiquer quand debuter la transmission
  // Permet d'utiliser une valeur random pour diminuer les collisions de transmissions
  uint16_t attenteTransmission = 0;

  long derniereAction = 0;  // Utiliser pour derminer prochaine action (sleep ou lecture selon power mode)

  // bool ivUsed = true;  // Va creer un IV random avant la premiere transmission
  bool refreshIv = true;  // Force la transmission d'un nouveau IV

  // Renouveller nodeId. Toujours verifier nodeId avec DHCP au demarrage
  // bool doitVerifierAdresseDhcp = true;
  bool ecouterBeacon = true;
  
  // bool prot9.isTransmissionOk() = false;  // Flag qui indique un echec de transmission
  bool ackTransmission = true;  // Si false, indique qu'on attend un ACK de transmission
  
  bool lectureDue = true;
  
  bool bypassSleep = false;
} infoReseau;

struct {
  // Output analogique sur le LED, 0xFF est max et 0x00 est OFF.
  byte pinOutput = 0xFF;

  // Indique direction de changement du LED (augmentation ou diminution de l'intensite)
  bool directionPin = false;

  // Utilise le nombre de bits pour controler la frequence de rafraichissement du LED
  unsigned int throttlePin : 4;
  
} ledStatus;

// Helper conversion de donnees avec protocole Version 7
MGProtocoleV9 prot9(&radio, &infoReseau.nodeId);

// Power management
ArduinoPower power;
volatile int f_wdt=1;


// Conflit avec lib crypto.h, __vector_6 (wachdog) deja defini
#ifndef WATCHDOG_INITD
#define WATCHDOG_INITD
  ISR(WDT_vect)
  {
    if(f_wdt == 0)
    {
      f_wdt=1;
    }
  }
#endif

// Setup initial
void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  
  Serial.begin(115200);

  #ifdef LOGGING_DEV_RADIO
    printf_begin();  // Utilise par radio.printDetails()
  #endif

  #ifdef MG_DEV
    Serial.println(F(" ****** Mode operation DEV ****** "));
  #elif defined(MG_INT)
    Serial.println(F(" ****** Mode operation INT ****** "));
  #endif

  chargerConfiguration();

  // Initialize the random number generator.
  RNG.setAutoSaveTime(120); // 2 heures

  // Add the noise source to the list of sources known to RNG.
  RNG.addNoiseSource(noise1);

  // Preparation senseurs
  #ifdef BUS_MODE_I2C
    bmp.begin();
  #endif

  #ifdef LOGGING_DEV_RADIO
    Serial.println(F("Setup radio"));
  #endif
  
  radio.begin();
  radio.setChannel(RADIO_CANAL);
  radio.setAutoAck(true);
  radio.setRetries(6, 2);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setPALevel(RF24_PA_MIN);  // Commencer avec emission MIN, va etre ajuste

  // Garder la radio hors ligne pour generer cle privee (au besoin)
  radio.powerDown();

  digitalWrite(PIN_LED, LOW);
  delay(200);
  digitalWrite(PIN_LED, HIGH);

  Serial.print(F("Setup termine apres "));
  Serial.println(millis());

//  #ifdef LOGGING_DEV
//    afficherTailleObjets();
//  #endif

}

void activerRadioDhcp() {
  // Ouvrir reading pipe avec adresse reception beacon DHCP
  radio.powerUp();
  uint64_t addresseDhcp = BROADCAST_DHCP_LISTEN;
  radio.openReadingPipe(1, addresseDhcp);

  #ifdef LOGGING_DEV_RADIO
    Serial.print(F("Ouverture radio"));
    radio.printDetails();
  #endif
  
  radio.startListening();
  radio.maskIRQ(true, true, false); // Masquer TX OK et fail sur IRQ, juste garder payload ready
  attachInterrupt(digitalPinToInterrupt(RF24_IRQ_PIN), checkRadio, FALLING);

  #ifdef LOGGING_DEV_RADIO
    Serial.println(F("Radio initialisee, ecoute beacon DHCP"));
  #endif
}

void loop() {

  // Perform regular housekeeping on the random number generator.
  RNG.loop();
  // Entretien radio/protocole
  prot9.loop();

  infoReseau.bypassSleep = false;

  // S'assurer que DHCP a ete execute correctement avant de transmettre
  // On n'effectue pas de lecture sur reception de commande
  bool actionExpiree = attenteActionExpiree();
  
  if( infoReseau.lectureDue && ! messageRecu && attenteActionExpiree() ) {

    #ifdef LOGGING_DEV_RADIO
      Serial.println(F("Action transmission"));
    #endif

    actionTransmission();
  }

  // Ecouter reseau
  ecouterReseau();

  // Verifier si on peut entrer en mode sleep
  if(power.isAlimentationSecteur()) {
    // Comportement sur alimentation secteur
    // On ne sleep jamais sur alimentation secteur
    infoReseau.bypassSleep = true;

    long attente;
    if( prot9.isTransmissionOk() && prot9.isAckRecu() ) {
      attente = ATTENTE_SECTEUR;
    } else {
      attente = 8000L;  // Retransmettre plus rapidement
    }
    if(millis() - infoReseau.derniereAction > attente) {
      // Declencher nouvelle lecture senseur (alimentation secteur)
      infoReseau.lectureDue = true;

      #ifdef LOGGING_DEV
        Serial.println(F("Mode secteur, action due"));
      #endif
    }
    
  } else {
    // Comportement sur batterie
    // L'idee est de tenter d'entrer en sleep aussi vite que possible.
    // Certaines operations d'entretien sont quand meme faites et on laisse l'antenne ouverte
    // pendant ce temps, ca donne le temps de recevoir des commandes.

    // Si un message est recu pendant que l'antenne fonctionne, on agit immediatement
    // On renouvelle le IV avant d'entrer en mode sleep (generement 1 seconde de travail en background)
    // On attend le ACK, jusqu'a concurrence de ATTENTE_BATTERIE
    
    infoReseau.bypassSleep |= messageRecu || infoReseau.lectureDue;

    if( ! infoReseau.bypassSleep ) {

      // On peut entrer en sleep des que le ACK est recu
      if( ! prot9.isAckRecu() && millis() - infoReseau.derniereAction < ATTENTE_BATTERIE) {
        // Attendre sleep
        infoReseau.bypassSleep = true;
      }
      
    }
    
  }
 
  if( ! infoReseau.bypassSleep ) {
    // Attendre la prochaine lecture
    attendreProchaineLecture();

    // Clear le flag du watchdog.
    f_wdt = 0;

    // Declencher nouvelle lecture senseur (batterie)
    infoReseau.lectureDue = true;
    
    #ifdef LOGGING_DEV
      Serial.println(F("Mode batterie, action due"));
    #endif
  }

}

bool attenteActionExpiree() {
  return millis() - infoReseau.derniereAction > infoReseau.attenteTransmission;
}

void actionTransmission() {
  // Cleanup flag lecture
  infoReseau.lectureDue = false;

  power.lireVoltageBatterie();

  // S'assurer qu'on est en position de transmettre la lecture
  if( modePairing == PAIRING_SERVEUR_CLE ) {

    #ifdef LOGGING_DEV
      Serial.println(F("Lecture normale"));
    #endif
    
    digitalWrite(PIN_LED, HIGH);  

    // Effectuer lectures
    #if defined(DHTPIN) && defined(DHTTYPE)
      dht.lire();
    #endif
    
    #ifdef BUS_MODE_ONEWIRE
      // Fait la recherche initiale sur le bus
      oneWireHandler.lire();
    #elif defined(BUS_MODE_I2C)
      bmp.lire();
    #endif

    // Transmettre information du senseur
    // RNG.rand(prot9.getIvBuffer(), 16);  // Generer nouveau IV
    bool erreurTransmission = ! transmettrePaquets();

    #ifdef LOGGING_DEV_RADIO
      Serial.print(F("Transmission lectures, erreur: "));
      Serial.println(erreurTransmission);
    #endif

  } else if(modePairing == PAIRING_ADRESSE_DHCP_ASSIGNEE) {
    // Transmettre l'information de pairing avec la cle publique
    transmettreClePublique();
  }

  // Indique temps de la derniere action
  infoReseau.derniereAction = millis();  
}

// Transmet les paquets. 
bool transmettrePaquets() {
  bool transmissionOk = true;
  
  #ifdef LOGGING_DEV_RADIO
    Serial.println(F("Transmettre lectures"));
  #endif

  if( RNG.available(16) || infoReseau.refreshIv ) {
    #ifdef LOGGING_DEV_RADIO
      Serial.println(F("transmettrePaquets: entropie OK, transmettre nouveau IV"));
    #endif 

    // Debut de la transmission, envoit 2 paquets (0 et IV)
    prot9.transmettrePaquet0(MSG_TYPE_ECHANGE_IV);
    if( prot9.isTransmissionOk() ) {
      // IV transmis, ok confirme avec paquet fin
      // Paquet de fin avec tag (hash)
      if( prot9.transmettrePaquetFin(2) ) {
        infoReseau.refreshIv = false; // IV pret
      } else {
        // Erreur - le serveur n'a pas confirme qu'on peut utiliser le nouveau IV...
        #ifdef LOGGING_DEV_RADIO
          Serial.println(F("transmettrePaquets: nouveau IV utilise sans confirmation finale..."));
        #endif
        infoReseau.refreshIv = true;
        transmissionOk = false;
      }
    }
  } else {
    // S'assurer de marquer le ACK comme recu - sinon pas de deep sleep
    prot9.setAckRecu();
  }

  #if defined(DHTPIN) && defined(DHTTYPE)
    transmissionOk &= prot9.transmettreLectureTHAntennePower(&dht, &prot9, &power);
  #endif 

  // Adafruit BMP
  #ifdef BUS_MODE_I2C
    transmissionOk &= prot9.transmettreLectureTPAntennePower(&bmp, &prot9, &power);
  #endif

  // Dalsemi OneWire (1W)
  #ifdef BUS_MODE_ONEWIRE
    transmissionOk &= prot9.transmettreLectureOneWire(&oneWireHandler);
  #endif

  return transmissionOk;
  

//  // DHT
//  #if defined(DHTPIN) && defined(DHTTYPE)
//    if( ! prot9.transmettrePaquetLectureTH(compteurPaquet++, &dht) ) return false;
//  #endif 
//
//  // Adafruit BMP
//  #ifdef BUS_MODE_I2C
//    if( ! prot9.transmettrePaquetLectureTP(compteurPaquet++, &bmp) ) return false;
//  #endif
//
//  // Power info
//  prot9.transmettrePaquetLecturePower(compteurPaquet++, &power);
//
//  // Antenne info
//  prot9.transmettrePaquetLectureAntenne(compteurPaquet++, &prot9);
//
//  // Paquet de fin avec tag (hash)
//  prot9.transmettrePaquetFin(compteurPaquet);
//
//  return prot9.isTransmissionOk();
}

bool transmettreClePublique() {
  #ifdef LOGGING_DEV_RADIO
    Serial.println(F("Transmettre cle publique"));
  #endif
  
  // Debut de la transmission
  byte compteurPaquet = prot9.transmettrePaquet0(MSG_TYPE_NOUVELLE_CLE);
  if(compteurPaquet == 0) return false;

  if( ! prot9.transmettrePaquetsClePublique(compteurPaquet) ) return false;
  compteurPaquet += 2; // 2 paquets transmis

  #ifdef LOGGING_DEV_RADIO
    Serial.println(F("Transmettre paquet fin"));
  #endif

  // Paquet de fin avec IV et tag
  prot9.transmettrePaquetFin(compteurPaquet);

  #ifdef LOGGING_DEV_RADIO
    Serial.println(F("Paquet fin transmis"));
  #endif
}

void ecouterReseau() {
  // Section lecture transmissions du reseau
  ledStatus.throttlePin++;
  if(modePairing != PAIRING_SERVEUR_CLE) {
    ledStatus.pinOutput = 255;  // Max pour indiquer que le senseur n'est pas pret a transmettre
  } else if(!ledStatus.throttlePin) {
    if(ledStatus.directionPin) ledStatus.pinOutput++;
    else ledStatus.pinOutput--;
    if(ledStatus.pinOutput == 10) ledStatus.directionPin = true;
    if(ledStatus.pinOutput == 200) ledStatus.directionPin = false;
  }
  
  analogWrite(PIN_LED, ledStatus.pinOutput);

  switch(modePairing) {
  case PAIRING_ADRESSE_DHCP_ASSIGNEE:
    if( ! infoReseau.lectureDue ) {
      // Retransmettre cle publique
      declencherAction(2000, 3000);
    }
  case PAIRING_CLE_PRIVEE_PRETE:
  case PAIRING_SERVEUR_CLE:
    networkProcess();
    break;
    
  case PAIRING_PAS_INIT:
  default:
    // Initialiser la cle, besoin de 32 bytes random
    if(prot9.isClePriveePrete() || RNG.available(32)) {

      #ifdef LOGGING_DEV_RADIO
        Serial.println(F("Init cle DH"));
        Serial.flush();
      #endif
      
      prot9.executerDh1();
      modePairing = PAIRING_CLE_PRIVEE_PRETE;

      #ifdef LOGGING_DEV_RADIO
        Serial.print(F("Cle publique : "));
        printArray(prot9.getCleBuffer(), 32);
        Serial.println();
      #endif

      // On a une cle privee, on prepare la radio a ecouter le beacon DHCP
      activerRadioDhcp();
    } else {
      infoReseau.bypassSleep = true;  // Doit generer contenu aleatoire
    }
  }
  
}

void declencherAction(int rangeMin, int rangeMax) {
  infoReseau.attenteTransmission = random(rangeMin, rangeMax);
  #ifdef LOGGING_DEV
    Serial.print(F("Action declenchee, attente "));
    Serial.println(infoReseau.attenteTransmission);
  #endif
  
  infoReseau.derniereAction = millis();
  infoReseau.lectureDue = true;
}

bool networkProcess() {

  while(radio.available()) {
    messageRecu = false;

    byte buffer[32];
    uint16_t typePaquet = prot9.recevoirPaquet((byte*)&buffer, sizeof(buffer));
    if(typePaquet == MSG_TYPE_PAQUET_INCONNU) {
      break;  // Abort, voir s'il y a d'autres donnees a lire
    }

    #ifdef LOGGING_DEV_RADIO
      Serial.print(F("Recu message "));
      printArray((byte*)&buffer, 32);
      Serial.println();
    #endif

    if(modePairing == PAIRING_CLE_PRIVEE_PRETE) {
      
      // On attend une confirmation de l'adresse DHCP
      if(!assignerAdresseDHCPRecue((byte*)&buffer)) {
        break;
      }
      
    } else if(modePairing == PAIRING_ADRESSE_DHCP_ASSIGNEE) {
      
      // On attend une reponse avec la cle publique du serveur
      recevoirClePubliqueServeur((byte*)&buffer);
      
    } else {

      switch(typePaquet) {
        case MSG_TYPE_REPONSE_ACK:
          #ifdef LOGGING_DEV_RADIO
            Serial.println(F("ACK recu"));
          #endif
          break; // Rien a faire, deja gere par la librairie

        default:
          #ifdef LOGGING_DEV_RADIO
            Serial.print(F("Message non gere, type : "));
            Serial.println(typePaquet);
          #endif
          break;
      }
      
    }

  }
  messageRecu = false;

  if( modePairing != PAIRING_PAS_INIT && prot9.nombreCyclesAbortConsecutifs() > PAQUETS_ECHECS_CONSECUTIFS_RESET ) {
    // Le serveur ne repond plus, possiblement parce qu'il n'a plus la cle. 
    // On reinit le processus (DHCP, echange cle)
    modePairing = PAIRING_PAS_INIT;
    prot9.resetNombreCyclesAbortConsecutifs();

    #ifdef LOGGING_DEV_RADIO
      Serial.println(F("Reinit pairing - serveur ne repond pas"));
    #endif
  }
  
  return true;
  
}

//void genererIv() {
//  // En mode alimentationSecteur, il devrait generalement y avoir assez d'info (16 bytes)
//  // Sur batterie, le generateur prend trop de temps (~1 byte/sec a 8MHz). On va utiliser le 
//  // generateur avec une quantite d'entropie insuffisante, mais regulierement remonter l'entropie.
//  bool genererIv = false;
//  
//  if(RNG.available(16)) {
//    
//    genererIv = true;
//    
//    #ifdef LOGGING_DEV_RADIO
//      Serial.println(F("Nouveau IV random"));
//    #endif
//    
//  } else if( RNG.available(1) ) {
//    // On n'a pas assez de donnees pour un nombre random de 16 bytes
//    // On genere quand meme un IV de 16 bytes mais une fois de temps en temps on
//    // laisse l'entropie remonter
//    
//    genererIv = true;
//    
//    #ifdef LOGGING_DEV_RADIO
//      Serial.println(F("Nouveau IV (entropie faible)"));
//    #endif
//    
//  }
//
//  if(genererIv) {
//    // IV utilise et RNG a suffisamment d'entropie, extraire un IV de 16 bytes
//    // RNG.rand(prot9.getIvBuffer(), 16);
//
//    infoReseau.ivUsed = false;
//  }
//
//}

void recevoirClePubliqueServeur(byte* data) {
  uint16_t typeMessage;
  memcpy(&typeMessage, data + 1, 2);

  #ifdef LOGGING_DEV_RADIO
    Serial.print(F("Reception cle publique, type message : "));
    Serial.println(typeMessage);
  #endif

  if(typeMessage == MSG_TYPE_CLE_DISTANTE_1) {
    
    // Copier la premiere partie de la cle (28 bytes), reset reste du buffer
    memcpy(prot9.getCleBuffer(), data + 4, 28);
    memset(prot9.getCleBuffer() + 28, 0x0, 4);
    
  } else if(typeMessage == MSG_TYPE_CLE_DISTANTE_2) {
    
    // S'assurer qu'on a recu la premiere partie de la cle - verifier que la
    // fin du buffer de cle est 0x0
    //if(memcmp(prot9.getCleBuffer() + 28, 0x0, 4) == 0) {
      // Copier la deuxieme partie de la cle
      memcpy(prot9.getCleBuffer() + 28, data + 4, 4);

      #ifdef LOGGING_DEV_RADIO
        Serial.print(F("Cle publique recue : "));
        printArray(prot9.getCleBuffer(), 32);

        // Serial.print(F("Cle secrete : "));
      #endif
      
      if ( prot9.executerDh2() ) {
        // Activer le cryptage pour tous les messages subsequents
        prot9.activerCryptage();
        #ifdef LOGGING_DEV
          printArray(prot9.getCleBuffer(), 32);
        #endif
  
        modePairing = PAIRING_SERVEUR_CLE;
        infoReseau.lectureDue = true;  // Preparer la premiere transmission de contenu
        infoReseau.derniereAction = millis();
        infoReseau.attenteTransmission = random(200, 250);
      } else {
        #ifdef LOGGING_DEV_RADIO
          Serial.println(F("DH2 Erreur decodage cle, reset"));
        #endif
        modePairing = PAIRING_ADRESSE_DHCP_ASSIGNEE;
      }

  }
}

void attendreProchaineLecture() {

  #if defined(LOGGING_DEV) || defined(LOGGING_DEV_RADIO)
    Serial.println(F("Sleep"));
    Serial.flush();
  #endif
  
  if(messageRecu) return;

  switch(modePairing) {
  case PAIRING_ADRESSE_DHCP_ASSIGNEE:
  case PAIRING_CLE_PRIVEE_PRETE:
    power.singleCycleSleep();
    if(messageRecu) return;  
  default:
    break;
  }

  // Fermer la radio, entrer en deep sleep
  radio.powerDown();
  digitalWrite(PIN_LED, LOW);
  if( prot9.isAckRecu() ) {
    power.deepSleep(&messageRecu);
  } else {
    power.singleCycleSleep();
  }
  digitalWrite(PIN_LED, HIGH);

  radio.powerUp();
  radio.startListening();

}

void chargerConfiguration() {

  // UUID
  byte uuid[16];
  EEPROM.get(EEPROM_UUID, uuid);
  Serial.print(F("UUID senseur "));
  printArray(uuid, 16);

  // Initialiser generateur nombre aleatoire avec UUID
  RNG.begin(uuid);

  unsigned long seed;
  memcpy(&seed, uuid, 4);  // Utiliser 4 premiers bytes du UUID pour seeder pseudo-random gen
  randomSeed(seed);

  // Mode pairing
  EEPROM.get(EEPROM_MODE_PAIRING, modePairing);

  #ifdef LOGGING_DEV_RADIO
    Serial.print(F("Mode pairing "));
    printHex(modePairing);
    Serial.println();
  #endif

}

void checkRadio(void) {
  #ifdef LOGGING_DEV_RADIO
    Serial.println(F("Message recu IRQ"));
  #endif
  messageRecu = true;
}

bool assignerAdresseDHCPRecue(byte* data) {
  uint16_t typeMessage = data[1];
  byte adresseNoeud[5];  // 4 premiers bytes = reseau, 5e est nodeId
  byte nodeIdReserve=0;

  #ifdef LOGGING_DEV_RADIO
    Serial.print(F("Type message "));
    Serial.println(typeMessage);
  #endif
  
  if(typeMessage == MSG_TYPE_BEACON_DHCP) {
    byte adresseServeur[5];
    prot9.lireBeaconDhcp(data, (byte*)&adresseServeur);
    adresseServeur[0] = 0; adresseServeur[1] = 0;
    
    // Transmettre demande adresse au serveur
    #ifdef LOGGING_DEV
      Serial.print(F("Adresse serveur "));
      for(byte h=0; h<5; h++){
        printHex(adresseServeur[h]);
      }
      Serial.println();
    #endif

    radio.openWritingPipe(adresseServeur);
    
    #ifdef LOGGING_DEV_RADIO
      radio.printDetails();
    #endif
    
    radio.stopListening();
    // Attendre quelques millisecondes random pour donner la chance
    // aux divers appareils de faire leur demande.
    byte attente = random(4, 64);
    delay(attente);
    bool transmisOk = prot9.transmettreRequeteDhcp();
    radio.startListening();
    infoReseau.ecouterBeacon = false;

    #ifdef LOGGING_DEV_RADIO
      if(transmisOk) {
        Serial.println(F("DHCP transmise"));
      } else {
        Serial.println(F("DHCP echec"));
      }
    #endif
    
  } else if(typeMessage == MSG_TYPE_REPONSE_DHCP) {
    nodeIdReserve = prot9.lireReponseDhcp(data, (byte*)&adresseNoeud);
    
    #ifdef LOGGING_DEV_RADIO
      Serial.print(F("Node ID Reserve: "));
      Serial.println(nodeIdReserve);
    #endif

    if(nodeIdReserve) {
      modePairing = PAIRING_ADRESSE_DHCP_ASSIGNEE;

      infoReseau.nodeId = nodeIdReserve;  // Modification du node Id interne
      // Changement de nodeId

      #ifdef LOGGING_DEV_RADIO
        Serial.print(F("Adresse reseau "));
        printArray(adresseNoeud, 5);
        Serial.println();
      #endif
      
      // Ajuster l'adresse reseau
      radio.openReadingPipe(1, adresseNoeud);

      // Ajouter adresse ecoute broadcast (nodeId = 0xff)
      adresseNoeud[0] = 0xff;
      radio.openReadingPipe(2, adresseNoeud);
      
      infoReseau.lectureDue = true;
      return false;  // Va indiquer qu'on veut transmettre immediatement
    }
  } else {
    #ifdef LOGGING_DEV_RADIO
      Serial.print(F("Type message inconnu: "));
      Serial.println(typeMessage);
    #endif
  }

  return true;
}

void printArray(byte* liste, int len) {
  byte valeur = 0;
  for(int i=0; i<len; i++){
    valeur = liste[i];
    printHex(valeur);
  }

  Serial.println();  
}

void printHex(byte val) {
  if(val < 16) {
    Serial.print(val < 16 ? "0" : "");
  }
  Serial.print(val, HEX);
}

//#ifdef LOGGING_DEV
//void afficherTailleObjets() {
//  
//  Serial.print(F("Taille radio : "));
//  Serial.println(sizeof(radio));
//  
//  Serial.print(F("Taille prot9 : "));
//  Serial.println(sizeof(prot9));
//
//  #if defined(DHTPIN) && defined(DHTTYPE)
//    Serial.print(F("Taille DHT : "));
//    Serial.println(sizeof(dht));
//  #endif
//
//  #ifdef BUS_MODE_I2C
//    Serial.print(F("Taille BMP : "));
//    Serial.println(sizeof(bmp));
//  #endif
//
//  #ifdef BUS_MODE_ONEWIRE
//    Serial.print(F("Taille 1W : "));
//    Serial.println(sizeof(oneWireHandler));
//  #endif
//
//  Serial.print(F("Taille noise1 : "));
//  Serial.println(sizeof(noise1));
//
//  Serial.print(F("Taille RNG : "));
//  Serial.println(sizeof(RNG));
//  
//  Serial.print(F("Taille power : "));
//  Serial.println(sizeof(power));
//  
//}
//#endif
