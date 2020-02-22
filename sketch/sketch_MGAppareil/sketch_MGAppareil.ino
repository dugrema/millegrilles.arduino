//#include <printf.h>
#include <EEPROM.h>
#include "Config.h"
#include "Power.h"
#include "MGAppareilsProt.h"

#include <SPI.h>
#include <RF24.h>

#include <RNG.h>
#include <TransistorNoiseSource.h>

#include <printf.h>


// ***********************************

// Configuration

// UUID, va etre charge a partir de la memoire EEPROM
byte uuid[16];
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

TransistorNoiseSource noise1(A1); // Utilise par RNG
TransistorNoiseSource noise2(A2); // Utilise par RNG
TransistorNoiseSource noise3(A3); // Utilise par RNG

// Radio nRF24L01 sur pins CE=7, CSN=8
RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);
byte data[32];  // Data buffer pour reception/emission RF24
volatile bool messageRecu = false;

byte nodeId = NODE_ID_DEFAULT;

// Renouveller nodeId. Toujours verifier nodeId avec DHCP au demarrage
bool doitVerifierAdresseDhcp = true;
bool ecouterBeacon = true;

// Flag qui indique un echec de transmission
bool transmissionOk = false;

// Helper conversion de donnees avec protocole Version 7
MGProtocoleV9 prot9(uuid, &radio, &nodeId);

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
  printf_begin();  // Utilise par radio.printDetails()

  chargerConfiguration();

  // Initialize the random number generator.
  RNG.begin(uuid);
  RNG.setAutoSaveTime(120); // 2 heures

  // Add the noise source to the list of sources known to RNG.
  RNG.addNoiseSource(noise1);

  // Preparation senseurs
  #ifdef BUS_MODE_I2C
    bmp.begin();
  #endif

//  // Ouverture de la radio, mesh configuration
//  if(nodeId == 0 || nodeId == NODE_ID_DEFAULT) {
//    nodeId = NODE_ID_DEFAULT;
//    // ID 0 est reserve au master
//    // Seeder avec le premier byte != 0 du UUID.
//    // Le serveur dhcp va decider si le noeud peut le garder.
//    for(byte i; i<sizeof(uuid); i++) {
//      if(uuid[i]) {
//        nodeId = uuid[i];
//        break;
//      }
//    }
//  }

  Serial.println(F("Setup radio"));
  radio.begin();
  radio.setChannel(RADIO_CANAL);
  // radio.enableDynamicPayloads();
  radio.setAutoAck(true);
  radio.setRetries(15, 1);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  // radio.setPALevel(RF24_PA_MAX);
  radio.setPALevel(RF24_PA_LOW);

//  // Ouvrir reading pipe avec adresse reception beacon DHCP
//  uint64_t addresseDhcp = BROADCAST_DHCP_LISTEN;
//  radio.openReadingPipe(1, addresseDhcp);
//
//  Serial.print(F("Ouverture radio"));
//  radio.printDetails();
//  radio.startListening();
//  radio.maskIRQ(true, true, false); // Masquer TX OK et fail sur IRQ, juste garder payload ready
//  attachInterrupt(digitalPinToInterrupt(3), checkRadio, FALLING);
//
//  Serial.println(F("Radio initialisee, ecoute beacon DHCP"));

  radio.powerDown();  // Garder la radio hors ligne pour generer cle privee (au besoin)

//  /* Clear the reset flag. */
//  MCUSR &= ~(1<<WDRF);
//  /* In order to change WDE or the prescaler, we need to
//   * set WDCE (This will allow updates for 4 clock cycles).
//   */
//  WDTCSR |= (1<<WDCE) | (1<<WDE);
//  /* set new watchdog timeout prescaler value */
//  // WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
//  WDTCSR = 0;
//  /* Enable the WD interrupt (note no reset). */
//  WDTCSR |= _BV(WDIE);

  digitalWrite(PIN_LED, LOW);
  delay(200);
  digitalWrite(PIN_LED, HIGH);

  Serial.print(F("Setup termine apres "));
  Serial.println(millis());
}

void activerRadioDhcp() {
  // Ouvrir reading pipe avec adresse reception beacon DHCP
  radio.powerUp();
  uint64_t addresseDhcp = BROADCAST_DHCP_LISTEN;
  radio.openReadingPipe(1, addresseDhcp);

  Serial.print(F("Ouverture radio"));
  radio.printDetails();
  radio.startListening();
  radio.maskIRQ(true, true, false); // Masquer TX OK et fail sur IRQ, juste garder payload ready
  attachInterrupt(digitalPinToInterrupt(3), checkRadio, FALLING);

  Serial.println(F("Radio initialisee, ecoute beacon DHCP"));
}

bool lectureDue = true;
bool bypassSleep = false;
long derniereAction = 0;  // Utiliser pour derminer prochaine action (sleep ou lecture selon power mode)
const long attenteAlimentationBatterie = 500L; // 5L;
const long attenteAlimentationSecteur = 8000L * CYCLES_SOMMEIL;

void loop() {

  // Perform regular housekeeping on the random number generator.
  RNG.loop();

  // S'assurer que DHCP a ete execute correctement avant de transmettre
  // On n'effectue pas de lecture sur reception de commande
  if( lectureDue && ! messageRecu ) {

    // Serial.println(F("Debut traitement lecture"));

    power.lireVoltageBatterie();

    // S'assurer qu'on est en position de transmettre la lecture
    if( ! doitVerifierAdresseDhcp ) {
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
      bool erreurTransmission = ! transmettrePaquets();
      Serial.print(F("Transmission lectures, erreur: "));
      Serial.println(erreurTransmission);
    } 

    // Cleanup flag lecture
    lectureDue = false;

    // Indique temps de la derniere action
    derniereAction = millis();
  }

  bypassSleep = false;

  // Ecouter reseau
  ecouterReseau();

  // Verifier si on peut entrer en mode sleep
  if(!power.isAlimentationSecteur()) {
    // Comportement sur alimentation secteur
    // On ne sleep jamais sur alimentation secteur
    bypassSleep = true;

    long attente;
    if(transmissionOk) {
      attente = attenteAlimentationSecteur;
    } else {
      attente = 8000L;  // Retransmettre plus rapidement
    }
    if(millis() - derniereAction > attente) {
      // Declencher nouvelle lecture senseur (alimentation secteur)
      lectureDue = true;
      Serial.println(F("Mode secteur, lecture due"));
    }
    
  } else {
    // Comportement sur batterie
    // Sur batterie, on agit immediatement si un message est Recu
  
    bypassSleep |= messageRecu || lectureDue;

    if( ! bypassSleep && millis() - derniereAction < attenteAlimentationBatterie) {
      // Attendre sleep
      bypassSleep = true;
    }
    
  }
 
  if( ! bypassSleep) {
    // Attendre la prochaine lecture
    attendreProchaineLecture();

    // Clear le flag du watchdog.
    f_wdt = 0;

    // Declencher nouvelle lecture senseur (batterie)
    lectureDue = true;
    Serial.println(F("Mode batterie, lecture due"));
  }

}

// Transmet les paquets. 
bool transmettrePaquets() {

  byte nombrePaquets = 2; // Init a 2, pour paquet 0 et paquet power.
  #ifdef BUS_MODE_ONEWIRE
    // OneWire peut avoir un nombre variable de senseurs
    nombrePaquets += oneWireHandler.nombreSenseurs();
  #endif

  #ifdef BUS_MODE_I2C
    nombrePaquets++;
  #endif

  #if defined(DHTPIN) && defined(DHTTYPE)
    nombrePaquets++;
  #endif

  // Debut de la transmission
  transmissionOk = prot9.transmettrePaquet0(MSG_TYPE_LECTURES_COMBINEES, nombrePaquets);
  if(!transmissionOk) return false;

  byte compteurPaquet = 1;  // Fourni le numero du paquet courant

  // Dalsemi OneWire (1W)
  #ifdef BUS_MODE_ONEWIRE
    transmissionOk = prot9.transmettrePaquetLectureOneWire(compteurPaquet++, &oneWireHandler);
    if(!transmissionOk) return false;
  #endif

  // DHT
  #if defined(DHTPIN) && defined(DHTTYPE)
    transmissionOk = prot9.transmettrePaquetLectureTH(compteurPaquet++, &dht);
    if(!transmissionOk) return false;
  #endif 

  // Adafruit BMP
  #ifdef BUS_MODE_I2C
    transmissionOk = prot9.transmettrePaquetLectureTP(compteurPaquet++, &bmp);
    if(!transmissionOk) return false;
  #endif

  // Power info
  transmissionOk = prot9.transmettrePaquetLecturePower(compteurPaquet++, &power);

  return transmissionOk;
}

byte pinOutput = 200;
bool directionPin = false;
byte pintThrottle = 0;

void ecouterReseau() {
  // Section lecture transmissions du reseau

  if(doitVerifierAdresseDhcp) {
    pinOutput = 255;  // Max pour indiquer que le senseur n'est pas pret
  } else if(pintThrottle++ == 4) {
    if(directionPin) pinOutput++;
    else pinOutput--;
    if(pinOutput == 20) directionPin = true;
    if(pinOutput == 180) directionPin = false;
  }
  
  analogWrite(PIN_LED, pinOutput);

  switch(modePairing) {
  case PAIRING_SERVEUR_CLE:
  case PAIRING_CLE_PRIVEE_PRETE:
    networkProcess();
    break;
    
  case PAIRING_PAS_INIT:
  default:
    // Initialiser la cle
    if(RNG.available(8)) {
      // wdt_reset();
      // wdt_enable();
      Serial.println(F("Init cle DH"));
      Serial.flush();
      prot9.executerDh1();
      modePairing = PAIRING_CLE_PRIVEE_PRETE;
      Serial.print(F("Cle publique : "));
      printArray(prot9.getCleBuffer(), 32);
      Serial.println();

      // On a une cle privee, on prepare la radio a ecouter le beacon DHCP
      activerRadioDhcp();
    } else {
      bypassSleep = true;  // Doit generer contenu aleatoire
    }
  }
  
}

bool networkProcess() {

  while(radio.available()){
    messageRecu = false;

    radio.read(&data, sizeof(data));

    Serial.print(F("Recu message "));
    for(byte i=0; i<32; i++){
      printHex(data[i]);
    }
    Serial.println("");

    // S'assurer que le paquet est de la bonne version
    if(data[0] == VERSION_PROTOCOLE) {
      uint16_t typeMessage = data[1];
      
      // memcpy(&typeMessage, (&data)+1, 2); // Lire les bytes [1:2], type message

      if(doitVerifierAdresseDhcp) {
        if(!assignerAdresseDHCPRecue()) {
          return false;
        }
      } else {
        Serial.print(F("Message non gere, type"));
        Serial.println(typeMessage);
      }

    }
  }
  messageRecu = false;
  
  return true;
  
}

void attendreProchaineLecture() {

  Serial.println(F("Sleep"));
  delay(10); // Finir transmettre Serial, radio (5ms minimum)
  if(messageRecu) return;
  
  if( doitVerifierAdresseDhcp ) {
    // Effectuer un seul cycle de sleep avec la radio ouverte
    // Le IRQ reveille le controleur pour recevoir une commande
    power.singleCycleSleep();
    if(messageRecu) return;  
  }

  // Fermer la radio, entrer en deep sleep
  radio.powerDown();
  digitalWrite(PIN_LED, LOW);
  if(transmissionOk) {
    power.deepSleep(&messageRecu);
  } else {
    power.singleCycleSleep();
  }
  digitalWrite(PIN_LED, HIGH);

  // radio.setPALevel(RF24_PA_LOW);
  radio.powerUp();
  radio.startListening();

}

void chargerConfiguration() {

  // UUID
  EEPROM.get(EEPROM_UUID, uuid);
  Serial.print(F("UUID senseur "));
  printArray(uuid, 16);

  // Mode pairing
  EEPROM.get(EEPROM_MODE_PAIRING, modePairing);
  Serial.print(F("Mode pairing "));
  printHex(modePairing);
  Serial.println();

  if(modePairing == PAIRING_SERVEUR_CLE) {
    // Charger la cle secrete et l'adresse du serveur
    byte bufferCle[32];
    EEPROM.get(EEPROM_MODE_PAIRING, *bufferCle);
    memset(prot9.getCleBuffer(), &bufferCle, 32);
    Serial.print(F("Cle secrete "));
    printArray(prot9.getCleBuffer(), 32);
    Serial.println();

    // A faire, charger adresse serveur
    
  }
}

void checkRadio(void) {
  Serial.println(F("Message recu IRQ"));
  messageRecu = true;
}

bool assignerAdresseDHCPRecue() {
  uint16_t typeMessage = data[1];
  byte adresseNoeud[5];  // 4 premiers bytes = reseau, 5e est nodeId
  byte nodeIdReserve=0;

  Serial.print(F("Type message "));
  Serial.println(typeMessage);
  
  if(typeMessage == MSG_TYPE_BEACON_DHCP) {
    byte adresseServeur[5];
    prot9.lireBeaconDhcp((byte*)&data, (byte*)&adresseServeur);
    adresseServeur[0] = 0; adresseServeur[1] = 0;
    
    // Transmettre demande adresse au serveur
    Serial.print(F("Adresse serveur "));
    for(byte h=0; h<5; h++){
      printHex(adresseServeur[h]);
    }
    Serial.println("");

    radio.openWritingPipe(adresseServeur);
    radio.printDetails();
    radio.stopListening();
    bool transmisOk = prot9.transmettreRequeteDhcp();
    radio.startListening();
    ecouterBeacon = false;
    
    if(transmisOk) {
      Serial.println(F("Requete DHCP transmise"));
    } else {
      Serial.println(F("Requete DHCP echec"));
    }
  } else if(typeMessage == MSG_TYPE_REPONSE_DHCP) {
    Serial.print(F("Node ID Reserve: "));
    nodeIdReserve = prot9.lireReponseDhcp((byte*)&data, (byte*)&adresseNoeud);
    Serial.println(nodeIdReserve);

    if(nodeIdReserve) {
      nodeId = nodeIdReserve;  // Modification du node Id interne
      doitVerifierAdresseDhcp = false;
      // Changement de nodeId
      Serial.print(F("Adresse reseau "));
      printArray(adresseNoeud, 5);
      Serial.println();
      
      // Ajuster l'adresse reseau
      radio.openReadingPipe(1, adresseNoeud);

      // Ajouter adresse ecoute broadcast (nodeId = 0xff)
      adresseNoeud[0] = 0xff;
      radio.openReadingPipe(2, adresseNoeud);
      radio.printDetails();

      Serial.println(F("Senseur pret a transmettre"));
      lectureDue = true;
      return false;  // Va indiquer qu'on veut transmettre immediatement
    }
  } else {
    Serial.print(F("Type message inconnu: "));
    Serial.println(typeMessage);
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
