//#include <printf.h>
#include <EEPROM.h>
#include "Config.h"
#include "Power.h"
#include "MGAppareilsProt.h"

#include <SPI.h>
#include <RF24.h>

#include <printf.h>


// ***********************************

// Configuration

// UUID, va etre charge a partir de la memoire EEPROM
byte uuid[16];

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
MGProtocoleV8 prot8(uuid, &radio, &nodeId);

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

  // Preparation senseurs
  #ifdef BUS_MODE_I2C
    bmp.begin();
  #endif

  // Ouverture de la radio, mesh configuration
  if(nodeId == 0 || nodeId == NODE_ID_DEFAULT) {
    nodeId = NODE_ID_DEFAULT;
    // ID 0 est reserve au master
    // Seeder avec le premier byte != 0 du UUID.
    // Le serveur dhcp va decider si le noeud peut le garder.
    for(byte i; i<sizeof(uuid); i++) {
      if(uuid[i]) {
        nodeId = uuid[i];
        break;
      }
    }
  }

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

  // Ouvrir reading pipe avec adresse reception beacon DHCP
  uint64_t addresseDhcp = BROADCAST_DHCP_LISTEN;
  radio.openReadingPipe(1,addresseDhcp);

  Serial.print(F("Ouverture radio"));
  radio.printDetails();
  radio.startListening();
  radio.maskIRQ(true, true, false); // Masquer TX OK et fail sur IRQ, juste garder payload ready
  attachInterrupt(digitalPinToInterrupt(3), checkRadio, FALLING);

  Serial.println(F("Radio initialisee, ecoute beacon DHCP"));

  /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  /* set new watchdog timeout prescaler value */
  WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);

  digitalWrite(PIN_LED, LOW);
  delay(200);
  digitalWrite(PIN_LED, HIGH);

  Serial.print(F("Setup termine apres "));
  Serial.println(millis());
}

bool lectureDue = true;
bool bypassSleep = false;
long derniereAction = 0;  // Utiliser pour derminer prochaine action (sleep ou lecture selon power mode)
const long attenteAlimentationBatterie = 500; // 5;
const long attenteAlimentationSecteur = 8000; // 8000 * CYCLES_SOMMEIL;

void loop() {

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

  // Ecouter reseau
  bypassSleep = !ecouterReseau();

  // Verifier si on peut entrer en mode sleep
  if(power.isAlimentationSecteur()) {
    // Comportement sur alimentation secteur
    // On ne sleep jamais sur alimentation secteur
    bypassSleep = true;

    if(millis() - derniereAction > attenteAlimentationSecteur) {
      // Declencher nouvelle lecture senseur (alimentation secteur)
      lectureDue = true;
      Serial.println(F("Mode secteur, lecture due"));
    }
    
  } else {
    // Comportement sur batterie
    // Sur batterie, on agit immediatement si un message est Recu
  
    bypassSleep |= messageRecu;

    if( ! bypassSleep && millis() - derniereAction < attenteAlimentationBatterie) {
      // Attendre sleep
      bypassSleep = true;
    }
    
  }
 
  if(!bypassSleep) {
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
  transmissionOk = prot8.transmettrePaquet0(MSG_TYPE_LECTURES_COMBINEES, nombrePaquets);
  if(!transmissionOk) return false;

  byte compteurPaquet = 1;  // Fourni le numero du paquet courant

  // Dalsemi OneWire (1W)
  #ifdef BUS_MODE_ONEWIRE
    transmissionOk = prot8.transmettrePaquetLectureOneWire(compteurPaquet++, &oneWireHandler);
    if(!transmissionOk) return false;
  #endif

  // DHT
  #if defined(DHTPIN) && defined(DHTTYPE)
    transmissionOk = prot8.transmettrePaquetLectureTH(compteurPaquet++, &dht);
    if(!transmissionOk) return false;
  #endif 

  // Adafruit BMP
  #ifdef BUS_MODE_I2C
    transmissionOk = prot8.transmettrePaquetLectureTP(compteurPaquet++, &bmp);
    if(!transmissionOk) return false;
  #endif

  // Power info
  transmissionOk = prot8.transmettrePaquetLecturePower(compteurPaquet++, &power);

  return transmissionOk;
}

byte pinOutput = 200;
bool directionPin = false;
byte pintThrottle = 0;

bool ecouterReseau() {


  // Section lecture transmissions du reseau
//  long timer = millis();
//  uint16_t attente = 5;  // 5ms sur batterie
//  if(power.isAlimentationSecteur()) {
//    // Mode alimentation secteur
//    // On reste en ecoute durant l'equivalent du mode sleep.
//    attente = 8000 * CYCLES_SOMMEIL;
//  }

//  Serial.println(F("Lecture reseau"));
//  while(millis() - timer < attente) {
    if(doitVerifierAdresseDhcp) {
      pinOutput = 255;  // Max pour indiquer que le senseur n'est pas pret
    } else if(pintThrottle++ == 4) {
      if(directionPin) pinOutput++;
      else pinOutput--;
      if(pinOutput == 20) directionPin = true;
      if(pinOutput == 180) directionPin = false;
    }
    
    analogWrite(PIN_LED, pinOutput);
    
    if(!networkProcess()) {
      return false; // Indique qu'on veut recommencer la loop, transmettre lecture
    }
    
//  }
//  Serial.println(F("Fin lecture reseau"));

  // digitalWrite(PIN_LED, HIGH);  
  return true;
}

bool networkProcess() {

  while(radio.available()){
    messageRecu = false;

    radio.read(&data, 32);

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
  delay(10); // Finir transmettre Serial, radio (5ms min)
  
  if( doitVerifierAdresseDhcp ) {
    // Effectuer un seul cycle de sleep avec la radio ouverte
    // Le IRQ reveille le controleur pour recevoir une commande
    power.singleCycleSleep();
    if(messageRecu) return;
  }

  // Fermer la radio, entrer en deep sleep
  radio.powerDown();
  digitalWrite(PIN_LED, LOW);
  power.deepSleep(&messageRecu);
  digitalWrite(PIN_LED, HIGH);

  radio.setPALevel(RF24_PA_LOW);
  radio.powerUp();
  radio.startListening();

}

void chargerConfiguration() {

  // configuration.lire_configuration();
  
  // EEPROM.get(ADDRESS_SENSEUR, nodeId);  // NodeId n'est plus important, c'est le serveur DHCP qui l'assigne
  EEPROM.get(ADDRESS_UUID, uuid);
  // EEPROM.get(ADDRESS_BATT_PIN, battery_pin);  

//  Serial.print(F("NodeID senseur "));
//  Serial.println(nodeId);
  Serial.print(F("UUID senseur "));
  printArray(uuid, 16);
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
    prot8.lireBeaconDhcp((byte*)&data, (byte*)&adresseServeur);
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
    bool transmisOk = prot8.transmettreRequeteDhcp();
    radio.startListening();
    ecouterBeacon = false;
    
    if(transmisOk) {
      Serial.println(F("Requete DHCP transmise"));
    } else {
      Serial.println(F("Requete DHCP echec"));
    }
  } else if(typeMessage == MSG_TYPE_REPONSE_DHCP) {
    Serial.print(F("Node ID Reserve: "));
    nodeIdReserve = prot8.lireReponseDhcp((byte*)&data, (byte*)&adresseNoeud);
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
