//#include <printf.h>
#include <EEPROM.h>
#include "Config.h"
#include "Power.h"
#include "MGAppareilsProt.h"

#include <SPI.h>
#include <RF24.h>
// #include <RF24Network.h>
// #include <RF24Mesh_config.h>
// #include <RF24Mesh.h>

#include <printf.h>


// ***********************************

// Configuration

// UUID, va etre charge a partir de la memoire EEPROM
byte uuid[16];
byte receivingPipe[] = {0x0,0x0,0x0,0x0,0x0};

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
// RF24Network network(radio);
// RF24Mesh mesh(radio,network);

byte nodeId = NODE_ID_DEFAULT;

// Renouveller nodeId. Toujours verifier nodeId avec DHCP au demarrage
bool doitVerifierAdresseDhcp = true;

// Flag qui indique un echec de transmission
// bool erreurMesh = false;
bool transmissionOk = false;

// Helper conversion de donnees avec protocole Version 7
// MGProtocoleV7 prot7(uuid, &mesh);
MGProtocoleV8 prot8(uuid, &radio, &nodeId);

// Power management
ArduinoPower power;
volatile int f_wdt=1;

ISR(WDT_vect)
{
  if(f_wdt == 0)
  {
    f_wdt=1;
  }
}

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
  // radio.enableDynamicPayloads();
  radio.setAutoAck(true);
  radio.setRetries(15, 1);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setPALevel(RF24_PA_MAX);
  // radio.setPALevel(RF24_PA_LOW);

  // Ouvrir writing pipe vers serveur
  radio.openWritingPipe(SERVER_ADDR);

  // Ouvrir reading pipe sur le canal "uuid[0:3] nodeId"
  radio.openReadingPipe(1,pipes[1]);

  Serial.print(F("Ouverture radio avec nodeId "));
  Serial.println(nodeId);
  radio.printDetails();
  radio.startListening();

  Serial.println(F("Radio initialisee"));

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

void loop() {

  // Lecture reseau, ecouter avant de transmettre
  networkMaintenance();

  power.lireVoltageBatterie();

  // S'assurer que DHCP a ete execute correctement par networkMaintenance()
  if( ! doitVerifierAdresseDhcp ) {
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
  
    // Lecture reseau
    if ( ! erreurTransmission ) {
      erreurTransmission = ecouterReseau();
    }
  } 

  if(!power.isAlimentationSecteur()) {
    // Attendre la prochaine lecture
    attendreProchaineLecture();

    // Clear le flag du watchdog.
    f_wdt = 0;
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

// Entretien reseau. Retourne false si le reseau est en erreur.
bool networkMaintenance() {

  if(radio.failureDetected) {
    Serial.print(F("Hardware failure detected "));
    Serial.println(radio.failureDetected);
    radio.failureDetected = 0; // Reset flag
  }

  // Section DHCP
  if(doitVerifierAdresseDhcp) {
    // Le senseur vient d'etre initialise, il faut demander un nouveau nodeId au serveur
    prot8.transmettreRequeteDhcp();
    Serial.println(F("Requete DHCP transmise"));
  }

}

bool ecouterReseau() {

  byte pinOutput = 200;
  byte pintThrottle = 0;
  bool directionPin = false;

  // Section lecture transmissions du reseau
  long timer = millis();
  uint16_t attente = 500;  // 500ms sur batterie
  if(power.isAlimentationSecteur()) {
    // Mode alimentation secteur
    // On reste en ecoute durant l'equivalent du mode sleep.
    attente = 8000 * CYCLES_SOMMEIL;
  }

  while(millis() - timer < attente) {
    if(pintThrottle++ == 4) {
      pintThrottle = 0;
      if(directionPin) pinOutput++;
      else pinOutput--;
      if(pinOutput == 20) directionPin = true;
      if(pinOutput == 180) directionPin = false;
    }
    
    analogWrite(PIN_LED, pinOutput);
    // mesh.update();
    bool resultatLecture = networkProcess();
    if(!resultatLecture) {
      return false;
    }
    
  }

  digitalWrite(PIN_LED, HIGH);  
  return true;
}

bool networkProcess() {

  while(radio.available()){

    byte data[32];
    byte nodeIdReserve=0;
    byte[] adresseNoeud = {0x0, 0x0, 0x0, 0x0, 0x0};  // 4 premiers bytes = reseau, 5e est nodeId

    // S'assurer que le paquet est de la bonne version
    if(data[0] == VERSION_PROTOCOLE) {
      nodeIdReserve = prot8.lireReponseDhcp((byte*)&dat, (byte*)&adresseNoeud);
      if(nodeIdReserve) {
          nodeId = nodeIdReserve;  // Modification du node Id interne
//          mesh.setNodeID(nodeIdReserve);
//          EEPROM.update(ADDRESS_SENSEUR, nodeId);
//          mesh.renewAddress(1000);
          doitVerifierAdresseDhcp = false;

          // Changement de nodeId
          Serial.print(F("Nouveau node id recu de DHCP: "));
          Serial.println(nodeIdReserve);
      }
    }

//    switch(header.type){
//      // Display the incoming millis() values from the sensor nodes
//      case 'd': // Reponse DHCP
//        network.read(header,&dat,sizeof(dat));
//        nodeIdReserve = prot7.lireReponseDhcp((byte*)&dat);
//        if(nodeIdReserve > 1) {
//          nodeId = nodeIdReserve;  // Modification du node Id interne
////          mesh.setNodeID(nodeIdReserve);
////          EEPROM.update(ADDRESS_SENSEUR, nodeId);
////          mesh.renewAddress(1000);
//          doitVerifierAdresseDhcp = false;
//
//          // Changement de nodeId
//          Serial.print(F("Nouveau node id recu de DHCP: "));
//          Serial.println(nodeIdReserve);
//        }
//        break;
//      default: network.read(header,0,0); Serial.println(header.type);break;

//    }
//  } else {
//    erreurMesh = ! mesh.checkConnection();
//    if( erreurMesh ) {
//      return false;  // Arrete traitement, va reconnecter et recommencer dans loop
//    }
  }
  return true;
  
}

void attendreProchaineLecture() {

  Serial.println(F("Sleep"));
  delay(20); // Finir transmettre Serial
  
  // Power down the radio.  Note that the radio will get powered back up
  // on the next write() call.
  radio.powerDown();

  digitalWrite(PIN_LED, LOW);
  power.deepSleep();
  digitalWrite(PIN_LED, HIGH);

  radio.powerUp();
  radio.startListening();

}

void chargerConfiguration() {

  // configuration.lire_configuration();
  
  EEPROM.get(ADDRESS_SENSEUR, nodeId);
  EEPROM.get(ADDRESS_UUID, uuid);
  // EEPROM.get(ADDRESS_BATT_PIN, battery_pin);  

  Serial.print(F("NodeID senseur "));
  Serial.println(nodeId);
  Serial.print(F("UUID senseur "));
  printArray(uuid, 16);
}

void printArray(byte* liste, int len) {
  byte valeur = 0;
  for(int i=0; i<len; i++){
    valeur = liste[i];
    printHex(valeur);
  }

  Serial.println();  
}

void printHex(uint8_t num) {
  char hexCar[2];

  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
}
