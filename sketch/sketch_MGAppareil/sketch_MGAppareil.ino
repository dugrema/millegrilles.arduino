//#include <printf.h>

#include <EEPROM.h>

#include <RF24Mesh_config.h>
#include <RF24Mesh.h>
#include <RF24_config.h>

#include "Config.h"

#include "Power.h"
#include "MGAppareilsProt.h"

// Librairies de senseurs
#include "dht.h"
#include "AdafruitSensors.h"
#include "OneWireHandler.h"

#define DEBUG Serial.print
#define DEBUGLN Serial.println

// ***********************************

// Configuration
// manual_config configuration;

// UUID, va etre charge a partir de la memoire EEPROM
byte uuid[16];

// *****************
// Senseurs
// *****************

// DHT
MilleGrillesDHT dht;
boolean bmpActif = false;

// Adafruit BMP
MilleGrillesAdafruitSensors bmp;

// OneWire
OneWireHandler oneWireHandler;

// *****************

// Radio nRF24L01 sur pins CE=7, CSN=8
RF24 radio(RF24_CE_PIN,RF24_CSN_PIN);
RF24Network network(radio);
RF24Mesh mesh(radio,network);

byte nodeId = NODE_ID_DEFAULT;
// Toujours verifier nodeId avec DHCP au demarrage
bool doitVerifierAdresseDhcp = true;

MGProtocoleV7 prot7(uuid, &mesh);

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

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  
  Serial.begin(9600);

  chargerConfiguration();

  // Preparation senseurs
  bmp.begin();

  // Ouverture de la radio, mesh configuration
  if(nodeId == 0 || nodeId == NODE_ID_DEFAULT) {
    // ID 0 est reserve au master
    // Seeder avec le premier byte du UUID.
    // Le serveur dhcp va decider si le noeud peut le garder.
    if(uuid[0] != 0x0) {
      nodeId = uuid[0];
    } else {
      // Le premier byte du UUID est 0, cet ID est reserve au master. 
      // On reste avec default.
      nodeId = NODE_ID_DEFAULT;
    }
  }
  mesh.setNodeID(nodeId);
  mesh.begin(CANAL_MESH);

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

  DEBUGLN(F("Pret"));

  digitalWrite(PIN_LED, LOW);
  delay(200);
  digitalWrite(PIN_LED, HIGH);
}

void loop() {

  Serial.println(F("Loop"));

  // Lecture reseau
  networkMaintenance();

  /* Clear le flag du watchdog. */
  f_wdt = 0;

  // Effectuer lectures
  dht.lire();
  bmp.lire();
  power.lireVoltageBatterie();

  // Transmettre information du senseur
  transmettrePaquets();

  // Lecture reseau
  networkMaintenance();

  // Attendre la prochaine lecture
  attendreProchaineLecture();
}

void transmettrePaquets() {

  prot7.transmettrePaquet0(MSG_TYPE_LECTURES_COMBINEES, 4);

  byte compteurPaquet = 1;
  prot7.transmettrePaquetLectureTH(compteurPaquet++, &dht);
  prot7.transmettrePaquetLectureTP(compteurPaquet++, &bmp);
  prot7.transmettrePaquetLecturePower(compteurPaquet++, &power);
  
}

void networkMaintenance() {

  if(doitVerifierAdresseDhcp) {
    // Le senseur vient d'etre initialise, il faut demander un nouveau nodeId au serveur
    prot7.transmettreRequeteDhcp();
    Serial.println(F("Requete DHCP transmise"));
  }
  
  long timer = millis();

  Serial.println(F("Debut verif maintenance"));

  while(millis() - timer < 1000) {
    mesh.update();
    
    if(network.available()){
      RF24NetworkHeader header;
      network.peek(header);
      
      byte dat[64];
      byte fromNodeId=0;
      byte nodeIdReserve=0;

      switch(header.type){
        // Display the incoming millis() values from the sensor nodes
        case 'd': // Reponse DHCP
          network.read(header,&dat,sizeof(dat)); 
          nodeIdReserve = prot7.lireReponseDhcp((byte*)&dat);
          if(nodeIdReserve > 1) {
            nodeId = nodeIdReserve;  // Modification du node Id interne
            mesh.setNodeID(nodeIdReserve);
            EEPROM.update(ADDRESS_SENSEUR, nodeId);
            mesh.renewAddress(1000);
            doitVerifierAdresseDhcp = false;
            
            // Changement de nodeId
            Serial.print(F("Nouveau node id recu de DHCP: "));
            Serial.println(nodeIdReserve);
          }
          break;
        default: network.read(header,0,0); Serial.println(header.type);break;
        
      }
    } else {
      if( ! mesh.checkConnection() ) {
        mesh.renewAddress(500);
      }
    }
  }
  Serial.println(F("Fin verif maintenance"));
}


void attendreProchaineLecture() {
  
  // Power down the radio.  Note that the radio will get powered back up
  // on the next write() call.
  // mesh.releaseAddress();
  radio.powerDown();

  digitalWrite(PIN_LED, LOW);
  power.deepSleep();
  digitalWrite(PIN_LED, HIGH);

  radio.startListening();
  mesh.renewAddress(1000);

}

void lireVoltageBatterie() {
  
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

