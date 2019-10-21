//#include <printf.h>
#include <EEPROM.h>
#include "Config.h"
#include "Power.h"
#include "MGAppareilsProt.h"

#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh_config.h>
#include <RF24Mesh.h>

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
  
  Serial.begin(115200);
  printf_begin();

  chargerConfiguration();

  // Preparation senseurs
  #ifdef BUS_MODE_I2C
    bmp.begin();
  #endif

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

  Serial.print(F("Connexion mesh avec nodeId "));
  mesh.setNodeID(nodeId);
  Serial.println(nodeId);
  mesh.begin(CANAL_MESH, RF24_250KBPS, MESH_RENEW_TIMEOUT);
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);
  Serial.println(F("Connexion mesh complete"));

  radio.setPALevel(RF24_PA_HIGH);
  radio.printDetails();

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

  Serial.println(F("Pret"));

  digitalWrite(PIN_LED, LOW);
  delay(200);
  digitalWrite(PIN_LED, HIGH);
}

void loop() {

  // Lecture reseau
  networkMaintenance();

  /* Clear le flag du watchdog. */
  f_wdt = 0;

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
  
  power.lireVoltageBatterie();

  // Transmettre information du senseur
  transmettrePaquets();

  // Lecture reseau
  networkMaintenance();

  if(!power.isAlimentationSecteur()) {
    // Attendre la prochaine lecture
    attendreProchaineLecture();
  }
}

void transmettrePaquets() {

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
  bool paquetOk = false;
  for(byte essai=0; essai<10 && !paquetOk; essai++) {
    paquetOk = prot7.transmettrePaquet0(MSG_TYPE_LECTURES_COMBINEES, nombrePaquets);
  }

  if(paquetOk) {
    byte compteurPaquet = 1;  // Fourni le numero du paquet courant
    #ifdef BUS_MODE_ONEWIRE
    //while()...
    prot7.transmettrePaquetLectureOneWire(compteurPaquet++, &oneWireHandler);
    #endif
  
    #if defined(DHTPIN) && defined(DHTTYPE)
      prot7.transmettrePaquetLectureTH(compteurPaquet++, &dht);
    #endif 
    
    #ifdef BUS_MODE_I2C
    prot7.transmettrePaquetLectureTP(compteurPaquet++, &bmp);
    #endif
    
    prot7.transmettrePaquetLecturePower(compteurPaquet++, &power);
  }  
}

void networkMaintenance() {

  byte pinOutput = 200;
  byte pintThrottle = 0;
  bool directionPin = false;

  if(doitVerifierAdresseDhcp) {
    // Le senseur vient d'etre initialise, il faut demander un nouveau nodeId au serveur
    prot7.transmettreRequeteDhcp();
    Serial.println(F("Requete DHCP transmise"));
  }
  
  long timer = millis();
  uint16_t attente = 1000;
  if(power.isAlimentationSecteur()) {
    // Mode alimentation secteur - cet appareil devient un node fiable pour le mesh.
    // On reste en ecoute durant l'equivalent du mode sleep.
    attente = 8000 * CYCLES_SOMMEIL;
//    Serial.print("Secteur, on attend sur reseau: ");
//    Serial.println(attente);
  }

//  Serial.print(F("Reserve batterie: "));
//  Serial.println(power.reservePct());
//  Serial.print(F("Sur secteur: "));
//  Serial.println(power.isAlimentationSecteur());

  while(millis() - timer < attente) {
    if(pintThrottle++ == 4) {
      pintThrottle = 0;
      if(directionPin) pinOutput++;
      else pinOutput--;
      if(pinOutput == 20) directionPin = true;
      if(pinOutput == 180) directionPin = false;
    }
    
    analogWrite(PIN_LED, pinOutput);
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
        if( ! mesh.renewAddress(1500) ) {
          // Tenter de redemarrer la radio
          Serial.println(F("Reconnexion au mesh"));
          mesh.begin(CANAL_MESH, RF24_250KBPS, MESH_RENEW_TIMEOUT);
          radio.enableDynamicPayloads();
          radio.setAutoAck(true);
        }
      }
    }
  }

  if(radio.failureDetected) {
    Serial.print(F("Hardware failure detected "));
    Serial.println(radio.failureDetected);
    radio.failureDetected = 0; // Reset flag
  }

  digitalWrite(PIN_LED, HIGH);
//   Serial.println(F("Fin verif maintenance"));
}


void attendreProchaineLecture() {

  Serial.println("Sleep");
  
  // Power down the radio.  Note that the radio will get powered back up
  // on the next write() call.
  // mesh.releaseAddress();
  radio.powerDown();

  digitalWrite(PIN_LED, LOW);
  power.deepSleep();
  digitalWrite(PIN_LED, HIGH);

  radio.powerUp();
  radio.startListening();
  mesh.renewAddress(1000);

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
