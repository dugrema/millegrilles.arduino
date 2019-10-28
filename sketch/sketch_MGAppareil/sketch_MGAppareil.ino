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
RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);
RF24Network network(radio);
RF24Mesh mesh(radio,network);

byte nodeId = NODE_ID_DEFAULT;

// Renouveller nodeId. Toujours verifier nodeId avec DHCP au demarrage
bool doitVerifierAdresseDhcp = true;

// Flag qui indique un echec de transmission
bool erreurMesh = false;
bool transmissionOk = false;

// Helper conversion de donnees avec protocole Version 7
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

// Setup initial
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

  Serial.println(F("Setup radio"));
  radio.begin();
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);
  radio.setRetries(15, 15);
  // radio.setPALevel(RF24_PA_HIGH);
  radio.setPALevel(RF24_PA_LOW);

  Serial.print(F("Connexion mesh avec nodeId "));
  mesh.setNodeID(nodeId);
  Serial.println(nodeId);
  mesh.begin(CANAL_MESH, RATE_MESH, MESH_RENEW_TIMEOUT);
  radio.printDetails();

  Serial.println(F("Connexion mesh initialise"));

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
  
  if( ! erreurMesh ) {
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
    transmettrePaquets();
  
    // Lecture reseau
    ecouterReseau();
  } 

  if(!power.isAlimentationSecteur()) {
    // Attendre la prochaine lecture
    attendreProchaineLecture();

    // Clear le flag du watchdog.
    f_wdt = 0;
  } else if( erreurMesh ) {
    // Mode secteur et on a une erreur reseau. On introduit un delai
    // avant de tenter une nouvelle connexion.
    long delai = 1 * (MESH_RENEW_TIMEOUT + (4 * nodeId));  // Delai en ms, utilise nodeID pour repartir attente
    Serial.print("Erreur mesh, reconnecter apres ");
    Serial.println(delai);
    delay(delai);
  }
}

// Transmet les paquets. 
bool transmettrePaquets() {

  const byte nombreEssais = 3;
  const byte nombreEssaisPaquet0 = 10;

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
  transmissionOk = false;
  for(byte essai=0; essai<nombreEssaisPaquet0 && !transmissionOk; essai++) {
    transmissionOk = prot7.transmettrePaquet0(MSG_TYPE_LECTURES_COMBINEES, nombrePaquets);
    mesh.update();
  }

  byte compteurPaquet = 1;  // Fourni le numero du paquet courant

  // Dalsemi OneWire (1W)
  #ifdef BUS_MODE_ONEWIRE
    if(transmissionOk) {

      transmissionOk = false;
      for(byte essai=0; essai<nombreEssais && !transmissionOk; essai++) {
        transmissionOk = prot7.transmettrePaquetLectureOneWire(compteurPaquet, &oneWireHandler);
        compteurPaquet++;
        mesh.update();
      }
    }
  #endif

  // DHT
  #if defined(DHTPIN) && defined(DHTTYPE)
    if(transmissionOk) {

      transmissionOk = false;
      for(byte essai=0; essai<nombreEssais && !transmissionOk; essai++) {
        transmissionOk = prot7.transmettrePaquetLectureTH(compteurPaquet, &dht);
        compteurPaquet++;
        mesh.update();
      }
    }
  #endif 

  // Adafruit BMP
  #ifdef BUS_MODE_I2C
    if(transmissionOk) {

      transmissionOk = false;
      for(byte essai=0; essai<nombreEssais && !transmissionOk; essai++) {
      
        transmissionOk = prot7.transmettrePaquetLectureTP(compteurPaquet, &bmp);
        compteurPaquet++;
        mesh.update();
      }
    }
  #endif

  // Power info
  if(transmissionOk) {

    transmissionOk = false;
    for(byte essai=0; essai<nombreEssais && !transmissionOk; essai++) {

      transmissionOk = prot7.transmettrePaquetLecturePower(compteurPaquet, &power);
      compteurPaquet++;
      mesh.update();
    }
  }

  return transmissionOk;
}

// Entretien reseau. Retourne false si le reseau est en erreur.
bool networkMaintenance() {

  // Section entretien connexion, reconnexion
  mesh.update();
  if(erreurMesh) {
    while(network.available()) {
      // Vider buffer
      mesh.update();
      networkProcess();
    }
    
    Serial.println(F("Reconnexion au mesh"));

    // erreurMesh = !mesh.checkConnection();

    if( erreurMesh ) {
      if( ! mesh.renewAddress(MESH_RENEW_TIMEOUT / 4) ) {
        // Tenter de redemarrer la radio
        Serial.println(F("Redemarrage radio"));
        mesh.begin(CANAL_MESH, RATE_MESH, MESH_RENEW_TIMEOUT);
      }
    }

    erreurMesh = false;
  }

  if(radio.failureDetected) {
    Serial.print(F("Hardware failure detected "));
    Serial.println(radio.failureDetected);
    radio.failureDetected = 0; // Reset flag
  }

  // Section DHCP
  if(doitVerifierAdresseDhcp) {
    // Le senseur vient d'etre initialise, il faut demander un nouveau nodeId au serveur
    erreurMesh = !prot7.transmettreRequeteDhcp();
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
    // Mode alimentation secteur - cet appareil devient un node fiable pour le mesh.
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
    mesh.update();
    bool resultatLecture = networkProcess();
    if(!resultatLecture) {
      return false;
    }
    
  }

  digitalWrite(PIN_LED, HIGH);  
  return true;
}

bool networkProcess() {
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
    erreurMesh = ! mesh.checkConnection();
    if( erreurMesh ) {
      return false;  // Arrete traitement, va reconnecter et recommencer dans loop
    }
  }

  return true;
}

void attendreProchaineLecture() {

  Serial.println(F("Sleep"));
  delay(75); // Finir transmettre Serial
  
  // Power down the radio.  Note that the radio will get powered back up
  // on the next write() call.
  mesh.releaseAddress();
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
