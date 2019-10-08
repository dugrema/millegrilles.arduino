//#include <printf.h>

#include <EEPROM.h>

#include <RF24Mesh_config.h>
#include <RF24Mesh.h>
#include <RF24_config.h>

#include "Config.h"

#include "Power.h"
#include "manual_config.h"
#include "MGAppareilsProt.h"

// Librairies de senseurs
#include "dht.h"
#include "AdafruitSensors.h"
#include "OneWireHandler.h"

#define DEBUG Serial.print
#define DEBUGLN Serial.println

// ***********************************
// Configurer l'information du senseur
// ***********************************
byte senseur = 1; // Valeur 1 par defaut, le master dhcp va reassigner un nodeID permanent sur son resaeu

// ***********************************

// Configuration
manual_config configuration;

// Conserver le UUID de l'appareil dans l'espace programme
const PROGMEM byte uuid[16] = {UUID_NOEUD};

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

  // Preparation senseur BMP
  bmpActif = bmp.begin();
  if( !bmpActif ) {
    Serial.println("Senseur BMP180 inactif");
  }

  // Ouverture de la radio, mesh configuration
  mesh.setNodeID(NODE_ID_DEFAULT);
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

  networkMaintenance();

  /* Clear le flag du watchdog. */
  f_wdt = 0;

  // Effectuer lectures
  dht.lire();
  power.lireVoltageBatterie();

  // Transmettre information du senseur
  transmettrePaquets();

  // Attendre la prochaine lecture
  attendreProchaineLecture();
}

void transmettrePaquets() {

  prot7.transmettrePaquet0(0x101, 3);

  byte compteurPaquet = 1;
  prot7.transmettrePaquetLectureTH(compteurPaquet++, &dht);
  prot7.transmettrePaquetLecturePower(compteurPaquet++, &power);
  
}

void networkMaintenance() {
  mesh.update();

  // Code pour reception de messages
  /*if(network.available()){
    RF24NetworkHeader header;
    uint32_t mills;
    network.read(header,&mills,sizeof(mills));
    int _ID = mesh.getNodeID(header.from_node);
    if( _ID > 0){
       Serial.println(_ID);
    }else{
       Serial.println("Mesh ID Lookup Failed"); 
    }
  }*/
}

// Lit la temperature vers le packet
int lireTemperatureDansPacket() {

  int temperature = NO_TEMP;
  boolean tempOK = false;
//  if( bmpActif ) {
//    sensors_event_t event;
//    bmp.getEvent(&event);
//    float tempfloat;
//    bmp.getTemperature(&tempfloat);
//    DEBUG(F("TempBMP:"));
//    DEBUGLN(tempfloat);
//    temperature = (int)(tempfloat * 10.0);
//    tempOK = true;
//  } else if( dht_type ) {
//    float tempfloat;
//    switch (dht_chk)
//    {
//      case DHTLIB_OK:  
//      tempfloat = dht_sensor.temperature;
//      temperature = int((tempfloat * 10.0));
//      tempOK = true;
//      break;
//      case DHTLIB_ERROR_CHECKSUM: 
//      Serial.print("DHT Checksum error,\t"); 
//      break;
//      case DHTLIB_ERROR_TIMEOUT: 
//      Serial.print("DHT time out error,\t"); 
//      break;
//      default: 
//      Serial.print("DHT Unknown error,\t"); 
//      break;
//    }
//    
//  }

  if(tempOK) {
    DEBUG(F("temperature:"));
    DEBUGLN(temperature);
  
    return temperature;
  }

  return 32767;
}

// Lit la pression vers le packet
int lirePressionDansPacket() {
  unsigned int pression = NO_PRESSURE;

//  if( bmpActif ) {
//    sensors_event_t event;
//    bmp.getEvent(&event);
//    if( event.pressure ) {
//      float pressureFloat;
//      bmp.getPressure(&pressureFloat);
//      DEBUG(F("PressBMP:"));
//      DEBUGLN(pressureFloat);
//  
//      pression = int((pressureFloat/10.0));
//    }
//  }
//  
//  DEBUG("pression:");
//  DEBUGLN(pression);
  
  return pression;
}

int lireDHT() {
    int dht_chk=0;
//  byte dht_read_attempt = DHT_READ_ATTEMPTS;
//  boolean dht_read_ok = false;
//  while( !dht_read_ok && dht_read_attempt-- > 0 ) {
//
//    delay(5);
//
//    switch(dht_type) {
//      case 11:
//      dht_chk = dht_sensor.read11(dht_pin);
//      break;
//      case 22:
//      dht_chk = dht_sensor.read22(dht_pin);
//      break;
//      default:
//      dht_chk = false;
//      break;
//    }
//
//    dht_read_ok = dht_chk == DHTLIB_OK;
//    if( !dht_read_ok ) {
//      Serial.print("DHT Error:");
//      Serial.println(DHT_READ_ATTEMPTS - dht_read_attempt - 1);
//      delay(2000);
//    }
//    
//  }

  return dht_chk;
}

// Lit l'humidite vers le packet
int lireHumiditeDansPacket() {

  unsigned int humidite = NO_HUMIDITY;
  boolean humOK = false;
  
//  if( dht_type ) {
//    int temperature = NO_TEMP;
//    dht_chk = lireDHT();
//    
//    double humiditefloat, tempfloat;
//    switch (dht_chk)
//    {
//      case DHTLIB_OK:  
//      humiditefloat = dht_sensor.humidity;
//      humidite = int((humiditefloat * 10.0)); 
//      humOK = true;
//      break;
//      case DHTLIB_ERROR_CHECKSUM: 
//      Serial.print("DHT Checksum error,\t"); 
//      break;
//      case DHTLIB_ERROR_TIMEOUT: 
//      Serial.print("DHT ime out error,\t"); 
//      break;
//      default: 
//      Serial.print("DHT Unknown error,\t"); 
//      break;
//    }
    
//    float humiditefloat = dht.readHumidity();
//    if(!isnan(humiditefloat)) {
//      humidite = int((humiditefloat * 10.0));  
//    }
// } 

  if(humOK) {
    return humidite;
  }

  return 0xFFFF;
}


void attendreProchaineLecture() {
  // Power down the radio.  Note that the radio will get powered back up
  // on the next write() call.
  radio.powerDown();

  delay(75); // Transmission serie

  digitalWrite(PIN_LED, LOW);
  power.deepSleep();
  digitalWrite(PIN_LED, HIGH);

  radio.startListening();

}

void lireVoltageBatterie() {
  
}

void chargerConfiguration() {

  configuration.lire_configuration();
  // configuration.demander_mode_interactif(2000); // Attendre 2 secondes pour mode interactif
  
//  EEPROM.get(ADDRESS_ADDRESSSINK, addressSink);
  EEPROM.get(ADDRESS_SENSEUR, senseur);
//  EEPROM.get(ADDRESS_RF24_CE_PIN, rf24_ce_pin);
//  EEPROM.get(ADDRESS_RF24_CSN_PIN, rf24_csn_pin);
//  EEPROM.get(ADDRESS_DHT_PIN, dht_pin);
//  EEPROM.get(ADDRESS_DHT_TYPE, dht_type);  
  // EEPROM.get(ADDRESS_BATT_PIN, battery_pin);  

  DEBUG(F("NodeID senseur "));
  DEBUGLN(senseur);
}

