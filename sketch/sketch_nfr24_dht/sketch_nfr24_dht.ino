#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
#include "dht.h"
#include "manual_config.h"

#include <nRF24L01.h>
//#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

#define DEBUG Serial.print
#define DEBUGLN Serial.println

#define NO_TEMP -1000
#define NO_PRESSURE 0
#define NO_HUMIDITY 1000

#define CYCLES_SOMMEIL 5

// Version du protocole de transmission NRF24
#define VERSION_PROTOCOLE 6
#define TAILLE_PACKET_NRF24 11

// ***********************************
// Configurer l'information du senseur
// ***********************************
uint64_t addressSink = 0x0; //0x316b6e69E1LL; // Canal 1 du sink
byte senseur = 0; //1;

#define PIN_LED 6

#define RF24_CE_PIN 7
#define RF24_CSN_PIN 8
byte rf24_ce_pin = RF24_CE_PIN;
byte rf24_csn_pin = RF24_CSN_PIN;

#define DHTPIN 4
//#define DHTTYPE DHT11
#define DHTTYPE 11
#define DHT_READ_ATTEMPTS 3

byte dht_pin = DHTPIN;
byte dht_type = DHTTYPE;
int dht_chk=DHTLIB_ERROR_TIMEOUT;

#define BATTERY_PIN_VCC 0
byte battery_pin = BATTERY_PIN_VCC; // Defaut est VCC

// ***********************************

// Configuration
manual_config configuration;

// Senseurs
dht dht_sensor;//(DHTPIN, DHTTYPE);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
boolean bmpActif = false;

// Radio nRF24L01 sur pins CE=7, CSN=8
RF24 radio(RF24_CE_PIN,RF24_CSN_PIN);
byte packet[TAILLE_PACKET_NRF24]; // Packet transmis par NRF24L

volatile int f_wdt=1;
const byte sleep_cycles=CYCLES_SOMMEIL; // Nombre de cycles de sommeil (par reveil watchdog)
byte current_sleep_count=0; // Cycles actuels
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

  // Preparation Senseur DHT
  if( dht_type ) {
    if( DHTPIN != dht_pin || DHTTYPE != dht_type) {
      // La configuration indique d'utiliser des valeurs differentes pour DHT
      Serial.print(F("Override DHT, pin:"));
      Serial.print(dht_pin);
      Serial.print(F(", type:"));
      Serial.println(dht_type);
      
      //dht = DHT(dht_pin, dht_type);
    }

    //dht.begin(); 
    
  } else if( !dht_type ) {
    Serial.println("Senseur DHT inactif");
  }

  // Preparation senseur BMP
  bmpActif = bmp.begin();
  if( !bmpActif ) {
    Serial.println("Senseur BMP180 inactif");
  }

  if( RF24_CE_PIN != rf24_ce_pin || RF24_CSN_PIN != rf24_csn_pin ) {
    Serial.print(F("Override RF24, CE PIN:"));
    Serial.print(rf24_ce_pin);
    Serial.print(F(", CSN PIN:"));
    Serial.println(rf24_csn_pin);

    radio = RF24(rf24_ce_pin, rf24_csn_pin);
  }

  // Ouverture de la radio
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(76);
  radio.setCRCLength(RF24_CRC_16);
  radio.openWritingPipe(addressSink);

  // Start the radio listening for data
  radio.startListening();

  packet[0] = VERSION_PROTOCOLE; // Version du protocole NRF24
  packet[1] = senseur;  // On insere la valeur du senseur dans le paquet (ne changera jamais)

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

  if( configuration.is_mode_interactif() == true ) {

    // On est en mode interactif pour la configuration
    configuration.executer_mode_interactif();
    
  } else {

    /* Clear le flag du watchdog. */
    f_wdt = 0;
    
    // put your main code here, to run repeatedly:
    
    lireHumiditeDansPacket();
    lireTemperatureDansPacket();
    lirePressionDansPacket();
  
    lireVoltageBatterie();
  
    calculerChecksum();
  
    transmettrePaquet(); // Transmet le packet par nRF24L
  
    attendreProchaineLecture(); // Attendre la prochaine lecture
    
  }
}

void transmettrePaquet() {
  radio.stopListening();
  int resultWrite = radio.write( &packet, sizeof(packet) );

  if( !resultWrite ) {
    DEBUGLN(F("Erreur transmission"));
  }
  radio.startListening();
}

// Lit la temperature vers le packet
void lireTemperatureDansPacket() {

  int temperature = NO_TEMP;
  boolean tempOK = false;
  if( bmpActif ) {
    sensors_event_t event;
    bmp.getEvent(&event);
    float tempfloat;
    bmp.getTemperature(&tempfloat);
    DEBUG(F("TempBMP:"));
    DEBUGLN(tempfloat);
    temperature = (int)(tempfloat * 10.0);
    tempOK = true;
  } else if( dht_type ) {
    float tempfloat;
    switch (dht_chk)
    {
      case DHTLIB_OK:  
      tempfloat = dht_sensor.temperature;
      temperature = int((tempfloat * 10.0));
      tempOK = true;
      break;
      case DHTLIB_ERROR_CHECKSUM: 
      Serial.print("DHT Checksum error,\t"); 
      break;
      case DHTLIB_ERROR_TIMEOUT: 
      Serial.print("DHT time out error,\t"); 
      break;
      default: 
      Serial.print("DHT Unknown error,\t"); 
      break;
    }
    
  }

  if(tempOK) {
    DEBUG(F("temperature:"));
    DEBUGLN(temperature);
  
    packet[2] = ((byte*)&temperature)[0];
    packet[3] = ((byte*)&temperature)[1];
  }
}

// Lit la pression vers le packet
void lirePressionDansPacket() {
  unsigned int pression = NO_PRESSURE;

  if( bmpActif ) {
    sensors_event_t event;
    bmp.getEvent(&event);
    if( event.pressure ) {
      float pressureFloat;
      bmp.getPressure(&pressureFloat);
      DEBUG(F("PressBMP:"));
      DEBUGLN(pressureFloat);
  
      pression = int((pressureFloat/10.0));
    }
  }
  
  DEBUG("pression:");
  DEBUGLN(pression);
  
  packet[6] = ((byte*)&pression)[0];
  packet[7] = ((byte*)&pression)[1];
}

int lireDHT() {

  byte dht_read_attempt = DHT_READ_ATTEMPTS;
  boolean dht_read_ok = false;
  while( !dht_read_ok && dht_read_attempt-- > 0 ) {

    delay(5);

    switch(dht_type) {
      case 11:
      dht_chk = dht_sensor.read11(dht_pin);
      break;
      case 22:
      dht_chk = dht_sensor.read22(dht_pin);
      break;
      default:
      dht_chk = false;
      break;
    }

    dht_read_ok = dht_chk == DHTLIB_OK;
    if( !dht_read_ok ) {
      Serial.print("DHT Error:");
      Serial.println(DHT_READ_ATTEMPTS - dht_read_attempt - 1);
      delay(2000);
    }
    
  }

  return dht_chk;
}

// Lit l'humidite vers le packet
void lireHumiditeDansPacket() {

  unsigned int humidite = NO_HUMIDITY;
  boolean humOK = false;
  
  if( dht_type ) {
    int temperature = NO_TEMP;
    dht_chk = lireDHT();
    
    double humiditefloat, tempfloat;
    switch (dht_chk)
    {
      case DHTLIB_OK:  
      humiditefloat = dht_sensor.humidity;
      humidite = int((humiditefloat * 10.0)); 
      humOK = true;
      break;
      case DHTLIB_ERROR_CHECKSUM: 
      Serial.print("DHT Checksum error,\t"); 
      break;
      case DHTLIB_ERROR_TIMEOUT: 
      Serial.print("DHT ime out error,\t"); 
      break;
      default: 
      Serial.print("DHT Unknown error,\t"); 
      break;
    }
    
//    float humiditefloat = dht.readHumidity();
//    if(!isnan(humiditefloat)) {
//      humidite = int((humiditefloat * 10.0));  
//    }
 } 

  if(humOK) {
    DEBUG(F("humidite:"));
    DEBUGLN(humidite);
    packet[4] = ((byte*)&humidite)[0];
    packet[5] = ((byte*)&humidite)[1];
  }
}

// Ajoute le checksum au packet
void calculerChecksum() {

  packet[10] = 0;
  for(int i=0; i<10; i++) {
    packet[10] += packet[i];
  }
}

void lireVoltageBatterie() {

  long voltage = 0;

  voltage = readVcc();
  if( battery_pin != BATTERY_PIN_VCC ) {
    int sensorValue = analogRead(battery_pin);
    voltage = map(sensorValue, 0, 1023, 0, voltage);
  }
  DEBUG(F("Volt: "));
  DEBUGLN(voltage);
  unsigned int mvint = int(voltage);
  packet[8] = ((byte*)&mvint)[0];
  packet[9] = ((byte*)&mvint)[1];
  
}

void attendreProchaineLecture() {
  delay(75); // Transmission serie

  // Power down the radio.  Note that the radio will get powered back up
  // on the next write() call.
  radio.powerDown();
  digitalWrite(PIN_LED, LOW);

  // set_sleep_mode(SLEEP_MODE_ADC);
  // Le power saving est maximal - le premier byte sur le UART est perdu
  // set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  while( current_sleep_count++ < sleep_cycles ) {
    sleep_enable();

    /* Now enter sleep mode. */
    sleep_mode();
  }
      
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
      
  /* Re-enable the peripherals. */
  power_all_enable();
  radio.startListening();

  digitalWrite(PIN_LED, HIGH);

  current_sleep_count = 0; // Reset sleep cycles

  //delay(5000);
}

void chargerConfiguration() {

  configuration.lire_configuration();
  configuration.demander_mode_interactif(2000); // Attendre 2 secondes pour mode interactif
  
  EEPROM.get(ADDRESS_ADDRESSSINK, addressSink);
  EEPROM.get(ADDRESS_SENSEUR, senseur);
  EEPROM.get(ADDRESS_RF24_CE_PIN, rf24_ce_pin);
  EEPROM.get(ADDRESS_RF24_CSN_PIN, rf24_csn_pin);
  EEPROM.get(ADDRESS_DHT_PIN, dht_pin);
  EEPROM.get(ADDRESS_DHT_TYPE, dht_type);  
  EEPROM.get(ADDRESS_BATT_PIN, battery_pin);  

  DEBUG("No senseur: ");
  DEBUGLN(senseur);
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

