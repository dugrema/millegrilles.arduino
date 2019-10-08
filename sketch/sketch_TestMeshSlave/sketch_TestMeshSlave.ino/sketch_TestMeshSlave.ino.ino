#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include "MilleGrillesAppareilsPaquet.h"
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
//#include <printf.h>

#define HASH_SIZE 32
#define BLOCK_SIZE 64

/**** Configure the nrf24l01 CE and CS pins ****/
RF24 radio(7,8);
RF24Network network(radio);
RF24Mesh mesh(radio,network);

/** 
 * User Configuration: 
 * nodeID - A unique identifier for each radio. Allows addressing to change dynamically
 * with physical changes to the mesh. (numbers 1-255 allowed)
 * 
 * otherNodeID - A unique identifier for the 'other' radio
 * 
 **/
#define VERSION 0x7
#define UUID_NOEUD 0xed,0x6f,0x2a,0x6a,0xe9,0x2a,0x11,0xe9,0x95,0xe3,0x0,0x15,0x5d,0x1,0x1f,0x9
#define nodeID 2
#define otherNodeID 0    
#define PIN_LED 6
#define CYCLES_SOMMEIL 1

const PROGMEM byte uuid[16] = {UUID_NOEUD};

uint32_t millisTimer=0;
uint32_t forcedRenewTimer=0;
int compteur=0;

// Watchdog et sleep behaviour
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

  Serial.begin(9600);
  pinMode(PIN_LED, OUTPUT);

  digitalWrite(PIN_LED, HIGH);

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

  // Mesh configuration
  mesh.setNodeID(nodeID);  

  // Connect to the mesh
  Serial.println(F("Connecting to the mesh..."));
  mesh.begin(87);
}


void loop() {

  mesh.update();
 
  if(network.available()){
    RF24NetworkHeader header;
    uint32_t mills;
    network.read(header,&mills,sizeof(mills));
    Serial.print("Rcv "); Serial.print(mills);
    Serial.print(" from nodeID ");
    int _ID = mesh.getNodeID(header.from_node);
    if( _ID > 0){
       Serial.println(_ID);
    }else{
       Serial.println("Mesh ID Lookup Failed"); 
    }
  }
  
  // Send to the other node every second
  bool transmissionOk = false;
  for(byte essai=0; !transmissionOk && essai<5; essai++) {
    transmissionOk = transmettrePaquet0(0x101, 2);
    if( ! transmissionOk ) {
      if( ! mesh.checkConnection() ) {
        mesh.renewAddress(15000);
      }
    }
  }

  if(transmissionOk) {
    Serial.println(F("Transmission Paquet 0 OK, on continue"));
    bool paquetTHP = transmettrePaquetLectureTHP(1, 298, 675, 1019);
    if(!paquetTHP) {
      Serial.println("Erreur transmission THP");
    } else {
      bool paquetMv = transmettrePaquetLectureMillivolt(2, 1500, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF);
      if(!paquetTHP) {
        Serial.println("Erreur transmission millivolt");
      }
    }
  }

  attendreProchaineLecture();

}

bool transmettrePaquet0(uint16_t typeMessage, uint16_t nombrePaquets) {
    uint8_t transmitBuffer[24];

    // Format message Paquet0:
    // Version (1 byte)
    // UUID 16 bytes
    // TYPE MESSAGE 2 bytes
    // Nombre paquets 2 bytes
    transmitBuffer[0] = VERSION;
    ecrireUUID(transmitBuffer + 1);
    memcpy(transmitBuffer + 17, &typeMessage, sizeof(typeMessage));
    memcpy(transmitBuffer + 20, &nombrePaquets, sizeof(nombrePaquets));
    transmitBuffer[22] = 0;
    transmitBuffer[23] = 0;
    
    // printArray(transmitBuffer, sizeof(transmitBuffer));

    bool transmissionOk = mesh.write(transmitBuffer, 'P', sizeof(transmitBuffer), otherNodeID);
    if(!transmissionOk) {
      Serial.println("Erreur transmission paquet 0");
    } else {
      Serial.println("Paquet 0 ok");
    }

    return transmissionOk;
}

bool transmettrePaquetLectureTHP(uint16_t noPaquet, int temperature, uint16_t humidite, uint16_t pression) {

  // Format message THP (Temperatures, Humidite, Pression Atmospherique)
  // noPaquet - 2 bytes
  // typeMessage - 2 bytes
  // temperature - 2 bytes
  // humidite - 2 bytes
  // pression - 2 bytes

  uint16_t typeMessage = 0x102;

  uint8_t transmitBuffer[24];
  memcpy(transmitBuffer + 0, &noPaquet, sizeof(noPaquet));
  memcpy(transmitBuffer + 2, &typeMessage, sizeof(typeMessage));
  memcpy(transmitBuffer + 4, &temperature, sizeof(temperature));
  memcpy(transmitBuffer + 6, &humidite, sizeof(humidite));
  memcpy(transmitBuffer + 8, &pression, sizeof(pression));

  return transmettrePaquet(transmitBuffer);
}

bool transmettrePaquetLectureMillivolt(uint16_t noPaquet, uint32_t millivolt1, uint32_t millivolt2, uint32_t millivolt3, uint32_t millivolt4) {

  // Format message millivol - supporte jusqu'a 4 milliards de volts
  // noPaquet - 2 bytes
  // typeMessage - 2 bytes
  // millivolt 1 - 4 bytes
  // millivolt 2 - 4 bytes
  // millivolt 3 - 4 bytes
  // millivolt 4 - 4 bytes

  uint16_t typeMessage = 0x103;

  uint8_t transmitBuffer[24];
  memcpy(transmitBuffer + 0, &noPaquet, sizeof(noPaquet));
  memcpy(transmitBuffer + 2, &typeMessage, sizeof(typeMessage));
  memcpy(transmitBuffer + 4, &millivolt1, sizeof(millivolt1));
  memcpy(transmitBuffer + 8, &millivolt2, sizeof(millivolt2));
  memcpy(transmitBuffer + 12, &millivolt3, sizeof(millivolt3));
  memcpy(transmitBuffer + 16, &millivolt4, sizeof(millivolt4));

  return transmettrePaquet(transmitBuffer);
}

bool transmettrePaquet(void* paquet) {
  bool transmissionOk = mesh.write(paquet, 'p', 24, otherNodeID);
  return transmissionOk;
}

void attendreProchaineLecture() {
  mesh.releaseAddress();
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
  mesh.renewAddress(5000);  // Renouveller l'adresse, 5 secondes max

  current_sleep_count = 0; // Reset sleep cycles
}

// Ecrit UUID a partir du program space vers un buffer
void ecrireUUID(byte* destination) {
  memcpy_P(destination, &uuid, 16);
}

//void printArray(byte* liste, int len) {
//  byte valeur = 0;
//  for(int i=0; i<len; i++){
//    valeur = liste[i];
//    printHex(valeur);
//  }
//
//  Serial.println();  
//}
//
//void printHex(uint8_t num) {
//  char hexCar[2];
//
//  sprintf(hexCar, "%02X", num);
//  Serial.print(hexCar);
//}

