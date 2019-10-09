// Ce sketch sert a programmer un UUID dans le EEPROM de l'appareil

#include <EEPROM.h>

#define UUID_NOEUD 0x4c, 0xc2, 0x98, 0xd0, 0xea, 0x93, 0x11, 0xe9, 0x95, 0xe3, 0x0, 0x15, 0x5d, 0x1, 0x1f, 0x9

#define PIN_LED 6
#define ADDRESS_SENSEUR 9
#define ADDRESS_UUID 15
#define NODE_ID_DEFAULT 1

const byte uuid[16] = {UUID_NOEUD};

void setup() {  
  Serial.begin(9600);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  Serial.print("Ecriture senseur nodeid: ");
  Serial.println(NODE_ID_DEFAULT);
  EEPROM.update(ADDRESS_SENSEUR, NODE_ID_DEFAULT);

  Serial.print("Ecriture uuid: ");
  printArray(uuid, 16);
  EEPROM.put(ADDRESS_UUID, uuid);

  Serial.println("Ecriture terminee avec succes");
}

void loop() {
  delay(500);
  digitalWrite(PIN_LED, LOW);
  delay(500);
  digitalWrite(PIN_LED, HIGH);
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

