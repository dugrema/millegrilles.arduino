// Ce sketch sert a programmer un UUID dans le EEPROM de l'appareil

#include <EEPROM.h>

#define UUID_NOEUD 0x10, 0x92, 0xf5, 0x2e, 0xf3, 0x5a, 0x11, 0xe9, 0xb2, 0x85, 0xb8, 0x27, 0xeb, 0x90, 0x64, 0xaf

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
