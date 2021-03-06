// Ce sketch sert a programmer un UUID dans le EEPROM de l'appareil

#include <EEPROM.h>

#define UUID_NOEUD 0xd1, 0xd0, 0xae, 0xd2, 0x54, 0xcd, 0x11, 0xea, 0x97, 0xf8, 0xb8, 0x27, 0xeb, 0x90, 0x64, 0xaf

#define PIN_LED 6
#define ADDRESS_SENSEUR 9
#define ADDRESS_UUID 15
#define NODE_ID_DEFAULT 1

const byte uuid[16] = {UUID_NOEUD};

void setup() {  
  Serial.begin(115200);
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
