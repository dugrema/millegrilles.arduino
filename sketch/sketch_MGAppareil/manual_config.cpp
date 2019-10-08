#include "manual_config.h"

//byte manual_config::demander_mode_interactif(int delai_ms) {
//
//  _mode = MODE_NORMAL;
//
//  Serial.println("Entrer i pour mode interactif");
//
//  unsigned long debut = millis();
//
//  while( millis() - debut < delai_ms ) {
//    if(Serial.available()) {
//  
//      char valeur = Serial.read();
//  
//      if(valeur == 'i') {
//        _mode = MODE_INTERACTIF;
//      }
//    }
//  }
//
//  Serial.print("Mode configuration:");
//  Serial.println(_mode);
//
//  return _mode;
//  
//}
//
//boolean manual_config::is_mode_interactif() {
//
//  Serial.print("Mode courant:");
//  Serial.println(_mode);
//  if(MODE_INTERACTIF == _mode) {
//    Serial.println("Interactif");
//    return true;
//  }
//
//  Serial.println("Normal");
//  return false;
//  
//}
//
//void manual_config::executer_mode_interactif() {
//
//  Serial.println("-----");
//  Serial.println("");
//
//  Serial.println("Mode interactif");
//
//  Serial.println("Entrer commande et valeurs a configurer");
//  Serial.println("a=Afficher la configuration courante");
//  Serial.println("s=Sauvegarder la configuration");
//  Serial.println("r=Reinitialiser defauts");
//  Serial.println("i=ID senseur");
//
//  while( Serial.available() == 0 ) {
//    delay(500);
//  }
//
//  char commande = Serial.read();
//
//  switch(commande) {
//    case 'a':
//      afficher_configuration();
//      break;
//    case 'r':
//      reinit_defauts();
//      break;
//    case 's':
//      enregistrer_configuration();
//      break;
//    case 'i':
//      Serial.print("ID Senseur");
//      senseur = (byte)read_int_fromserial();
//      Serial.println(senseur);
//      break;
//  }
//  
//}

void manual_config::lire_configuration() {

  Serial.println("");
  Serial.println("Lecture de la configuration du EEPROM");

  // EEPROM.get(ADDRESS_ADDRESSSINK, addressSink);
  EEPROM.get(ADDRESS_SENSEUR, senseur);
  // EEPROM.get(ADDRESS_RF24_CE_PIN, RF24_CE_PIN);
  // EEPROM.get(ADDRESS_RF24_CSN_PIN, RF24_CSN_PIN);
  // EEPROM.get(ADDRESS_DHT_PIN, DHT_PIN);
  EEPROM.get(ADDRESS_DHT_TYPE, DHT_TYPE);  
  EEPROM.get(ADDRESS_BATT_PIN, battery_pin);  

}


void manual_config::enregistrer_configuration() {

    // EEPROM.put(ADDRESS_ADDRESSSINK, addressSink);
    EEPROM.put(ADDRESS_SENSEUR, senseur);
    // EEPROM.put(ADDRESS_RF24_CE_PIN, RF24_CE_PIN);
    // EEPROM.put(ADDRESS_RF24_CSN_PIN, RF24_CSN_PIN);
    // EEPROM.put(ADDRESS_DHT_PIN, DHT_PIN);
    EEPROM.put(ADDRESS_DHT_TYPE, DHT_TYPE);
    EEPROM.put(ADDRESS_BATT_PIN, battery_pin);

    Serial.println("Configuration sauvegardee");
    
}

void manual_config::reinit_defauts() {

    // addressSink = 0x316b6e69E1LL; // 8 bytes - ADDRESS=0
    senseur = CONFIG_NO_SENSEUR; // 1 byte - ADDRESS = 
    // RF24_CE_PIN = 7; // 1 byte  7 ou 10
    // RF24_CSN_PIN = 8; // 1 byte  8 ou A2
    // DHT_PIN = 4; // 1 byte
    DHT_TYPE = DHT22; // 1 byte
    battery_pin = 0;  // 0 pur VCC, sinon pin pour analog()    
  
}

void manual_config::afficher_configuration() {

  Serial.println("");
  Serial.println("Configuration");

//  Serial.print("Address Sink: 0x");
//  for(int i=7;i>=0;i--) {
//    Serial.print((byte)((byte*)(&addressSink))[i], HEX);
//  }
//  Serial.println("");

  Serial.print("Senseur: ");
  Serial.println(senseur);

//  Serial.print("RF24 CE Pin: ");
//  Serial.println(RF24_CE_PIN);
//
//  Serial.print("RF24 CSN Pin: ");
//  Serial.println(RF24_CSN_PIN);
//
//  Serial.print("DHT Pin: ");
//  Serial.println(DHT_PIN);

  Serial.print("DHT Type: ");
  Serial.println(DHT_TYPE);

  Serial.print("Battery Pin: ");
  Serial.println(battery_pin);

}

//int manual_config::read_int_fromserial() {
//
//   byte index = 0;
//   char valeurs[9];
//
//   while(Serial.available() && index<8) {
//    valeurs[index++] = (char)Serial.read();
//   }
//   valeurs[index] = '\0';
//
//   int valeur = atoi(valeurs);
//
//   return valeur;
//  
//}

