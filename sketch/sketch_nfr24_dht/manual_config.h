#ifndef manual_config_h
#define manual_config_h

#include <Arduino.h>
#include <EEPROM.h>
#include <DHT.h>

// Rendu a 25
#define CONFIG_NO_SENSEUR 25

// Addresses EEPROM
#define ADDRESS_ADDRESSSINK 0
#define ADDRESS_SENSEUR 9
#define ADDRESS_RF24_CE_PIN 10
#define ADDRESS_RF24_CSN_PIN 11
#define ADDRESS_DHT_PIN 12
#define ADDRESS_DHT_TYPE 13
#define ADDRESS_BATT_PIN 14

#define MODE_NORMAL 1
#define MODE_INTERACTIF 2

class manual_config
{
public:

    // Utiliser cette methode au setup, donne une chance d'envoyer le signal pour mode interactif
    byte demander_mode_interactif(int delai_ms);

    // Fonction a utiliser lorsque le mode interactif est utilise, fourni un prompt
    void executer_mode_interactif();

    // Utiliser cette method au debut de la loop pour savoir si on est dans le mode interactif.
    boolean is_mode_interactif();

    // Charge la configuration a partir du EEPROM.
    void lire_configuration();

    // Enregistre (commit) la configuration dans le EEPROM.
    void enregistrer_configuration();

    // Reinitialise avec les valeurs par defaut.
    void reinit_defauts();

    void afficher_configuration();

private:
    byte _mode;

    uint64_t addressSink = 0x316b6e69E1LL; // 8 bytes - ADDRESS=0
    byte senseur = CONFIG_NO_SENSEUR; // 1 byte - ADDRESS = 
    byte RF24_CE_PIN = 7; // 1 byte  7 ou 10
    byte RF24_CSN_PIN = 8; // 1 byte  8 ou A2
    byte DHT_PIN = 4; // 1 byte
    
    //byte DHT_TYPE = DHT22; //DHT11; // 1 byte - 0 pour disable
    //DHT11,DHT21,DHT22
    byte DHT_TYPE = DHT22; // 1 byte

    byte battery_pin = 0;  // 0 pur VCC, sinon pin pour analog()    

    int read_int_fromserial();

};

#endif
//
// END OF FILE
//
