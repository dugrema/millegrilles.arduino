// Contient des constantes pour tous les modules
#ifndef MGCONFIG_config_h
#define MGCONFIG_config_h

#include <RF24.h>

// Modes pour debugger ou travail developpement
#define MG_DEV
// #define MG_INT

#define LOGGING_DEV

#define MG_DEV_TEST_BATTERIE

// ------------------------------------------------------
// Appareils

// Bus TWI / I2C
// #define BUS_MODE_I2C

// Bus OneWire, utilise pour thermometres DS18B20/DS18S20.
#define BUS_MODE_ONEWIRE
#define ONE_WIRE_PIN 10

// Senseur DHT
//#define DHTPIN 4
//#define DHTTYPE 22
// ------------------------------------------------------

// ------------------------------------------------------
// Radio
// Canal pour la radio
#ifdef MG_DEV
  #define RADIO_CANAL 0x0c  // Dev
#elif defined(MG_INT)
  #define RADIO_CANAL 0x24  // Test
#else
  #define RADIO_CANAL 0x5e  // Prod
#endif

// Configuration pour radio RF24
#define DATA_RATE RF24_250KBPS
#define RF24_CE_PIN 7
#define RF24_CSN_PIN 8
#define RF24_IRQ_PIN 3
#define RF24_RETRANSMISSIONS 15
// ------------------------------------------------------

#define NODE_ID_DEFAULT 1
#define PIN_LED 6
#define RANDOM_LOWENTROPY_MAXCOUNT 32

// Version du protocole de transmission NRF24
#define VERSION_PROTOCOLE 9

#define SERVER_ADDR 0x3141CA49CALL
#define BROADCAST_DHCP_LISTEN 0x290E92548BLL

// ------------------------------------------------------
// Power
#define CYCLES_SOMMEIL 2
#define BATTERY_PIN_VCC 0
// #define BATTERY_PIN_VCC A0
// ------------------------------------------------------

// ------------------------------------------------------
// EEPROM
// Note: E2END + 1 - 48 (0x3D0 +) est utilise par lib RNG (random numbers, entropy pool)

// UUID de l'appareil - 16 bytes - 0x00F a 0x01E
#define EEPROM_UUID 0x00f

// Mode pairing - 1 byte - 0xFF pas init, 0x01 adresse serveur et cle secrete sauvegardee
#define EEPROM_MODE_PAIRING 0x00a

// Adresse du serveur (pairing) - 3 bytes (24 bits) - 0x00B a 0x00D
#define EEPROM_ADRESSE_SERVEUR 0x00B

// Cle secrete - 32 bytes - 0x020 a 0x03F
#define EEPROM_CLE_SECRETE 0x020
// ------------------------------------------------------

#endif
//
// END OF FILE
//
