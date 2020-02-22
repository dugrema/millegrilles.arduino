#ifndef MGCONFIG_config_h
#define MGCONFIG_config_h

#include <RF24.h>

// Contient des constantes pour tous les modules

// Bus TWI / I2C
// #define BUS_MODE_I2C

// Bus OneWire, utilise pour thermometres DS18B20/DS18S20.
#define BUS_MODE_ONEWIRE

// Senseur DHT
// #define DHTPIN 4
// #define DHTTYPE 22

// Nouveau sketch

#define DATA_RATE RF24_250KBPS

// #define RADIO_CANAL 0x5e  // Prod
// #define RADIO_CANAL 0x24  // Test
#define RADIO_CANAL 0x0c  // Dev

#define NODE_ID_DEFAULT 1
#define PIN_LED 6
#define ONE_WIRE_PIN 10

// Configuration pour radio RF24
#define RF24_CE_PIN 7
#define RF24_CSN_PIN 8

// Version du protocole de transmission NRF24
#define VERSION_PROTOCOLE 9

#define SERVER_ADDR 0x3141CA49CALL
#define BROADCAST_DHCP_LISTEN 0x290E92548BLL

// Power
#define CYCLES_SOMMEIL 7
#define BATTERY_PIN_VCC 0


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

#endif
//
// END OF FILE
//
