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

#define RADIO_CANAL 0x5e  // Prod
// #define RADIO_CANAL 0x24  // Test
// #define RADIO_CANAL 0x0c  // Dev

#define NODE_ID_DEFAULT 1
#define PIN_LED 6
#define ONE_WIRE_PIN 10

// Configuration pour radio RF24
#define RF24_CE_PIN 7
#define RF24_CSN_PIN 8

// Version du protocole de transmission NRF24
#define VERSION_PROTOCOLE 8

#define SERVER_ADDR 0x3141CA49CALL
#define BROADCAST_DHCP_LISTEN 0x290E92548BLL

// Power
#define CYCLES_SOMMEIL 7
#define BATTERY_PIN_VCC 0


// EEPROM
#define ADDRESS_SENSEUR 9
#define ADDRESS_UUID 15


#endif
//
// END OF FILE
//
