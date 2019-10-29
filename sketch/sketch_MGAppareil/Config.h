#ifndef MGCONFIG_config_h
#define MGCONFIG_config_h

#include <RF24Mesh.h>

// Contient des constantes pour tous les modules

// Bus TWI / I2C
// #define BUS_MODE_I2C

// Bus OneWire, utilise pour thermometres DS18B20/DS18S20.
#define BUS_MODE_ONEWIRE

// Senseur DHT
#define DHTPIN 4
#define DHTTYPE 22

// Nouveau sketch

// Prod
#define CANAL_MESH 0x7d  // Prod
#define RATE_MESH RF24_2MBPS

// Test
// #define CANAL_MESH 0x48  // Test
// #define CANAL_MESH 0x65  // Dev
// #define RATE_MESH RF24_250KBPS

#define NODE_ID_DEFAULT 1
#define PIN_LED 6
#define ONE_WIRE_PIN 10

// Configuration pour radio RF24
#define RF24_CE_PIN 7
#define RF24_CSN_PIN 8
#define MESH_RENEW_TIMEOUT 8000

// Version du protocole de transmission NRF24
#define VERSION_PROTOCOLE 7

#define MESH_MASTER_ID 0    

// Power
#define CYCLES_SOMMEIL 5
#define BATTERY_PIN_VCC 0


// EEPROM
#define ADDRESS_SENSEUR 9
#define ADDRESS_UUID 15


#endif
//
// END OF FILE
//

