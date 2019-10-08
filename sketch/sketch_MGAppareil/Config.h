#ifndef MGCONFIG_config_h
#define MGCONFIG_config_h

// Contient des constantes pour tous les modules

// ******************************* UUID *******************************
// Generer un nouveau uuid pour chaque appareil
#define UUID_NOEUD 0xed,0x6f,0x2a,0x6a,0xe9,0x2a,0x11,0xe9,0x95,0xe3,0x0,0x15,0x5d,0x1,0x1f,0x9
// ******************************* UUID *******************************

// Nouveau sketch
#define CANAL_MESH 87
#define NODE_ID_DEFAULT 1
#define PIN_LED 6
// #define CYCLES_SOMMEIL 1


#define RF24_CE_PIN 7
#define RF24_CSN_PIN 8

// Version du protocole de transmission NRF24
#define VERSION_PROTOCOLE 7

#define MESH_MASTER_ID 0    
#define NO_TEMP -32768
#define NO_PRESSURE 0xFF
#define NO_HUMIDITY 0XFF

// Power
#define CYCLES_SOMMEIL 1
#define BATTERY_PIN_VCC 0

#endif
//
// END OF FILE
//
