#include <printf.h>    // Fix pour radio.printDetails()
#include <SPI.h>
#include "RF24.h"

#define BUFFER_SIZE 32

uint64_t addr_broadcast = 0x290E92548BLL;

RF24 radio(7,8);                           // Set up nRF24L01 radio on SPI bus plus pins 7 & 8

void setup(void) {
  printf_begin();  // Fix pour radio.printDetails()
  Serial.begin(115200);
  Serial.println(F("C'est parti"));

  radio.begin();                           // Setup and configure rf radio
  radio.setChannel(0x24);
  radio.setPALevel(RF24_PA_MAX);           // If you want to save power use "RF24_PA_MIN" but keep in mind that reduces the module's range
//  radio.setDataRate(RF24_1MBPS);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(1);                     // Ensure autoACK is enabled
  radio.setRetries(15,1);                  // Optionally, increase the delay between retries & # of retries
  // radio.enableDynamicPayloads();
  
  radio.setCRCLength(RF24_CRC_16);          // Use 8-bit CRC for performance

  radio.openReadingPipe(1, addr_broadcast);

  radio.printDetails();                    // Dump the configuration of the rf unit for debugging
  
  // radio.powerUp();                         //Power up the radio
  radio.startListening();                  // Start listening

  Serial.println(F("Debut"));
}

void loop() {
   byte data[32];
   while(radio.available()){       
    radio.read(&data,32);
    Serial.print(F("Recu V"));
    Serial.print(data[0]);
    Serial.print(F(" addr: 0x"));
    for(byte i=0; i<32; i++) {
      printHex(data[i]);
    }
    Serial.println("");
    
   }

}

void printHex(byte val) {
  if(val < 16) {
    Serial.print(val < 16 ? "0" : "");
  }
  Serial.print(val, HEX);
}
