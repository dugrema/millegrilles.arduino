#include "OneWireHandler.h"
#include <Arduino.h>

byte* OneWireHandler::adresse() {
  return _addr;
}

byte* OneWireHandler::data() {
  return _data;
}

byte OneWireHandler::nombreSenseurs() {
  return 1;
}

void OneWireHandler::lire() {
  _present = 0;

//  Serial.println(F("Recherche sur bus OneWire"));

  _ds.reset_search();
  if ( !_ds.search(_addr)) {
//    Serial.print("No more addresses.\n");
    _ds.reset_search();
    memset(_data, 0xFF, sizeof(_data));  // Reset data pour eviter de transmettre vieille valeur
    return;
  }

//  Serial.print("R=");
//  for( byte i = 0; i < 8; i++) {
//    Serial.print(_addr[i], HEX);
//    Serial.print(" ");
//  }

  if ( OneWire::crc8( _addr, 7) != _addr[7]) {
//    Serial.print("CRC is not valid!\n");
    memset(_data, 0xFF, sizeof(_data));  // Reset data pour eviter de transmettre vieille valeur
    return;
  }

  if ( _addr[0] == 0x10 || _addr[0] == 0x28) {
    lireData(1000);
    // lire_temperature();
  }
  else {
    #ifdef LOGGING_DEV
      Serial.print("Device family is not recognized: 0x");
      Serial.println(_addr[0],HEX);
    #endif
    return;
  }

}

bool OneWireHandler::lireData(int attente) {
  _ds.reset();
  _ds.select(_addr);
  _ds.write(0x44,0);         // start conversion, with no parasite (all thermometers at the same time)
  // _ds.write(0x44,1);      // start conversion, with parasite power on at the end

  delay(attente);     // maybe 750ms is enough, maybe not
  _ds.depower();       // we might do a ds.depower() here, but the reset will take care of it.

  byte present = _ds.reset();
  _ds.select(_addr);    
  _ds.write(0xBE);         // Read Scratchpad

//  Serial.print("P=");
//  Serial.print(present,HEX);
//  Serial.print(" ");
  for ( byte i = 0; i < 9; i++) {           // we need 9 bytes
    _data[i] = _ds.read();
//    Serial.print(_data[i], HEX);
//    Serial.print(" ");
  }
//  Serial.print(" CRC=");
//  Serial.print( OneWire::crc8( _data, 8), HEX);
//  Serial.println();
}
