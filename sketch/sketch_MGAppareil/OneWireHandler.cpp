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
    Serial.print("Device family is not recognized: 0x");
    Serial.println(_addr[0],HEX);
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

//int OneWireHandler::lire_temperature() {
//  int temperature = NO_TEMP;
//  int HighByte, LowByte, TReading, SignBit, Tc_100, Whole, Fract;
//
//  LowByte = _data[0];
//  HighByte = _data[1];
//  TReading = (HighByte << 8) + LowByte;
//  SignBit = TReading & 0x8000;  // test most sig bit
//  if (SignBit) // negative
//  {
//    TReading = (TReading ^ 0xffff) + 1; // 2's comp
//  }
//  Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25
//
//  Whole = Tc_100 / 100;  // separate off the whole and fractional portions
//  Fract = Tc_100 % 100;
//
//  if (SignBit) // If its negative
//  {
//     Serial.print("-");
//  }
//  Serial.print(Whole);
//  Serial.print(".");
//  if (Fract < 10)
//  {
//     Serial.print("0");
//  }
//  Serial.println(Fract);
//
//  return temperature;
//}


