#ifndef onewireHandler_config_h
#define onewireHandler_config_h

#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include "MGAppareilsProt.h"

class OneWireHandler: public FournisseurLectureOneWire
{
  public:
    OneWireHandler(byte pin) {
      _ds = OneWire(pin);
    }
    void lire();
    bool hasNext();
    byte nombreSenseurs();
    
    byte* adresse(); // byte[8]
    byte* data();  // byte[12]
    

  private:
    OneWire _ds;
    byte _present;
    byte _data[12];
    byte _addr[8];

    bool lireData(int attente);
    int lire_temperature();
    
};

#endif
//
// END OF FILE
//
