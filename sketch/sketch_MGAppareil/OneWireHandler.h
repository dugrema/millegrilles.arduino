#ifndef onewireHandler_config_h
#define onewireHandler_config_h

#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>

#define ONE_WIRE_PIN 10

class OneWireHandler
{
  public:

  private:
    OneWire ds = OneWire(ONE_WIRE_PIN);
    
};

#endif
//
// END OF FILE
//
