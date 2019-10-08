#ifndef AdafruitSensors_config_h
#define AdafruitSensors_config_h

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

class MilleGrillesAdafruitSensors {

  public:
    bool begin();

  private:
    Adafruit_BMP085_Unified _bmp = Adafruit_BMP085_Unified(10085);
    boolean _bmpActif = false;

};


#endif
//
// END OF FILE
//
