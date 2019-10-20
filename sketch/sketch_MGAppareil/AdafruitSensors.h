#ifndef AdafruitSensors_config_h
#define AdafruitSensors_config_h

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include "MGAppareilsProt.h"

class MilleGrillesAdafruitSensors : public FournisseurLectureTP {

  public:
    void begin();
    void lire();
    int temperature();
    uint16_t pression();

  private:
    Adafruit_BMP085_Unified _bmp = Adafruit_BMP085_Unified(10085);
    boolean _bmpActif = false;
    int _temperature;
    uint16_t _pression;

};


#endif
//
// END OF FILE
//

