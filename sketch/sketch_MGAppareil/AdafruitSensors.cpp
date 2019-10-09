#include "AdafruitSensors.h"

void MilleGrillesAdafruitSensors::begin() {
   _bmpActif = _bmp.begin();
}

void MilleGrillesAdafruitSensors::lire() {
  uint16_t pression = 0xFFFF;
  if( _bmpActif ) {
    sensors_event_t event;
    _bmp.getEvent(&event);
    
    if( event.temperature ) {
        
      float temperatureFloat;
      _bmp.getTemperature(&temperatureFloat);
      _temperature = int(temperatureFloat*10.0);
        
    } else {
      _temperature = NO_TEMP;
    }
    
    if( event.pressure ) {
      
      float pressureFloat;
      _bmp.getPressure(&pressureFloat);
      _pression = int((pressureFloat/10.0));
      
    } else {
      _pression = NO_PRESSURE;
    }
    
  } else {
    _temperature = NO_TEMP;
    _pression = NO_PRESSURE;
  }

}

int MilleGrillesAdafruitSensors::temperature() {
  return _temperature;
}

uint16_t MilleGrillesAdafruitSensors::pression() {
  return _pression;
}

